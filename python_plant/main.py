"""
Python Plant Runner for Smart Room Control System
-------------------------------------------------
Simulates the smart room model, optionally communicates with the STM32 controller
via UART, logs results to CSV, and provides live plotting.

Usage examples:
  # Offline simulation, Ts=10s, 10 minutes
  python3 main.py --mode offline --Ts 10 --duration 600

  # Online mode with STM32 on COM6 @115200 baud
  python3 main.py --mode online --port COM6 --baud 115200 --Ts 1

  # Disable live plotting
  python3 main.py --mode offline --no-plot
"""

from __future__ import annotations
import argparse
import csv
import time
from pathlib import Path
from collections import deque
from typing import Optional, Tuple

# Project modules
from model import SmartRoomModel
from comms import SerialManager

# Live plotting
import matplotlib
# Use TkAgg on Windows (comment out if unavailable)
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt


def parse_args():
    """Parse command-line arguments."""
    ap = argparse.ArgumentParser(description="Python Plant for Smart Room Control System")
    ap.add_argument('--mode', choices=['online', 'offline'], default='offline',
                    help='online: UART with STM32, offline: simulation only')
    ap.add_argument('--port', type=str, required=False,
                    help='UART port (e.g., COM6 on Windows)')
    ap.add_argument('--baud', type=int, default=115200, help='Baud rate')
    ap.add_argument('--Ts', type=float, default=10.0, help='Sampling time [s]')
    ap.add_argument('--duration', type=float, default=600.0, help='Simulation duration [s]')
    ap.add_argument('--N', type=float, default=2.0, help='Occupancy [persons]')
    ap.add_argument('--To', type=float, default=10.0, help='Outdoor temperature [°C]')
    ap.add_argument('--wo', type=float, default=0.004, help='Outdoor absolute humidity [kg/kg]')
    ap.add_argument('--co', type=float, default=420.0, help='Outdoor CO₂ concentration [ppm]')
    ap.add_argument('--logdir', type=str, default='python_plant/logs', help='Directory for CSV logs')
    ap.add_argument('--no-plot', action='store_true', help='Disable live plotting')
    ap.add_argument('--window', type=int, default=600, help='Number of samples in each plot window')
    ap.add_argument('--init-uh', type=float, default=1, help='Initial heater duty [0..1]')
    ap.add_argument('--init-uf', type=float, default=1, help='Initial fan duty [0..1]')
    ap.add_argument('--heater-scale', type=float, default=1.0,
                    help='Multiplier for heater max power Ph')
    ap.add_argument('--fan-scale', type=float, default=1.0,
                    help='Multiplier for fan max flow q_max')
    ap.add_argument('--sim-speed', type=float, default=1.0,
                    help='Simulated seconds per real second (time acceleration)')
    return ap.parse_args()


class LivePlot:
    """
    LivePlot class
    --------------
    Displays each variable in its own subplot with a shared X-axis (time).
    This provides clear separation between temperature, humidity, CO2, etc.
    """

    def __init__(self, window_samples: int = 600):
        self.window = window_samples

        # Buffers (deques automatically discard old samples)
        self.t  = deque(maxlen=window_samples)
        self.T  = deque(maxlen=window_samples)
        self.w  = deque(maxlen=window_samples)
        self.c  = deque(maxlen=window_samples)
        self.RH = deque(maxlen=window_samples)
        self.uh = deque(maxlen=window_samples)
        self.uf = deque(maxlen=window_samples)

        # Create subplots (one row per variable)
        self.fig, self.axes = plt.subplots(6, 1, figsize=(10, 10), sharex=True)
        self.fig.suptitle("Smart Room Live Signals", fontsize=14)

        # Line handles
        self.l_T,  = self.axes[0].plot([], [], label="T [°C]", color="tab:red")
        self.l_w,  = self.axes[1].plot([], [], label="w [kg/kg]", color="tab:blue")
        self.l_c,  = self.axes[2].plot([], [], label="CO₂ [ppm]", color="tab:green")
        self.l_RH, = self.axes[3].plot([], [], label="RH [%]", color="tab:orange")
        self.l_uh, = self.axes[4].plot([], [], label="uh (heater)", color="tab:purple")
        self.l_uf, = self.axes[5].plot([], [], label="uf (fan)", color="tab:brown")

        # Configure each subplot
        labels = ["Temperature", "Abs. Humidity", "CO₂", "Relative Humidity", "Heater Duty", "Fan Duty"]
        ylabels = ["T [°C]", "w [kg/kg]", "CO₂ [ppm]", "RH [%]", "uh [0..1]", "uf [0..1]"]

        for ax, label, ylabel in zip(self.axes, labels, ylabels):
            ax.set_ylabel(ylabel)
            ax.set_title(label)
            ax.grid(True, alpha=0.25)
            ax.legend(loc="upper right")

        self.axes[-1].set_xlabel("Sim Time [s]")
        plt.ion()
        plt.tight_layout(rect=[0, 0, 1, 0.97])
        plt.show(block=False)

    def update(self, t, T, w, c, RH, uh, uf):
        """Append a new sample and refresh all subplots."""
        # Add new values to buffers
        self.t.append(t)
        self.T.append(T)
        self.w.append(w)
        self.c.append(c)
        self.RH.append(RH)
        self.uh.append(uh)
        self.uf.append(uf)

        x = list(self.t)

        # Update each line plot
        self.l_T.set_data(x, list(self.T))
        self.l_w.set_data(x, list(self.w))
        self.l_c.set_data(x, list(self.c))
        self.l_RH.set_data(x, list(self.RH))
        self.l_uh.set_data(x, list(self.uh))
        self.l_uf.set_data(x, list(self.uf))

        # Autoscale Y and set X limits
        for ax, data in zip(self.axes, [self.T, self.w, self.c, self.RH, self.uh, self.uf]):
            ax.relim()
            ax.autoscale_view()
            ax.set_xlim(x[0], x[-1] if len(x) > 1 else 1)

        self.fig.canvas.draw()
        self.fig.canvas.flush_events()


def main():
    """Main entry point."""
    args = parse_args()

    # Initialize model and disturbances
    mdl = SmartRoomModel(
        Ts=args.Ts,
        time_scale=args.sim_speed,           
        heater_scale=args.heater_scale,      
        fan_scale=args.fan_scale             
    )
    mdl.set_disturbances(args.To, args.wo, args.co, args.N)

    # UART manager (only in online mode)
    ser: Optional[SerialManager] = None
    online = (args.mode == 'online')
    if online:
        if not args.port:
            raise ValueError("In online mode, you must provide --port (e.g., --port COM6).")
        ser = SerialManager(args.port, baudrate=args.baud, timeout=max(0.5 * args.Ts, 0.02))
        ser.open()

    # CSV logging setup
    Path(args.logdir).mkdir(parents=True, exist_ok=True)
    timestamp = int(time.time())
    csv_path = Path(args.logdir) / f"run_{timestamp}.csv"
    f = open(csv_path, 'w', newline='')
    writer = csv.writer(f)
    writer.writerow(['t_sim', 'T', 'w', 'c', 'RH', 'uh', 'uf', 'N', 'To', 'wo', 'co', 'mode',
                     'heater_scale', 'fan_scale', 'sim_speed'])
    # Initial control values (heater, fan)
    uh, uf = float(args.init_uh), float(args.init_uf)

    # Create live plot (optional)
    live = None if args.no_plot else LivePlot(window_samples=args.window)

    # Simulation loop parameters
    t_sim  = 0.0
    t_end = float(args.duration)
    next_time = time.time()

    print(f"Starting run: mode={args.mode}, Ts(real)={args.Ts}s, sim_speed={args.sim_speed}x, duration(sim)={t_end}s")
    print(f"Actuator scales: heater_scale={args.heater_scale}, fan_scale={args.fan_scale}")
    print(f"Logging to: {csv_path}")

    try:
        while t_sim  <= t_end:
            # (1) Step model
            _ = mdl.step(uh, uf)
            meas = mdl.measure(add_noise=True)  # returns dict: T, w, c, RH

            # (2) STM32 communication (if online)
            if online and ser is not None:
                try:
                    ser.send_meas(meas['T'], meas['w'], meas['c'], args.N)
                    ctrl: Optional[Tuple[float, float]] = ser.recv_ctrl()
                    if ctrl is not None:
                        uh_new, uf_new = ctrl
                        uh = min(max(uh_new, 0.0), 1.0)
                        uf = min(max(uf_new, 0.0), 1.0)
                except Exception as e:
                    print(f"[WARN] UART step failed: {e}")

            # (3) Log to CSV
            writer.writerow([t_sim, meas['T'], meas['w'], meas['c'], meas['RH'],
                             uh, uf, args.N, args.To, args.wo, args.co, args.mode,
                             args.heater_scale, args.fan_scale, args.sim_speed])

            # (4) Live plotting
            if live is not None:
                live.update(t_sim, meas['T'], meas['w'], meas['c'], meas['RH'], uh, uf)

            # (5) Timing control
            next_time += mdl.Ts
            sleep_dt = next_time - time.time()
            if sleep_dt > 0:
                time.sleep(sleep_dt)
            else:
                # If running late, resync to current time
                next_time = time.time()

            # (6) Advance simulation time
            t_sim += mdl.Ts * (mdl.time_scale if mdl.time_scale > 0 else 1.0)

    except KeyboardInterrupt:
        print("Interrupted by user (Ctrl+C).")
    finally:
        f.close()
        if online and ser is not None:
            try:
                ser.close()
            except Exception:
                pass
        print("Run finished.")


if __name__ == '__main__':
    main()
