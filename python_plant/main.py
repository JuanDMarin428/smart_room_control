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
    ap.add_argument('--noise-mult-T', type=float, default=1.0, help='Multiplier for T sensor noise std')
    ap.add_argument('--noise-mult-w', type=float, default=1.0, help='Multiplier for w sensor noise std')
    ap.add_argument('--noise-mult-c', type=float, default=1.0, help='Multiplier for CO2 sensor noise std')
    return ap.parse_args()


class LivePlot:
    """
    LivePlot class
    --------------
    Displays each variable in its own subplot with a shared X-axis (time).
    Shows sensor (solid), KF from MCU (dashed), and true model values (dotted).
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

        # KF (from STM32) buffers
        self.T_kf = deque(maxlen=window_samples)
        self.w_kf = deque(maxlen=window_samples)
        self.c_kf = deque(maxlen=window_samples)

        # True (from model) buffers
        self.T_true = deque(maxlen=window_samples)
        self.w_true = deque(maxlen=window_samples)
        self.c_true = deque(maxlen=window_samples)

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

        # KF overlay lines (dashed).
        self.l_T_kf,  = self.axes[0].plot([], [], linestyle="--", label="T KF [°C]")
        self.l_w_kf,  = self.axes[1].plot([], [], linestyle="--", label="w KF [kg/kg]")
        self.l_c_kf,  = self.axes[2].plot([], [], linestyle="--", label="CO₂ KF [ppm]")

        # True overlay (dotted)
        self.l_T_true,  = self.axes[0].plot([], [], linestyle=":",  label="T true [°C]")
        self.l_w_true,  = self.axes[1].plot([], [], linestyle=":",  label="w true [kg/kg]")
        self.l_c_true,  = self.axes[2].plot([], [], linestyle=":",  label="CO₂ true [ppm]")

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

    def update(self, t, T, w, c, RH, uh, uf,
               T_kf=None, w_kf=None, c_kf=None,
               T_true=None, w_true=None, c_true=None):
        # Buffers base
        self.t.append(t)
        self.T.append(T); self.w.append(w); self.c.append(c); self.RH.append(RH)
        self.uh.append(uh); self.uf.append(uf)

        # KF (append only if provided)
        if T_kf is not None: self.T_kf.append(T_kf)
        if w_kf is not None: self.w_kf.append(w_kf)
        if c_kf is not None: self.c_kf.append(c_kf)

        # True (append only if provided)
        if T_true is not None: self.T_true.append(T_true)
        if w_true is not None: self.w_true.append(w_true)
        if c_true is not None: self.c_true.append(c_true)

        x = list(self.t)

        # Update raw lines
        self.l_T.set_data(x, list(self.T))
        self.l_w.set_data(x, list(self.w))
        self.l_c.set_data(x, list(self.c))
        self.l_RH.set_data(x, list(self.RH))
        self.l_uh.set_data(x, list(self.uh))
        self.l_uf.set_data(x, list(self.uf))

        # Update KF lines (only if we have any samples)
        if len(self.T_kf) > 0:
            self.l_T_kf.set_data(x[-len(self.T_kf):], list(self.T_kf))
        if len(self.w_kf) > 0:
            self.l_w_kf.set_data(x[-len(self.w_kf):], list(self.w_kf))
        if len(self.c_kf) > 0:
            self.l_c_kf.set_data(x[-len(self.c_kf):], list(self.c_kf))

        # True lines (only if we have any samples)
        if len(self.T_true) > 0:
            self.l_T_true.set_data(x[-len(self.T_true):], list(self.T_true))
        if len(self.w_true) > 0:
            self.l_w_true.set_data(x[-len(self.w_true):], list(self.w_true))
        if len(self.c_true) > 0:
            self.l_c_true.set_data(x[-len(self.c_true):], list(self.c_true))

        # Autoscale and redraw
        for ax, data in zip(self.axes, [self.T, self.w, self.c, self.RH, self.uh, self.uf]):
            ax.relim(); ax.autoscale_view()
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
    mdl.set_noise_multipliers(T=args.noise_mult_T, w=args.noise_mult_w, c=args.noise_mult_c)

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
    writer.writerow([
        't_sim',
        'T_meas','w_meas','c_meas','RH',
        'T_kf','w_kf','c_kf',
        'T_true','w_true','c_true',
        'uh','uf','N','To','wo','co','mode',
        'heater_scale','fan_scale','sim_speed'
    ])
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
        # Initialize KF overlay vars
        T_kf = w_kf = c_kf = None

        while t_sim <= t_end:
            # Reset KF overlays for this iteration
            T_kf = w_kf = c_kf = None

            # (1) Step model
            _ = mdl.step(uh, uf)
            meas = mdl.measure(add_noise=True)  # returns dict: T, w, c, RH
            x_true = mdl.get_state()            # true model state

            # (2) STM32 communication (if online)
            if online and ser is not None:
                # 2.1) Send measurement with noise
                try:
                    ser.send_meas(meas['T'], meas['w'], meas['c'], args.N)
                except Exception as e:
                    print(f"[WARN] UART send failed: {e}")
                # 2.2) Read control if available (non-blocking)
                try:
                    ctrl = ser.recv_ctrl_nowait()
                    if ctrl is not None:
                        uh_new, uf_new = ctrl
                        uh = min(max(uh_new, 0.0), 1.0)
                        uf = min(max(uf_new, 0.0), 1.0)
                except Exception:
                    pass
                # 2.3) Read KF if available (non-blocking)
                try:
                    kf_pkt = ser.recv_kf_nowait()  # (T,w,c,N)
                    if kf_pkt is not None:
                        T_kf, w_kf, c_kf, _ = kf_pkt
                except Exception:
                    pass

            # (3) Log to CSV
            writer.writerow([
                t_sim,
                meas['T'], meas['w'], meas['c'], meas['RH'],
                T_kf, w_kf, c_kf,
                x_true['T'], x_true['w'], x_true['c'],
                uh, uf, args.N, args.To, args.wo, args.co, args.mode,
                args.heater_scale, args.fan_scale, args.sim_speed
            ])

            # (4) Live plotting
            if live is not None:
                live.update(
                    t_sim,
                    meas['T'], meas['w'], meas['c'], meas['RH'],
                    uh, uf,
                    T_kf=T_kf, w_kf=w_kf, c_kf=c_kf,
                    T_true=x_true['T'], w_true=x_true['w'], c_true=x_true['c']
                )

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
