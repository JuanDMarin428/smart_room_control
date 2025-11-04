# comms.py — SerialManager with structured ASCII frames
# ASCII protocol with newline-terminated frames:
#   TX (PC -> MCU): <MEAS, T, w, c, N>\n
#   RX (MCU -> PC): <CTRL, uh, uf>\n      (optional if MCU sends control)
#                   <KF,   T, w, c, N>\n  (Kalman-filtered values from MCU)

from __future__ import annotations
import serial
import threading
import time
from typing import Optional, Tuple, Deque
from collections import deque


def _try_float(x: str) -> Optional[float]:
    """Safely convert a string to float, returning None on failure."""
    try:
        return float(x.strip())
    except Exception:
        return None


def _parse_frame(line: str):
    """Validate and parse an ASCII frame enclosed in < > brackets."""
    s = line.strip()
    if not (s.startswith("<") and s.endswith(">")):
        return None, None
    s = s[1:-1]  # remove < >
    parts = [p.strip() for p in s.split(",")]
    if not parts:
        return None, None
    tag = parts[0]
    vals = parts[1:]
    return tag, vals


class SerialManager:
    """
    Serial communication manager for structured ASCII frames.
    Handles asynchronous reception and non-blocking parsing.

    Protocol:
        PC -> MCU:
            <MEAS, T, w, c, N>\n

        MCU -> PC:
            <CTRL, uh, uf>\n      (optional control feedback)
            <KF,   T, w, c, N>\n  (Kalman-filtered values)
    """

    def __init__(self, port: str, baudrate: int = 115200, timeout: float = 0.05):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.ser: Optional[serial.Serial] = None

        # Non-blocking queues for parsed messages
        self._ctrl_q: Deque[Tuple[float, float]] = deque(maxlen=8)         # (uh, uf)
        self._kf_q:   Deque[Tuple[float, float, float, float]] = deque(maxlen=128)  # (T, w, c, N)

        # Background reader thread
        self._rx_thread: Optional[threading.Thread] = None
        self._rx_stop = threading.Event()

    # ---------- Public API ----------
    def open(self):
        """Open the serial port and start the receiver thread."""
        self.ser = serial.Serial(self.port, self.baudrate, timeout=self.timeout)
        self._rx_stop.clear()
        self._rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
        self._rx_thread.start()

    def close(self):
        """Stop receiver thread and close serial port."""
        self._rx_stop.set()
        if self._rx_thread and self._rx_thread.is_alive():
            self._rx_thread.join(timeout=0.5)
        if self.ser:
            try:
                self.ser.close()
            except Exception:
                pass
        self.ser = None

    def send_meas(self, T: float, w: float, c: Optional[float], N: float):
        """
        Send a <MEAS,...> frame to the MCU.
        If 'c' is None, NaN is transmitted instead.
        """
        if self.ser is None:
            return
        cval = c if c is not None else float("nan")
        frame = f"<MEAS, {T:.6f}, {w:.8f}, {cval:.3f}, {N:.3f}>\n"
        self.ser.write(frame.encode())

    def recv_ctrl_nowait(self) -> Optional[Tuple[float, float]]:
        """Retrieve (uh, uf) if available, otherwise return None."""
        try:
            return self._ctrl_q.popleft()
        except IndexError:
            return None

    def recv_kf_nowait(self) -> Optional[Tuple[float, float, float, float]]:
        """Retrieve (T, w, c, N) filtered values if available, otherwise None."""
        try:
            return self._kf_q.popleft()
        except IndexError:
            return None

    # ---------- Internal Methods ----------
    def _rx_loop(self):
        """Background thread that continuously reads and parses incoming lines."""
        buf = bytearray()
        while not self._rx_stop.is_set():
            try:
                if self.ser is None:
                    time.sleep(0.01)
                    continue
                data = self.ser.read(self.ser.in_waiting or 1)
                if not data:
                    continue
                buf.extend(data)
                # Process complete lines
                while b'\n' in buf:
                    line, _, rest = buf.partition(b'\n')
                    buf = bytearray(rest)
                    self._handle_line(line.decode(errors="ignore"))
            except Exception:
                # Prevent thread crash; keep reading
                time.sleep(0.01)

    def _handle_line(self, line: str):
        """Parse a single received ASCII frame and push to the correct queue."""
        tag, vals = _parse_frame(line)
        if tag is None:
            return

        if tag.upper() == "CTRL" and len(vals) >= 2:
            uh = _try_float(vals[0]); uf = _try_float(vals[1])
            if uh is not None and uf is not None:
                self._ctrl_q.append((uh, uf))

        elif tag.upper() == "KF" and len(vals) >= 4:
            T = _try_float(vals[0]); w = _try_float(vals[1])
            c = _try_float(vals[2]); N = _try_float(vals[3])
            if None not in (T, w, c, N):
                self._kf_q.append((T, w, c, N))
