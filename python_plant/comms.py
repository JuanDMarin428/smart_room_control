
"""comms.py — UART communication helpers for Smart Room Control System.
Protocol (ASCII over UART, 115200 8N1):
  Python -> STM32: <MEAS, T, w, c, N>
  STM32 -> Python: <CTRL, uh, uf>
"""
from __future__ import annotations
import time
from typing import Optional, Tuple

try:
    import serial  # pyserial
except ImportError as e:
    raise SystemExit("pyserial is required. Install with: pip install pyserial") from e

class SerialManager:
    def __init__(self, port: str, baudrate: int = 115200, timeout: float = 0.05):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.ser = None

    def open(self):
        self.ser = serial.Serial(self.port, self.baudrate, timeout=self.timeout)
        time.sleep(0.2)

    def close(self):
        if self.ser and self.ser.is_open:
            self.ser.close()

    def send_meas(self, T: float, w: float, c: float, N: float) -> None:
        packet = f"<MEAS, {T:.3f}, {w:.6f}, {c:.1f}, {N:.0f}>\n"
        self._write(packet)

    def recv_ctrl(self) -> Optional[Tuple[float, float]]:
        line = self._readline()
        if not line:
            return None
        line = line.strip()
        if not (line.startswith('<') and line.endswith('>')):
            return None
        body = line[1:-1]
        parts = [p.strip() for p in body.split(',')]
        if len(parts) != 3 or parts[0].upper() != 'CTRL':
            return None
        try:
            uh = float(parts[1])
            uf = float(parts[2])
            uh = max(0.0, min(1.0, uh))
            uf = max(0.0, min(1.0, uf))
            return (uh, uf)
        except ValueError:
            return None

    def _write(self, s: str) -> None:
        if not self.ser or not self.ser.is_open:
            raise RuntimeError("Serial port not open")
        self.ser.write(s.encode('ascii', errors='ignore'))

    def _readline(self) -> Optional[str]:
        if not self.ser or not self.ser.is_open:
            raise RuntimeError("Serial port not open")
        try:
            line = self.ser.readline().decode('ascii', errors='ignore')
            return line if line else None
        except Exception:
            return None
