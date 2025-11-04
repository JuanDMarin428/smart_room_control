
"""
model.py — Smart Room Control System (Python Plant)
---------------------------------------------------
Lightweight physical model for a single, well-mixed room with states:
  - T [°C]: air temperature
  - w [kg/kg]: absolute humidity
  - c [ppm]: CO2 concentration

Inputs:
  - u_h in [0,1]: heater PWM duty
  - u_f in [0,1]: fan PWM duty

Disturbances (measured or set by scenario):
  - T_o [°C], w_o [kg/kg], c_o [ppm], N [persons]

The airflow q depends on fan command and a stack-effect term that couples
ventilation to the temperature difference (T - T_o), creating cross-coupling
between states — useful for control/estimation realism while staying simple.

This module is intended to be imported by the simulation runner and GUI.
"""

from __future__ import annotations
import numpy as np
from dataclasses import dataclass, asdict
from typing import Dict, Optional, Tuple

@dataclass
class RoomParams:
    # Geometry & air properties
    V: float = 50.0          # [m^3] room volume
    rho: float = 1.2         # [kg/m^3] air density
    cp: float = 1005.0       # [J/(kg·°C)] air specific heat
    P_atm: float = 101325.0  # [Pa] atmospheric pressure

    # Actuators & ventilation
    Ph: float = 1500.0       # [W] base heater max power
    eta_h: float = 0.95      # [-] heater efficiency
    q_max: float = 0.056     # [m^3/s] base max fan flow (~200 m^3/h)
    k_stack: float = 0.002   # [m^3/(s·°C)] stack effect coefficient

    # Human loads (per person, at rest)
    Q_person: float = 75.0       # [W/person] sensible heat
    G_w: float = 1.1e-5          # [kg/s/person] moisture generation
    gamma_c: float = 0.1         # [ppm/s/person] CO2 generation

class SmartRoomModel:
    """Minimal yet realistic room model for control and estimation experiments."""

    def __init__(self,
                 Ts: float = 10.0,
                 x0: Tuple[float, float, float] = (20.0, 0.006, 800.0),
                 params: Optional[RoomParams] = None,
                 sensor_co2: bool = True,
                 rng_seed: Optional[int] = 42,
                 time_scale: float = 1.0,
                 heater_scale: float = 1.0,
                 fan_scale: float = 1.0):
        self.Ts = float(Ts)
        self.params = params if params is not None else RoomParams()
        self.x = np.array(x0, dtype=float)  # [T, w, c]
        self.sensor_co2 = sensor_co2
        self.rng = np.random.default_rng(rng_seed)

        # Scales and TSIM 
        self.time_scale = float(max(time_scale, 0.0))
        self.heater_scale = float(max(heater_scale, 0.0))
        self.fan_scale = float(max(fan_scale, 0.0))


        # Default disturbances (can be updated each step)
        self.d = np.array([10.0, 0.004, 420.0, 2.0], dtype=float)  # [To, wo, co, N]

        # Measurement noise standard deviations (tunable)
        self.noise_std = {
            "T": 0.10,       # °C
            "w": 1e-4,       # kg/kg
            "c": 20.0        # ppm
        }

        self.noise_mult = {
            "T": 1.0,
            "w": 1.0,
            "c": 1.0
        }

    def set_noise_multipliers(self, T: float = 1.0, w: float = 1.0, c: float = 1.0) -> None:
        """Scale the measurement noise std-devs by given multipliers."""
        self.noise_mult["T"] = float(max(T, 0.0))
        self.noise_mult["w"] = float(max(w, 0.0))
        self.noise_mult["c"] = float(max(c, 0.0))

    # ---------- Helpers ----------
    def q_flow(self, u_f: float, T: float, To: float) -> float:
        """Total airflow [m^3/s] as fan + stack effect."""
        p = self.params
        return (p.q_max * self.fan_scale) * float(np.clip(u_f, 0.0, 1.0)) + p.k_stack * (T - To)

    @staticmethod
    def _clip_unit(u: float) -> float:
        return float(np.clip(u, 0.0, 1.0))

    # ---------- Physics ----------
    def f_cont(self, x: np.ndarray, u: np.ndarray, d: np.ndarray) -> np.ndarray:
        """Continuous-time derivatives f(x,u,d)."""
        T, w, c = x
        u_h, u_f = u
        To, wo, co, N = d
        p = self.params

        q = self.q_flow(u_f, T, To)
        alpha = q / p.V

        dT = alpha * (To - T)                  + (p.eta_h * p.Ph * self.heater_scale) / (p.rho * p.cp * p.V) * u_h                  + (p.Q_person) / (p.rho * p.cp * p.V) * N

        dw = alpha * (wo - w)                  + (p.G_w) / (p.rho * p.V) * N

        dc = alpha * (co - c) + p.gamma_c * N

        return np.array([dT, dw, dc], dtype=float)

    def step(self, u_h: float, u_f: float, d: Optional[np.ndarray] = None) -> np.ndarray:
        """Euler-forward integration: x_{k+1} = x_k + Ts * f(x_k,u_k,d_k)."""
        u = np.array([self._clip_unit(u_h), self._clip_unit(u_f)], dtype=float)
        if d is not None:
            self.d = np.array(d, dtype=float)
        dt = self.Ts * (self.time_scale if self.time_scale > 0 else 1.0)  # << aceleración
        self.x = self.x + dt * self.f_cont(self.x, u, self.d)
        return self.x.copy()

    # ---------- Measurement model ----------
    def rh_from_Tw(self, T: float, w: float) -> float:
        """Compute Relative Humidity [-] from temperature [°C] and absolute humidity [kg/kg].
        Uses: w = 0.62198 * p_w / (P - p_w)  =>  p_w = w * P / (w + 0.62198)
              Magnus formula for saturation vapor pressure over water (Pa).
        """
        p = self.params
        pw = (w * p.P_atm) / (w + 0.62198)
        pws = 610.94 * np.exp(17.625 * T / (T + 243.04))  # Pa
        RH = np.clip(pw / pws, 0.0, 1.0)
        return float(RH)

    def measure(self, add_noise: bool = True) -> Dict[str, Optional[float]]:
        """Return a dict with sensor readings. Includes RH for convenience."""
        T, w, c = self.x
        mT = float(T + (self.rng.normal(0, self.noise_std['T'] * self.noise_mult['T']) if add_noise else 0.0))
        mw = float(w + (self.rng.normal(0, self.noise_std['w'] * self.noise_mult['w']) if add_noise else 0.0))
        if self.sensor_co2:
            mc = float(c + (self.rng.normal(0, self.noise_std['c'] * self.noise_mult['c']) if add_noise else 0.0))
        else:
            mc = None
        RH = self.rh_from_Tw(mT, mw)
        return {"T": mT, "w": mw, "c": mc, "RH": RH}

    # ---------- Linearization ----------
    def linearize(self,
                  xs: Optional[np.ndarray] = None,
                  us: Optional[np.ndarray] = None,
                  ds: Optional[np.ndarray] = None) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """Return discrete-time (A,B,E,C) around (xs,us,ds) using Euler discretization."""
        if xs is None: xs = self.x
        if us is None: us = np.array([0.2, 0.3], dtype=float)
        if ds is None: ds = self.d
        T, w, c = xs
        u_h, u_f = us
        To, wo, co, N = ds
        p = self.params

        q = self.q_flow(u_f, T, To)
        alpha = q / p.V
        beta = p.k_stack / p.V

        # d f / d x
        dfdx = np.array([
            [ -alpha + beta*(To - T),  0.0,                 0.0 ],
            [  beta*(wo - w),         -alpha,              0.0 ],
            [  beta*(co - c),          0.0,               -alpha ]
        ], dtype=float)

        # d f / d u
        dfdu = np.zeros((3,2), dtype=float)
        dfdu[:,0] = [(p.eta_h*p.Ph*self.heater_scale)/(p.rho*p.cp*p.V), 0.0, 0.0]
        dfdu[:,1] = [ ((p.q_max*self.fan_scale)/p.V)*(To - T),
                      ((p.q_max*self.fan_scale)/p.V)*(wo - w),
                      ((p.q_max*self.fan_scale)/p.V)*(co - c) ]

        # d f / d d  where d = [To, wo, co, N]
        dfdd = np.zeros((3,4), dtype=float)
        dfdd[0,0] =  alpha - beta*(To - T)
        dfdd[1,0] = -beta*(wo - w)
        dfdd[2,0] = -beta*(co - c)

        dfdd[1,1] =  alpha
        dfdd[2,2] =  alpha

        dfdd[0,3] = (p.Q_person)/(p.rho*p.cp*p.V)
        dfdd[1,3] = (p.G_w)/(p.rho*p.V)
        dfdd[2,3] = p.gamma_c

        # Discrete-time matrices (Euler)
        dt = self.Ts * (self.time_scale if self.time_scale > 0 else 1.0)
        A = np.eye(3) + dt * dfdx
        B = dt * dfdu
        E = dt * dfdd
        C = np.eye(3)  # measure [T,w,c] by default

        return A, B, E, C

    # ---------- Utilities ----------
    def set_disturbances(self, To: float, wo: float, co: float, N: float) -> None:
        self.d = np.array([To, wo, co, N], dtype=float)

    def set_state(self, T: float, w: float, c: float) -> None:
        self.x = np.array([T, w, c], dtype=float)

    def get_state(self) -> Dict[str, float]:
        T, w, c = self.x
        return {"T": float(T), "w": float(w), "c": float(c)}

    def params_dict(self) -> Dict[str, float]:
        return asdict(self.params)

# ---- Smoke test ----
if __name__ == "__main__":
    mdl = SmartRoomModel(Ts=10.0)
    # 1 minute simulation with moderate heating and ventilation
    for k in range(6):
        x = mdl.step(u_h=0.4, u_f=0.3)
        y = mdl.measure()
        print(f"k={k:02d} x={x} y={y}")
    A,B,E,C = mdl.linearize()
    print("A=\n", A)
    print("B=\n", B)
    print("E=\n", E)
    print("C=\n", C)
