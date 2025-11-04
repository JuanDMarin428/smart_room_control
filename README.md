# 🧠 Smart Room Control System
### Modeling, Simulation and Embedded Control using STM32 and Python

---

## 📘 Overview
This project implements a **closed-loop environmental control system** for a single room.  
It combines a **simulated plant in Python** with a **real embedded controller (STM32F767ZI)**.  
The objective is to regulate **temperature**, **humidity**, and **CO₂ concentration** using:  
- A **heater (PWM-controlled)**  
- A **ventilation fan (PWM-controlled)**  

Both systems communicate via **UART (USB Virtual COM Port)**.  

---

## ⚙️ System Architecture
```
          ┌────────────────────────────┐
          │        Python Plant        │
          │  (Simulation + UART I/O)   │
          └────────────┬───────────────┘
                       │ UART (USB)
                       ▼
          ┌────────────────────────────┐
          │       STM32F767ZI MCU      │
          │ (Kalman Filter + Control)  │
          └────────────────────────────┘
```

---

## 🧩 Components
| Module | Description |
|--------|-------------|
| `python_plant/` | Python-based plant simulation and UART communication with STM32. |
| `stm32_controller/smart_room_control_v1/` | STM32 firmware with Kalman filter and closed-loop control. |
| `docs/` | Documentation and detailed modeling report. |

---

## 🧮 Mathematical Model Summary

State vector:
\[
x =
\begin{bmatrix}
T \\ w \\ c
\end{bmatrix}
\]
where:  
- \( T \) [°C]: Room air temperature  
- \( w \) [kg/kg]: Absolute humidity  
- \( c \) [ppm]: CO₂ concentration  

Inputs:
\[
u =
\begin{bmatrix}
u_h \\ u_f
\end{bmatrix}
\]
- \( u_h \): Heater PWM  
- \( u_f \): Fan PWM  

Disturbances:
\[
d =
\begin{bmatrix}
T_o \\ w_o \\ c_o \\ N
\end{bmatrix}
\]

Continuous-time dynamics:
\[
\begin{aligned}
\dot{T} &= \frac{q}{V}(T_o - T) + \frac{\eta_h P_h}{\rho c_p V}u_h + \frac{Q_{pers}}{\rho c_p V}N \\
\dot{w} &= \frac{q}{V}(w_o - w) + \frac{G_w}{\rho V}N \\
\dot{c} &= \frac{q}{V}(c_o - c) + \gamma_c N
\end{aligned}
\]

where:  
\( q = q_{max}u_f + k_{stack}(T - T_o) \)

---

## 🧰 Software Stack
| Component | Technology |
|------------|-------------|
| Simulation | Python (NumPy, Matplotlib, PySerial) |
| Control | STM32CubeIDE (C with CMSIS/FreeRTOS optional) |
| Communication | UART (115200 bps, ASCII packets) |
| Documentation | LaTeX / PDF report |

---

## 🧠 Control Algorithms
- **Kalman Filter:** Estimates states (Temp, humidity and CO₂) from noisy measurements.  
- **Closed-loop control:** Applies heater and fan actuation to maintain setpoints.  

---

## 🔌 Communication Protocol
**Packet format (ASCII via UART):**  
- Python → STM32: `<MEAS, T, w, c, N>`  
- STM32 → Python: `<CTRL, uh, uf>`

**Example:**  
```
<MEAS, 22.4, 0.0062, 750, 2>
<CTRL, 0.45, 0.70>
```

**Baud rate:** 115200 bps  
**Data bits:** 8, **Stop bits:** 1, **Parity:** None  

---

## 🚀 Running the Simulation

### Command used for the online simulation
```bash
python python_plant/main.py --mode online --port COM11 --baud 115200 --Ts 0.01 --duration 10 --sim-speed 10 --noise-mult-T 20 --noise-mult-w 2 --noise-mult-c 20 --heater-scale 5 --fan-scale 20
```

This configuration corresponds to a **real-time experiment** with the STM32F767ZI controller.  

---

## 📊 Results and Documentation
All results, conclusions, and detailed mathematical modeling are available in:  
📄 `docs/smart_control_room.pdf`  

---

## 🧩 Directory Structure
```
smart-room-control/
│
├── README.md
│
├── docs/
│   └── smart_control_room.pdf
│
├── python_plant/
│   ├── main.py
│   ├── model.py
│   └── comms.py
│
└── stm32_controller/
    └── smart_room_control_v1/
```

---

## 🧪 Future Work
- Test and validate **varying occupancy scenarios (N variable)**.  
- Extend system to include **humidity and CO₂ actuators**.  
- Evaluate **long-term performance and energy efficiency**.  

---

## 🧑‍💻 Author
**Juan Marin**  
Control Engineer / Embedded Developer  
📧 juandiegomarin428@gmail.com  

---

## 📄 License
MIT License © 2025 Juan Marin
