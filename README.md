# 🧠 Smart Room Control System
### Modeling, Simulation and Embedded Control using STM32 and Python

---

## 📘 Overview
This project implements a **closed-loop environmental control system** for a single room.
It combines **a simulated plant in Python** with a **real embedded controller (STM32F767ZI)**.
The goal is to regulate **temperature**, **humidity**, and **CO₂ concentration** using:
- A **heater (PWM-controlled)**, and  
- A **ventilation fan (PWM-controlled)**.

Both systems communicate over **UART (USB Virtual COM Port)**.
A **graphical user interface (GUI)** in Python allows visualization and adjustment of setpoints in real-time.

---

## ⚙️ System Architecture
```
          ┌────────────────────────────┐
          │        Python Plant        │
          │ (Simulation + GUI + UART)  │
          └────────────┬───────────────┘
                       │ UART (USB)
                       ▼
          ┌────────────────────────────┐
          │       STM32F767ZI MCU      │
          │ (Kalman Filter + MPC)      │
          └────────────────────────────┘
```

---

## 🧩 Components
| Module | Description |
|--------|-------------|
| `python_plant/` | Simulated environment, UART communication, and GUI. |
| `stm32_controller/` | STM32 firmware with Kalman Filter + MPC. |
| `docs/latex/` | Overleaf-ready LaTeX documentation (main.tex + references). |
| `utils/` | Auxiliary scripts for calibration and testing. |

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
| GUI | Streamlit / Tkinter / PyQt5 |
| Documentation | Overleaf (LaTeX) |

---

## 🧠 Control Algorithms
- **Kalman Filter:** Estimates hidden states (humidity, CO₂) from noisy sensors.  
- **Model Predictive Control (MPC):** Minimizes cost function to reach setpoints with constraints on PWM.

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

## 🧩 Directory Structure
```
smart-room-control/
│
├── README.md
├── CHECKLIST.md
├── .gitignore
│
├── docs/
│   ├── latex/
│   │   ├── main.tex
│   │   ├── references.bib
│   │   └── figures/
│   └── diagrams/
│
├── python_plant/
│   ├── main.py
│   ├── model.py
│   ├── comms.py
│   ├── gui/
│   │   ├── app.py
│   │   └── assets/
│   └── logs/
│
├── stm32_controller/
│   ├── Core/
│   ├── Drivers/
│   ├── Middlewares/
│   └── README.md
│
└── utils/
    ├── calibration/
    └── testing/
```

---

## 🚀 Getting Started
### 1️⃣ Clone the repository
```bash
git clone https://github.com/<your-username>/smart-room-control.git
cd smart-room-control
```

### 2️⃣ Run the Python plant
```bash
cd python_plant
python3 main.py
```

### 3️⃣ Flash STM32 firmware
Compile and upload firmware from `stm32_controller/` using STM32CubeIDE.

### 4️⃣ Start the GUI
```bash
cd python_plant/gui
streamlit run app.py
```

---

## 🧪 Testing Procedure
1. Run Python plant and GUI.  
2. Connect STM32 board via USB.  
3. Verify UART data exchange.  
4. Observe temperature and CO₂ tracking.  
5. Adjust MPC weights and noise parameters.

---

## 📚 Documentation
Full modeling and theoretical details are available in `docs/latex/main.tex` (Overleaf project).

---

## 🧑‍💻 Author
**Juan Marin**  
Control Engineer / Embedded Developer  
juandiegomarin428@gmail.com

---

## 📄 License
MIT License © 2025 Juan Marin
