# ✅ Project Checklist – Smart Room Control System

A complete step-by-step checklist to track progress from modeling to full integration.

---

## 🧩 Phase 1 – Documentation and Modeling
- [ ] Create **Overleaf project** with the provided LaTeX template.
- [ ] Write introduction and system overview.
- [ ] Define all **variables, actuators, and disturbances**.
- [ ] Derive **continuous-time equations** for temperature, humidity, and CO₂.
- [ ] Define **physical parameters** (room volume, heater power, etc.).
- [ ] Linearize the model around nominal conditions.
- [ ] Discretize system equations for implementation.
- [ ] Document all assumptions clearly (well-mixed air, constant pressure, etc.).

---

## 🧮 Phase 2 – Python Plant (Simulation)
- [ ] Implement `model.py` with continuous and discrete update functions.
- [ ] Define simulation constants and time step `Ts`.
- [ ] Add **sensor noise** and output saturation.
- [ ] Create `comms.py` for UART (PySerial) communication.
- [ ] Send measurements `<MEAS, T, w, c, N>` to STM32.
- [ ] Receive control actions `<CTRL, uh, uf>` from STM32.
- [ ] Log data (CSV) and plot states vs time (Matplotlib).
- [ ] Verify dynamic behavior of temperature, humidity, and CO₂.

---

## ⚙️ Phase 3 – STM32 Controller (Firmware)
- [ ] Create STM32CubeIDE project for **Nucleo-F767ZI**.
- [ ] Configure:
  - [ ] UART (115200 bps, USB CDC Virtual COM Port).
  - [ ] PWM channels for heater and fan.
  - [ ] FreeRTOS (optional, for scheduling).
- [ ] Implement **UART parser** for Python messages.
- [ ] Implement **Kalman Filter**:
  - [ ] Define matrices A, B, C, Q, R.
  - [ ] Compute prediction and update steps.
- [ ] Implement **Model Predictive Control (MPC)**:
  - [ ] Define cost function.
  - [ ] Apply constraints to PWM signals.
  - [ ] Optimize control at each iteration.
- [ ] Send control data `<CTRL, uh, uf>` to Python.
- [ ] Test timing consistency and loop stability.

---

## 🔄 Phase 4 – Integration and Communication Testing
- [ ] Run both Python plant and STM32 firmware.
- [ ] Verify UART communication (no parsing errors).
- [ ] Confirm sampling period consistency.
- [ ] Validate that control commands modify the simulated states.
- [ ] Record temperature and CO₂ responses.

---

## 🖥️ Phase 5 – Graphical User Interface (GUI)
- [ ] Choose GUI framework (Streamlit, Tkinter, or PyQt5).
- [ ] Design layout:
  - [ ] Real-time plots for T, w, and c.
  - [ ] Display PWM signals uh, uf.
  - [ ] Input fields for setpoints.
  - [ ] Status indicators (connected/disconnected).
- [ ] Connect GUI to simulation backend.
- [ ] Implement start/pause/reset controls.
- [ ] Enable CSV export for logs.

---

## 🧪 Phase 6 – System Testing and Validation
- [ ] Perform step tests on temperature and CO₂ setpoints.
- [ ] Tune Kalman Filter noise covariance matrices (Q, R).
- [ ] Tune MPC weights (comfort vs energy usage).
- [ ] Simulate varying occupancy (N).
- [ ] Validate stability and steady-state behavior.
- [ ] Document results with graphs and analysis in LaTeX.

---

## 📘 Phase 7 – Final Documentation and Release
- [ ] Finalize Overleaf documentation (`main.tex`).
- [ ] Include figures and simulation plots.
- [ ] Add source code listings (Python + STM32).
- [ ] Write final discussion and conclusions.
- [ ] Export PDF and link in `README.md`.
- [ ] Push all content to GitHub with tags and changelog.

---

### 🏁 Optional Next Steps
- [ ] Add temperature and CO₂ sensors for hardware-in-the-loop test.
- [ ] Integrate wireless communication (ESP32, BLE, or WiFi).
- [ ] Extend control to multi-zone building model.
- [ ] Evaluate real-time performance vs simulation.

---

📅 **Version:** 1.0  
🧠 **Author:** Juan Marin  
🔧 **Last Updated:** $(date)
