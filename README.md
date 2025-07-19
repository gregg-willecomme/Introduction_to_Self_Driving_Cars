# 🚗 Vehicle Modeling, Control, and CARLA Simulation

This repository presents a complete project on vehicle motion modeling and control, including:

- Two educational Jupyter notebooks
- A final integrated autonomous driving project in the CARLA simulator

---

## 📚 Project Structure

### 1. `Bicycle_Kinematic_Model.ipynb`
A simplified lateral vehicle model using bicycle geometry. It demonstrates the basics of path tracking using kinematic assumptions, without dynamic slip effects.

### 2. `Longitudinal_Vehicle_Model.ipynb`
A second-order longitudinal vehicle model incorporating:
- Engine force
- Aerodynamic drag
- Rolling resistance
- Gravitational slope

It implements a **throttle controller** combining:
- A **PID controller**, tuned via the **Ziegler–Nichols method** (closed-loop tuning using \( K_u \) and \( P_u \))
- A **feedforward term**, assuming a linear relation between acceleration and throttle

A simple **proportional controller** is used for braking.

---

### 3. `CARLA_Project/`
Contains the final autonomous control system implemented in the CARLA simulator, including:

- **Longitudinal control**:
  - PID + feedforward throttle control
  - Proportional brake control
  - Time-based speed profiles
- **Lateral control**:
  - **Pure Pursuit controller**
  - **Stanley controller** (as an alternative)
- **Waypoint tracking**
- **Real-time CSV logging** and live plotting

---

## 🎯 Objectives

- Understand both kinematic and dynamic modeling of vehicles
- Design and tune throttle controllers (feedback + feedforward)
- Implement robust lateral tracking using geometric control methods
- Integrate full control systems into a CARLA simulation

---

## 📈 Features

- Feedforward-based acceleration control
- Ziegler–Nichols PID tuning
- Modular control architecture
- Live plotting and logging
- Supports lateral and longitudinal decoupled control

---

## 🔧 Requirements

- Python 3.6+
- NumPy
- Matplotlib
- Jupyter Notebook
- CARLA Simulator (e.g. 0.9.6 for Coursera setup)


Install dependencies:
```bash
pip install numpy matplotlib notebook


