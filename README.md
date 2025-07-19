# Introduction_to_Self_Driving_Cars
# 🚗 Longitudinal Vehicle Dynamics and Control

This repository contains a simulation of a vehicle's longitudinal dynamics and its control system using Python and Jupyter Notebooks. It is based on simplified vehicle modeling and includes a feedforward+feedback PID throttle controller.

---

## 📂 Contents

- `Longitudinal_Vehicle_Model.ipynb`  
  Simulates the vehicle's longitudinal dynamics using a second-order model with engine force, drag, rolling resistance, and slope effects.

- `Controller_Implementation.ipynb`  
  Implements a feedforward + PID-based throttle control strategy to track a desired velocity profile, based on the dynamic model.

---

## 🎯 Objectives

- Model the longitudinal dynamics of a ground vehicle.
- Compute forces acting on the vehicle (aerodynamic, rolling resistance, gravitational).
- Implement a feedforward control based on inverse dynamics.
- Tune a PID controller to improve tracking performance.
- Visualize speed tracking and control signals.

---

## 📈 Example Results

Plots include:
- Vehicle speed vs desired speed
- Control inputs (throttle)
- External forces acting on the system

---

## 🔧 Requirements

- Python 3.6+
- NumPy
- Matplotlib
- Jupyter Notebook

You can install the required packages with:

```bash
pip install numpy matplotlib notebook
