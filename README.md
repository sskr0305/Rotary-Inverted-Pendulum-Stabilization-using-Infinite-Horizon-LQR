# Rotary Inverted Pendulum Stabilization using Infinite-Horizon LQR

A MATLAB-based implementation of an **Infinite-Horizon Linear Quadratic Regulator (LQR)** for stabilizing a **Rotary Inverted Pendulum (Furuta Pendulum)**. The project models the nonlinear electromechanical system, derives its linearized state-space representation, designs an optimal state-feedback controller by solving the Algebraic Riccati Equation (ARE), and validates stabilization through simulation and 3D animation.

---

## Overview

The rotary inverted pendulum is a classical benchmark in control engineering due to its inherently unstable dynamics and nonlinear behavior. This project demonstrates the complete control design workflow:

- Mathematical modeling using Euler-Lagrange equations
- Linearization around the upright equilibrium
- State-space formulation
- Infinite-Horizon LQR controller design
- Closed-loop simulation
- 3D visualization and animation of pendulum stabilization

---

## Features

- Dynamic modeling of the rotary inverted pendulum
- Linearized state-space representation
- Controllability analysis
- Infinite-Horizon LQR optimal controller
- Closed-loop stability verification
- Time-response analysis
- 3D animation of pendulum motion
- MP4 video generation

---

## System Model

### State Vector

\[
x =
\begin{bmatrix}
\theta\\
\dot{\theta}\\
\phi\\
\dot{\phi}
\end{bmatrix}
\]

where

- **θ** – Rotary arm angle
- **φ** – Pendulum angle

---

### Linear State-Space Model

\[
\dot{x}=Ax+Bu
\]

where

- **A** – System matrix
- **B** – Input matrix
- **u** – Motor torque

The model is obtained by linearizing the nonlinear equations around the upright equilibrium position.

---

## Infinite-Horizon LQR Design

The optimal control law is

\[
u=-Kx
\]

where

\[
K=R^{-1}B^TP
\]

and **P** is obtained by solving the Continuous Algebraic Riccati Equation (CARE):

\[
A^TP+PA-PBR^{-1}B^TP+Q=0
\]

The controller minimizes

\[
J=\int_0^\infty (x^TQx+u^TRu)\,dt
\]

balancing state regulation and control effort.

---

## Workflow

```
Dynamic Modeling
        │
        ▼
Euler-Lagrange Equations
        │
        ▼
Linearization
        │
        ▼
State-Space Model
        │
        ▼
Controllability Check
        │
        ▼
Infinite-Horizon LQR
        │
        ▼
Closed-Loop Simulation
        │
        ▼
3D Animation
```

---

## Results

The designed LQR controller

- Stabilizes the inherently unstable upright equilibrium
- Moves all closed-loop poles to the left-half plane
- Minimizes state deviations and control effort
- Achieves smooth pendulum stabilization
- Produces a realistic 3D animation exported as MP4

---

## Project Structure

```
.
├── A2_Prob2_24EC10125_Controls.m
├── Pendulum_LQR.mp4
├── README.md
└── images
    ├── closed_loop_response.png
    ├── state_response.png
    └── animation_snapshot.png
```

---

## Requirements

- MATLAB R2022a or later
- Control System Toolbox
- MATLAB Graphics

---

## How to Run

1. Clone the repository

```bash
git clone https://github.com/your-username/Rotary-Inverted-Pendulum-LQR.git
```

2. Open MATLAB.

3. Run

```matlab
A2_Prob2_24EC10125_Controls
```

4. The script automatically

- Computes the optimal LQR gain
- Simulates the closed-loop dynamics
- Plots the arm and pendulum responses
- Generates a 3D animation
- Saves the animation as **Pendulum_LQR.mp4**

---

## Applications

- Optimal Control
- Modern Control Theory
- Robotics
- Autonomous Systems
- Mechatronics
- Control Systems Education

---

## Future Improvements

- PID controller implementation
- Pole placement control
- Kalman Filter / State Observer
- H∞ Robust Control
- Model Predictive Control (MPC)
- Swing-up control
- Hardware implementation on a real rotary inverted pendulum

---

## Author

**Sai Krushik Reddy Sammidi**

Department of Electronics and Electrical Communication Engineering  
Indian Institute of Technology Kharagpur

---
