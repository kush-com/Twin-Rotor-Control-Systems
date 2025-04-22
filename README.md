# Twin Rotor Hover and Pitch Control

This repository contains MATLAB scripts and files for the design, simulation, and analysis of a twin‑rotor UAV's **pitch** control loops, using both **PD** controllers and a **Fuzzy Inference System**. The controllers are tuned to meet user‑specified performance specifications, simulated via `ode45`, and verified with root‑locus and time‑domain analyses.

---

## 📂 File Structure

- **performance\_specs.m**\
  Defines global performance specifications:

  - `Ts_spec` (settling time, 2% band)
  - `OS_spec` (percent overshoot)

- **theta\_Param.m**\
  Computes PD gains (`Kp`, `Kd`) for the pitch plant \(G_{θ}(s)=l/(J\,s^2)\) based on performance specs.

- **theta\_ODE.m**\
  Defines the plant ODE for pitch:\
  \(\dot θ = θ̇, \; \ddot θ = (l/J)\,u_{in}\)\
  Used by `ode45` in the pitch execution script.

- **theta\_control\_simulation.m**\
  Simulates pitch control using PD following the same pattern as altitude.

- **theta\_pid\_design.m**\
  Uses `pidtune` to design a full **PID** controller for \(G_{θ}(s)\) to meet bandwidth derived from `Ts_spec`.\
  Plots step response (±OS bounds, Tr, Tp, Ts) and root locus (with damping contour).


- **fuzzy\_controller.fis** (or `.xml`)\
  Contains the Mamdani‐type fuzzy inference system (membership functions, rule base) used to predict thrust adjustments based on error and rate.

---

## 🚀 Getting Started

1. **Install Dependencies**

   - MATLAB R2025a (or later)
   - Control System Toolbox
   - Fuzzy Logic Toolbox

2. **Set Your Specs**

   - Open `performance_specs.m` and edit `Ts_spec, OS_spec, Tr_spec, Tp_spec` to your design targets.


   - Runs the PD‑driven ODE simulation and root‑locus plot for height.

4. **Pitch Control**

   ```matlab
   >> theta_control_simulation
   ```

5. **PID Design for Pitch**

   ```matlab
   >> theta_pid_design
   ```

   - Tunes PID via `pidtune`, plots step response and root locus with performance overlays.



7. **Fuzzy Controller GUI**

   ```matlab
   >> fuzzy('fuzzy_controller.fis')
   ```

---

## 📖 Theory & Validation

- **Second‑Order Linear Models**\
  \(h:\,1/(m\,s^2),\;θ:\,l/(J\,s^2)\)
- **PD/PID Tuning**
  - Analytical PD: match \(s^2 + 2ζω_n s + ω_n^2\)
  - PID via `pidtune`: bandwidth = \(4/T_s\)
- **ODE45 Simulation**
  - Iterative “3‑point” integration for state updates
- **Stability & Performance**
  - Root locus with constant‑ζ contour
  - Time‑domain metrics via `stepinfo`
- **Fuzzy Logic**
  - Mamdani inference to adapt thrust under nonlinearities

---

## 📄 License

This code is released under the MIT License. Feel free to use and modify for academic or research purposes.

---

*Prepared by Kushagra Katare, April 2025*

