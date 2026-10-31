# Descent 3.0 — 3-DoF Fuel-Optimal Rocket Landing Simulator (CVXPY & Three.js 3D)

A full **3-DoF (3-Dimensional Degrees of Freedom)** powered descent guidance simulator and optimization toolkit based on the **G-FOLD** algorithm developed by **Behçet Açıkmeşe, Scott R. Ploen, John M. Carson III, and Lars Blackmore** (*Journal of Guidance, Control, and Dynamics* 2007, *IEEE TCST* 2013).

---

## 🚀 What's New in Descent 3.0

1. **Full 3-DoF (3-Dimensional Degrees of Freedom):**
   - **3D Spatial Position:** $[x, y, z]^\top$ (Downrange $x$, Crossrange $y$, Altitude $z$).
   - **3D Velocity:** $[v_x, v_y, v_z]^\top$.
   - **3D Thrust Vector:** $[u_x, u_y, u_z]^\top$ constrained inside a 3D Second-Order Cone ($\sqrt{u_x^2 + u_y^2 + u_z^2} \le \Gamma$).
   - **3D Gimbal Pointing Cone:** $u_z \ge \Gamma \cos\theta_{\max}$.
   - **3D Glide-Slope Corridor:** $z \ge \sqrt{x^2 + y^2} \tan\gamma_{\text{gs}}$.
   - **3D Crosswinds:** Dual-axis wind modeling ($w_x, w_y$) and stochastic turbulence.

2. **Interactive 3D Three.js Visualizer:**
   - **Free Orbit Controls:** Click and drag with mouse to orbit around the landing site, right-click to pan, scroll wheel to zoom.
   - **Camera Presets:** Orbit 3D, Chase Cam (locks camera behind the falling booster), Pad Cam (stationed on the landing pad looking up), Top-Down ($X-Y$), and Side View ($X-Z$).
   - **3D Rocket Mesh:** Modeled fuselage, nosecone, 4 grid fins, 4 deployable landing legs, dynamic gimbaling nozzle, and dynamic exhaust flame with lighting.
   - **3D Glide-Slope Cone:** Visualized as an amber wireframe cone showing terrain safety limits.

3. **Advanced Mode: In-Browser Pyodide CVXPY Engine:**
   - Switchable engine selector in the header:
     - `Standard (50 Hz Native SOCP)`: Smooth, deterministic, sub-millisecond solving for real-time 60 FPS animation.
     - `Advanced (In-Browser Pyodide CVXPY)`: Executes the actual Python `cvxpy` library directly in WebAssembly inside your browser tab.
   - **Disclaimer:** Running direct CVXPY in WebAssembly takes ~100–350 ms per solve; visual stuttering or frame delay may occur depending on your CPU and hardware.

---

## 🧠 What is 3-DoF vs 6-DoF?

- **1-DoF:** Pure 1D vertical motion ($z, v_z$). No lateral divert.
- **2-DoF:** Planar 2D motion ($x, z, v_x, v_z$). Crossrange is ignored.
- **3-DoF:** Full 3D spatial translation of a point-mass:
  $$\mathbf{r}(t) = \begin{bmatrix} x(t) \\ y(t) \\ z(t) \end{bmatrix}, \quad \mathbf{v}(t) = \begin{bmatrix} v_x(t) \\ v_y(t) \\ v_z(t) \end{bmatrix}, \quad \mathbf{u}(t) = \begin{bmatrix} u_x(t) \\ u_y(t) \\ u_z(t) \end{bmatrix}$$
  The vehicle can divert arbitrarily across downrange $x$ and crossrange $y$ while managing vertical descent $z$.
- **6-DoF:** Adds the 3 rotational degrees of freedom (roll, pitch, yaw and angular rates $\boldsymbol{\omega}$ governed by moment of inertia tensor $\mathbf{I}$). In modern rocket architectures (SpaceX Falcon 9, NASA Mars Landers), **3-DoF guidance generates the fuel-optimal acceleration command $\mathbf{u}^*(t)$**, and an inner-loop attitude controller actuates the gimbal motors and RCS thrusters to point the vehicle.

---

## 📁 Included Files in Workspace

- [`index.html`](file:///c:/Users/USER/Downloads/Build%20TKS/index.html): Complete interactive web application with 3D Three.js canvas, 3-DoF controls, Pyodide CVXPY engine, and camera modes.
- [`gfold_cvxpy.py`](file:///c:/Users/USER/Downloads/Build%20TKS/gfold_cvxpy.py): Standalone Python script with 1D, 2D, and full 3-DoF G-FOLD CVXPY implementations.
- [`gfold_cvxpy_3d.ipynb`](file:///c:/Users/USER/Downloads/Build%20TKS/gfold_cvxpy_3d.ipynb): Standalone Jupyter Notebook with 3-DoF derivations, 3D plots, and paper validation.
- [`gfold_cvxpy_2d.ipynb`](file:///c:/Users/USER/Downloads/Build%20TKS/gfold_cvxpy_2d.ipynb): 2D reference notebook.
