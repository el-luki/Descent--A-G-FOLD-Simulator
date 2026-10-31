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

3. **Two guidance engines (selector in the header):**
   - `Fast solver`: a JavaScript heuristic (Hermite-curve guess, clipped to the thrust and tilt limits). Instant, but **not an optimizer**.
   - `CVXPY (Wasm)`: the real convex program, solved **in your browser**. [Pyodide 0.27.7](https://pyodide.org) loads Python 3.12, NumPy, SciPy, CVXPY 1.5.1 and the Clarabel interior-point solver inside a background Web Worker, so the 3D scene keeps animating while it solves.
     - **First plan:** flight time is not convex, so it is searched outside the SOCP: a 1 s scan from 3 to 40 s, then golden-section refinement (about 50 solves, under 1 s in total).
     - **In flight:** receding-horizon re-planning about 5× per second from the noisy measured state (N = 30, first feasible flight time ≥ the time left). Below 15 m a soft-touchdown controller takes over.
     - The problem is built with CVXPY `Parameter`s (DPP), so it compiles once and each later solve only swaps numbers in. Measured in Chrome: **≈ 7–12 ms per solve**, 12–14 interior-point iterations, relaxation gap max |Γ − ‖u‖| ≈ 1e-7.
     - The Python source is a `<script type="text/x-python" id="gfold-py">` block in `index.html`. The same text is sent to Pyodide and shown in the "Python code" window, where "Execute" prints Clarabel's real verbose log.
     - First load downloads the Python runtime and packages from the jsDelivr CDN (≈ 5–6 s on a fast connection, cached afterwards). If no feasible landing exists for the chosen sliders, the page says so and the fast solver flies instead.
     - **Terminal gate:** the plan ends 15 m straight above the pad, with no sideways speed, descending no faster than the touchdown controller's entry speed (≈ 14 m/s at default thrust), not at the ground. A pure minimum-fuel plan to the ground is a last-instant full-thrust "suicide burn" that an engine with spool-up and turn-rate limits can't follow. The speed is an upper bound rather than an exact value; an exact value can be unreachable because of the throttle floor, which made re-planning loop and the rocket hover.
     - **Tightness check:** lossless convexification only guarantees Γ = ‖u‖ when the original problem is feasible. For a flight time that's impossible for the real engine, the relaxed problem can still be solved by quietly using thrust below the minimum (Γ > ‖u‖). Any solution with max |Γ − ‖u‖| > 1e-4·a_max is therefore rejected as infeasible. Without this, the 1D check found a flight time 0.23 s shorter than physically possible.
     - **Re-plans** search arrival times from the current one to 6 s later and keep the cheapest. If the current arrival became impossible, they add 0.75 s of margin, because the cheapest time sits exactly on the edge of feasibility.
     - **Thrust reserve:** CVXPY plans with 85 % of maximum thrust, so the vehicle can catch up when it falls behind the plan and re-plans stay feasible.
   - **Where it runs:** any normal page served over `http(s)` (GitHub Pages, `localhost`). Sandboxed preview frames that block WebAssembly or workers will not run it.

4. **Realistic vehicle response:** guidance only sets *commands*. The body turns toward the commanded thrust direction at ≤ 30°/s with ≤ 60°/s² angular acceleration, the engine throttles with a 0.25 s spool lag, and the bell visibly gimbals ahead of the body. Throttle is computed from the attitude the rocket actually has, so it keeps the vertical thrust guidance asked for while still turning. Both solvers aim for the same 15 m gate above the pad; below it a touchdown controller follows a descent profile that slows steadily to ≈ 0.6 m/s at touchdown (≈ 3.5 s for the last 15 m) and brings the rocket upright. If it is clearly off the pad it slows to 1.5 m/s for at most 3 s to steer back. It never stops to hover.

5. **Random weather (Conditions tab):** every 1.8–4 s above 40 m, one of: a wind shift, a sudden 12–22 m/s gust, an air-pressure drop or rise (density 40–180 %), a 5–10 m/s downdraft/updraft, or a 2.5–4.5 s engine thrust dip to 82–92 %. Each event is announced on the 3D view and triggers an immediate re-plan; CVXPY is told about the reduced thrust.

6. **Inside the optimizer (bottom of the page):** the convex problem written out with live numbers, the non-convex throttle floor highlighted, and two checks that run CVXPY from the current sliders:
   - *Why the convexification matters:* the same landing solved with Γ (tight, flyable) and with the floor simply dropped (cheaper on paper, but asks for less than the engine minimum; forcing those steps up afterwards misses the gate by hundreds of metres at default settings).
   - *Check against the exact 1D answer:* coast at minimum throttle, then full throttle at the last moment (closed-form switch time), vs CVXPY. At default settings: same 7.91 s coast, Δv within 0.01 %.
7. **Landing history (under the 3D view):** one row per touchdown with solver, weather, distance, touchdown speed, fuel, flight time and, for CVXPY flights, the number of convex solves, the worst |Γ − ‖u‖| and how often the throttle floor, tilt limit and full thrust were active. Kept in this browser (localStorage).

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
