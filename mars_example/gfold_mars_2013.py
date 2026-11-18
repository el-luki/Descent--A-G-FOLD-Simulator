"""
Reproduces the Mars numerical example (Table I) of

  B. Acikmese, J. M. Carson III, L. Blackmore, "Lossless Convexification of Nonconvex Control
  Bound and Pointing Constraints of the Soft Landing Optimal Control Problem",
  IEEE Trans. Control Systems Technology 21(6), 2013, Section IV.

Paper's example (all values from Section IV):
  m0 = 2000 kg, fuel available m_f = 300 kg, Tmax = 24000 N, throttle 20 %..80 %,
  alpha = 5e-4 s/m, r0 = (2400, 450, -330) m, v0 = (-10, -40, 10) m/s, target q = 0,
  g = (-3.71, 0, 0) m/s^2, omega = (2.53e-5, 0, 6.62e-5) rad/s, x axis = up (altitude).
  Pointing cone n = e1 (local vertical), theta = unconstrained / 90 deg / 45 deg.
Paper's Table I:  unconstrained 200.1 kg, 44.63 s | 90 deg 201.8 kg, 46.96 s | 45 deg 222.3 kg, 57.29 s

Method (as in the paper and its ref. [1], Acikmese & Ploen 2007):
  change of variables sigma = Gamma/m, u = Tc/m, z = ln m makes the dynamics linear;
  lossless convexification: ||u|| <= sigma, n^T u >= cos(theta) sigma;
  thrust bounds rho1 e^-z <= sigma <= rho2 e^-z via the second-order-cone approximation (36)
  around z0(t) = ln(m0 - alpha rho2 t); flight time tf searched outside the convex problem.
The paper does not list N, the discretization, a glide-slope angle or a speed limit for this
example; those choices are ours and are printed with the results.

Runs in normal Python (pip install cvxpy numpy scipy) or in the browser via Pyodide.
"""
import json
import numpy as np
import cvxpy as cp
from scipy.linalg import expm

# ---- example parameters (Section IV) ----
M0, MF = 2000.0, 300.0
TMAX = 24000.0
RHO1, RHO2 = 0.2 * TMAX, 0.8 * TMAX
ALPHA = 5e-4
R0 = np.array([2400.0, 450.0, -330.0])
V0 = np.array([-10.0, -40.0, 10.0])
G = np.array([-3.71, 0.0, 0.0])
OMEGA = np.array([2.53e-5, 0.0, 6.62e-5])
NHAT = np.array([1.0, 0.0, 0.0])


def skew(w):
    return np.array([[0.0, -w[2], w[1]], [w[2], 0.0, -w[0]], [-w[1], w[0], 0.0]])


def discretize(dt, method="zoh"):
    """Discretize xdot = A x + B (g + u), x = (r, v): exact zero-order hold, or forward Euler."""
    S = skew(OMEGA)
    A = np.block([[np.zeros((3, 3)), np.eye(3)], [-S @ S, -2.0 * S]])
    B = np.vstack([np.zeros((3, 3)), np.eye(3)])
    if method == "euler":
        return np.eye(6) + A * dt, B * dt
    M = np.zeros((9, 9))
    M[:6, :6], M[:6, 6:] = A * dt, B * dt
    E = expm(M)
    return E[:6, :6], E[:6, 6:]          # Phi, Gamma_d


def solve(tf, N=120, theta_deg=None, glide_deg=None, vmax=None, method="zoh"):
    """Minimum-fuel landing for a fixed flight time tf. Returns dict or None if infeasible."""
    dt = tf / N
    Phi, Gd = discretize(dt, method)
    x = cp.Variable((6, N + 1))
    z = cp.Variable(N + 1)               # ln(mass)
    u = cp.Variable((3, N))              # thrust / mass
    s = cp.Variable(N)                   # sigma = Gamma / mass (the slack)
    t = dt * np.arange(N)
    z0 = np.log(M0 - ALPHA * RHO2 * t)   # reference ln(mass) at full thrust
    mu1, mu2 = RHO1 * np.exp(-z0), RHO2 * np.exp(-z0)
    cons = [x[:, 0] == np.hstack([R0, V0]), z[0] == np.log(M0),
            x[:, N] == 0,                                       # land on the target at rest
            z[N] >= np.log(M0 - MF)]                            # no more fuel than available
    dz = z[:-1] - z0
    cons += [x[:, 1:] == Phi @ x[:, :-1] + Gd @ (G[:, None] + u),
             z[1:] == z[:-1] - ALPHA * dt * s,
             cp.norm(u, 2, axis=0) <= s,                                      # ||u|| <= sigma
             cp.multiply(mu1, 1 - dz + cp.square(dz) / 2) <= s,               # eq. (36), lower bound
             s <= cp.multiply(mu2, 1 - dz)]                                   # eq. (36), upper bound
    # mass stays between "full thrust all the way" and "minimum thrust all the way"
    cons += [z[1:] >= np.log(M0 - ALPHA * RHO2 * dt * np.arange(1, N + 1)),
             z[1:] <= np.log(M0 - ALPHA * RHO1 * dt * np.arange(1, N + 1))]
    if theta_deg is not None:
        cons += [NHAT @ u >= np.cos(np.radians(theta_deg)) * s]
    if glide_deg is not None:
        cons += [np.tan(np.radians(glide_deg)) * cp.norm(x[1:3, :], 2, axis=0) <= x[0, :]]
    if vmax is not None:
        cons += [cp.norm(x[3:6, :], 2, axis=0) <= vmax]
    prob = cp.Problem(cp.Maximize(z[N]), cons)
    try:
        prob.solve(solver=cp.CLARABEL)
    except Exception:
        return None
    if prob.status not in (cp.OPTIMAL, cp.OPTIMAL_INACCURATE):
        return None
    m = np.exp(z.value)
    thrust = s.value * m[:-1]                    # Gamma = sigma * m
    gap = float(np.max(np.abs(s.value - np.linalg.norm(u.value, axis=0)) * m[:-1]))
    tilt = np.degrees(np.arccos(np.clip(u.value[0] / np.maximum(np.linalg.norm(u.value, axis=0), 1e-9), -1, 1)))
    return dict(tf=tf, fuel=float(M0 - m[-1]), thr_min=float(thrust.min() / TMAX),
                thr_max=float(thrust.max() / TMAX), gap_N=gap, max_tilt=float(tilt.max()),
                min_alt=float(x.value[0].min()))


def best_tf(lo=30.0, hi=75.0, step=2.5, **kw):
    """Coarse scan, then golden-section refinement of fuel over flight time."""
    grid = [(tf, solve(tf, **kw)) for tf in np.arange(lo, hi + 1e-9, step)]
    ok = [(tf, r) for tf, r in grid if r]
    if not ok:
        return None
    tf0 = min(ok, key=lambda p: p[1]["fuel"])[0]
    a, b = max(lo, tf0 - step), min(hi, tf0 + step)
    gr = (5 ** 0.5 - 1) / 2
    f = lambda tf: (solve(tf, **kw) or {"fuel": 1e9})["fuel"]
    c, d = b - gr * (b - a), a + gr * (b - a)
    fc, fd = f(c), f(d)
    for _ in range(14):
        if fc < fd:
            b, d, fd = d, c, fc
            c = b - gr * (b - a)
            fc = f(c)
        else:
            a, c, fc = c, d, fd
            d = a + gr * (b - a)
            fd = f(d)
    return solve((a + b) / 2, **kw)


def run(N=120, glide_deg=None, vmax=None):
    paper = {"Unconstrained": (200.1, 44.63), "90 deg": (201.8, 46.96), "45 deg": (222.3, 57.29)}
    out = []
    for name, theta in [("Unconstrained", None), ("90 deg", 90.0), ("45 deg", 45.0)]:
        r = best_tf(N=N, theta_deg=theta, glide_deg=glide_deg, vmax=vmax)
        pf, pt = paper[name]
        out.append(dict(case=name, paper_fuel=pf, paper_tf=pt, **(r or {})))
    return out


if __name__ == "__main__":
    for row in run():
        print(json.dumps(row))
