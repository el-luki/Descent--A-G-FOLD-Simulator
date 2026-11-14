# Reproducing the Mars example of Açıkmeşe, Carson & Blackmore (2013)

Paper: "Lossless Convexification of Nonconvex Control Bound and Pointing Constraints of the Soft Landing
Optimal Control Problem", IEEE Trans. Control Systems Technology 21(6), 2013, Section IV, Table I.
Script: [gfold_mars_2013.py](gfold_mars_2013.py) (CVXPY 1.5.1 + Clarabel, run in the browser via Pyodide).

Example (from the paper): m0 = 2000 kg, 300 kg fuel available, Tmax = 24 000 N, throttle 20–80 %,
α = 5×10⁻⁴ s/m, r0 = (2400, 450, −330) m, v0 = (−10, −40, 10) m/s, target at the origin,
g = (−3.71, 0, 0) m/s², Mars rotation ω = (2.53×10⁻⁵, 0, 6.62×10⁻⁵) rad/s, x axis = up.

Our choices (not given in the paper for this example): N = 100 steps, exact zero-order-hold
discretization, no glide-slope or speed limit (adding a 30° glide slope and a 90 m/s limit changes
nothing above 0.1 kg; N = 200 changes nothing above 0.1 kg).

## Same flight time as the paper

| Pointing limit | Flight time | Paper fuel | Our fuel | Difference |
|---|---|---|---|---|
| None | 44.63 s | 200.1 kg | 199.0 kg | −0.5 % |
| 90° | 46.96 s | 201.8 kg | 201.0 kg | −0.4 % |
| 45° | 57.29 s | 222.3 kg | 221.0 kg | −0.6 % |

All solutions: throttle within 20–80 %, thrust within the pointing limit, largest |Γ − ‖T‖| = 0.012 N
(the minimum thrust is 4 800 N), so the relaxation is lossless here too.

## Our own flight-time search (golden section)

| Pointing limit | Paper | Ours |
|---|---|---|
| None | 200.1 kg, 44.63 s | 198.9 kg, 43.83 s |
| 90° | 201.8 kg, 46.96 s | 200.9 kg, 46.18 s |
| 45° | 222.3 kg, 57.29 s | 209.6 kg, 52.64 s |

For 45° the fuel keeps rising with flight time (52.64 s → 209.6 kg, 55 s → 214.5 kg, 57.29 s → 221.0 kg),
so the cheapest landing is the earliest feasible one. Our discretization finds 52.6 s still feasible;
the paper's search stopped at 57.29 s. At equal flight times the two agree within 0.6 %.
