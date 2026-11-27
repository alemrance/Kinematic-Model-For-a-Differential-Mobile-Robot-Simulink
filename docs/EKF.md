# Extended Kalman Filter (EKF) for a Differential Drive Robot

This document explains the EKF used in `mobile_robot_model.slx` from the ground up: the intuition, the equations, every matrix and its size, and a line-by-line walkthrough of the code.

---

## 1. The problem: no source can be fully trusted

The true pose of the robot is unknown. We have two imperfect sources of information:

| Source | Strength | Weakness |
|---|---|---|
| **Motion model** (encoders + gyro) | Smooth, fast (100 Hz) | Errors accumulate → drift |
| **Absolute position sensor** (UWB, RTK-GPS, lidar localization …) | Does not drift | Noisy, slow (1–10 Hz), does not measure heading |

The Kalman filter answers one question: **how do we combine two imperfect estimates in the best possible way?**
The key idea is to carry, next to every estimate, **how reliable it is** – expressed as a variance σ². Small variance = trustworthy.

## 2. One-dimensional intuition (the whole idea in two lines)

A robot moves along a line; we only care about x.

- Odometry says **x = 2.0 m** with σ = 0.2 m → variance **P = 0.04**
- The sensor says **z = 2.3 m** with σ = 0.1 m → variance **R = 0.01**

The sensor is more reliable, so the answer should lie closer to 2.3. The Kalman gain decides how much closer:

```
K     = P / (P + R)        = 0.04 / 0.05      = 0.8
x_new = x + K·(z − x)      = 2.0 + 0.8·0.3    = 2.24 m
P_new = (1 − K)·P          = 0.2·0.04         = 0.008   → σ ≈ 0.09 m
```

Two important observations:
1. The result leans towards the more reliable source.
2. **The new uncertainty (0.09 m) is smaller than either source alone.** Fusing is better than choosing.

The full EKF is exactly these two lines, generalised to **three states** (x, y, θ) and a **nonlinear** motion model.

---

## 3. Vectors and matrices

### State vector x̂ (3×1)
```
x̂ = [ x ; y ; θ ]
```
The hat (^) means "estimate".

### Covariance matrix P (3×3)
With several variables, uncertainty becomes a matrix:
```
      ┌ σx²   σxy   σxθ ┐
P  =  │ σxy   σy²   σyθ │
      └ σxθ   σyθ   σθ² ┘
```
- **Diagonal:** uncertainty of each variable on its own.
- **Off-diagonal:** how errors are **coupled**. σyθ ≠ 0 means "a heading error also shows up as a y error".
  These terms are what allow the EKF to correct θ although the sensor never measures it (Section 6).

### Input vector u (2×1)
```
u = [ v ; ω ]      v = r·(ωR + ωL)/2   (encoders)      ω = ω_gyro   (gyro)
```

### Measurement vector z (2×1)
```
z = [ x_measured ; y_measured ]
```

---

## 4. Prediction step (every Ts = 0.01 s)

### 4.1 Propagate the state
Discretised kinematics (forward Euler):
```
x ← x + v·cos(θ)·Ts
y ← y + v·sin(θ)·Ts
θ ← θ + ω·Ts
```
Written compactly as **x̂ ← f(x̂, u)**. This is identical to the dead-reckoning odometry.

### 4.2 Why linearisation is needed
Propagating the estimate is easy; propagating its **uncertainty** P is the hard part.
For a linear 1-D model x_new = a·x the variance becomes P_new = a²·P. Our model contains cos θ and sin θ, so it is **nonlinear**.
The EKF linearises the model around the current estimate – that is the "Extended" in Extended Kalman Filter. The multivariable derivative is called the **Jacobian**.

### 4.3 F – how state errors propagate (3×3)
F = ∂f/∂x̂ (each new state differentiated with respect to each old state):
```
          ∂/∂x   ∂/∂y   ∂/∂θ
x_new  [  1      0      −v·sin(θ)·Ts ]
y_new  [  0      1       v·cos(θ)·Ts ]
θ_new  [  0      0       1           ]
```
The top-right terms say: **a heading error leaks into x and y as the robot drives.** This is exactly the oscillating x/y error observed in the odometry experiments.

### 4.4 G – how input errors propagate (3×2)
G = ∂f/∂u:
```
          ∂/∂v          ∂/∂ω
x_new  [  cos(θ)·Ts     0  ]
y_new  [  sin(θ)·Ts     0  ]
θ_new  [  0             Ts ]
```
An error in v moves the robot along its heading; an error in ω only changes the heading.

### 4.5 Q – uncertainty added at every step (3×3)
Input noise is described by a 2×2 matrix and mapped into state space through G:
```
M = diag(σv², σω²)              (2×2)
Q = G · M · Gᵀ                  (3×2)(2×2)(2×3) = 3×3
```

### 4.6 Prediction equations
```
x̂ ← f(x̂, u)
P ← F·P·Fᵀ + Q
```
- **F·P·Fᵀ** – carry the old uncertainty along with the motion (e.g. heading uncertainty turns into position uncertainty).
- **+ Q** – add the new uncertainty of this step.

**P grows at every prediction step.**

---

## 5. Correction step (only when a measurement arrives, every Ts_pos)

### 5.1 Measurement model H (2×3)
"If the state were x̂, what would the sensor read?"
```
H = [ 1 0 0 ]        H·x̂ = [ x ; y ]
    [ 0 1 0 ]
```
Already linear, so no Jacobian is needed here.

### 5.2 Measurement noise R (2×2)
```
R = σpos² · I₂
```

### 5.3 Correction equations – side by side with the 1-D case

| Step | Matrix form | Size | 1-D equivalent |
|---|---|---|---|
| Innovation | y = z − H·x̂ | 2×1 | z − x |
| Innovation covariance | S = H·P·Hᵀ + R | 2×2 | P + R |
| Kalman gain | K = P·Hᵀ·S⁻¹ | 3×2 | P / (P + R) |
| State update | x̂ ← x̂ + K·y | 3×1 | x + K·(z − x) |
| Covariance update | P ← (I − K·H)·P | 3×3 | (1 − K)·P |

Same formulas – division becomes a matrix inverse, products become matrix products.

---

## 6. How θ is corrected without being measured

K is 3×2; its rows tell how much x, y and θ are corrected. The **third row** comes from

```
K = P·Hᵀ·S⁻¹
```

P·Hᵀ selects the first two columns of P, whose third row contains **σxθ and σyθ** – the coupling between heading and position. While the robot drives, F creates this coupling. When a measurement then reports an x/y deviation, the third row of K concludes *"part of this deviation must come from a heading error"* and corrects θ as well.

> When the robot stands still (v = 0) no coupling is created and θ cannot be corrected – which makes sense: a heading error of a stationary robot does not show up in its position.

---

## 7. Code walkthrough

```matlab
function [pose, Pdiag] = ekf(wL, wR, w_gyro, z, t, r, Ts, Ts_pos, sig_pos, x0)
```
- **Inputs (ports):** `wL`, `wR` (noisy encoders), `w_gyro`, `z` (2×1 position measurement), `t` (time from a Digital Clock)
- **Parameters** (Scope = *Parameter*, read from `init_robot.m`): `r`, `Ts`, `Ts_pos`, `sig_pos`, `x0`
- **Outputs:** `pose` (3×1 estimate), `Pdiag` (3×1 variances of x, y, θ)
- **Block settings:** *Update method = Discrete*, *Sample time = Ts* (otherwise the block may run several times per step and corrupt its persistent memory)

```matlab
sig_v = 0.02;   % m/s
sig_w = 0.1;    % rad/s
```
The elements of M – the filter's **tuning knobs**:
- `sig_v`: encoder noise is 0.5 rad/s per wheel → noise on v ≈ 0.045·0.5/√2 ≈ 0.016 m/s → 0.02 with margin.
- `sig_w`: gyro noise is only 0.02 rad/s, but the filter does not know about the gyro **bias**. Inflating σω to 0.1 makes the filter trust the gyro less and lean more on the position sensor, which absorbs the unmodeled bias (see Section 9).

```matlab
persistent xh P
if isempty(xh)
    xh = x0;
    P  = diag([1e-4 1e-4 1e-4]);
end
```
- `persistent` variables keep their values between time steps – the filter's memory.
- Initial P: σ = 1 cm and 0.01 rad, i.e. "the start pose is well known".

```matlab
v  = r*(wR + wL)/2;
w  = w_gyro;
th = xh(3);
xh = xh + [v*cos(th); v*sin(th); w]*Ts;
```
State prediction x̂ ← f(x̂, u). Note that `th` holds the heading **before** the update; F and G must be evaluated at that point.

```matlab
F = [1 0 -v*sin(th)*Ts;  0 1 v*cos(th)*Ts;  0 0 1];
G = [cos(th)*Ts 0;  sin(th)*Ts 0;  0 Ts];
Q = G*diag([sig_v^2 sig_w^2])*G';
P = F*P*F' + Q;
```
Covariance prediction (Sections 4.3–4.6).

```matlab
if abs(t/Ts_pos - round(t/Ts_pos)) < 1e-6
```
"Is this a measurement instant?" – true when t is an integer multiple of Ts_pos. A tolerance is used instead of `== 0` because values like 0.01 are not exactly representable in floating point.

```matlab
    H = [1 0 0; 0 1 0];
    R = sig_pos^2*eye(2);
    y = z - H*xh;
    S = H*P*H' + R;
    K = P*H'/S;
    xh = xh + K*y;
    P  = (eye(3) - K*H)*P;
end
```
Correction step (Section 5.3). `P*H'/S` computes P·Hᵀ·S⁻¹ in a numerically safer way than `inv(S)`.

```matlab
pose  = xh;
Pdiag = diag(P);
```
`sqrt(Pdiag)` is the filter's own estimate of its 1σ error. `plot_results` draws ±3σ around the EKF error: for a consistent filter the error stays inside these bounds.

---

## 8. Summary of all quantities

| Symbol | Size | Meaning | Source |
|---|---|---|---|
| x̂ | 3×1 | Estimate [x; y; θ] | Filter memory |
| P | 3×3 | Estimate covariance | Filter memory, initial P₀ |
| u | 2×1 | [v; ω] | Encoders + gyro |
| F | 3×3 | State error propagation | ∂f/∂x̂ |
| G | 3×2 | Input error propagation | ∂f/∂u |
| M | 2×2 | Input noise diag(σv², σω²) | **Tuning:** `sig_v`, `sig_w` |
| Q | 3×3 | Process noise per step | G·M·Gᵀ |
| z | 2×1 | Measured [x; y] | Position sensor |
| H | 2×3 | State → measurement | Sensor measures x, y |
| R | 2×2 | Measurement noise | **Tuning:** `sig_pos` |
| y | 2×1 | Innovation | z − H·x̂ |
| S | 2×2 | Innovation covariance | H·P·Hᵀ + R |
| K | 3×2 | Kalman gain | P·Hᵀ·S⁻¹ |

## 9. Tuning

| Change | Filter's interpretation | Effect |
|---|---|---|
| `sig_v`, `sig_w` ↑ (Q ↑) | "I trust odometry less" | Follows measurements more closely; estimate becomes jumpier |
| `sig_v`, `sig_w` ↓ (Q ↓) | "My odometry is excellent" | Smooth estimate, but slow to correct drift (over-confident) |
| `sig_pos` ↑ (R ↑) | "The sensor is poor" | Smaller corrections |
| `sig_pos` ↓ (R ↓) | "The sensor is perfect" | Estimate snaps to each measurement |

Measured effect of `sig_w` (gyro bias −0.005 rad/s, `demo` profile, RMS over t > 5 s):

| `sig_w` [rad/s] | RMS position | RMS heading | Mean heading error |
|---|---|---|---|
| 0.03 | 0.119 m | 0.116 rad | 0.110 rad |
| **0.10** | **0.060 m** | **0.052 rad** | **0.045 rad** |
| 0.20 | 0.063 m | 0.045 rad | 0.023 rad |
| 0.50 | 0.084 m | 0.057 rad | 0.007 rad |

Too small → the filter is over-confident in the biased gyro. Too large → it follows the noisy position measurements too closely. The proper fix for the bias is to estimate it as a 4th state.

## 10. Lessons learned (pitfalls)

- **Timing matters more than the equations.** The `t` input must come from a Digital Clock with sample time `Ts`. With a 1 s clock, `t` was held constant for a whole second, the "measurement instant" condition stayed true for 100 consecutive steps, and the filter kept pulling the estimate back to a stale measurement. Result: the estimate trailed the robot by ~0.25 rad.
- **XY plots hide timing errors.** An estimate that lies on the correct path but at the wrong time looks fine in an XY plot; the error-vs-time plot reveals it. Always check both.
- **Use a fixed-step solver.** With a variable-step solver the simulation time may never land exactly on the measurement instants.
- **Rate Transition blocks** (default settings) add one slow-rate sample of delay between a slow sensor and a fast filter – avoid them in this path or disable *Ensure deterministic data transfer*.
