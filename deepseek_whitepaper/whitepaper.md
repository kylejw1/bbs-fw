# A Virtual Pedal Torque Sensor for Cadence-Limited Pedelecs

### Estimating rider load from cadence and motor current alone

**Design report / whitepaper — v1.0**
Method name: **VLSE** (Virtual Load Sensing Estimator)

---

## Abstract

Most low-cost e-bikes have no crank torque sensor. Assist is therefore switched on
cadence alone, which means a rider who is merely being *carried* around by the
motor ("ghost pedalling") receives the same assist as a rider grinding up a hill
at the same cadence. This report specifies, analyses and simulates an algorithm
that recovers an estimate of **rider crank torque** — a virtual torque sensor —
from five signals that a sensorless pedelec already has: rider pedal cadence,
motor cadence, target maximum motor cadence, motor current, and target motor
current.

The central difficulty is stated as an observability result. Because the drivetrain
contains a one-way clutch, the rider can never be faster than the motor, and when
the two are locked they share a single rotational degree of freedom. In that
locked regime the driveline torque balance contains two unknowns — rider torque and
the grade/rolling load — and constant-speed data gives one equation. **Rider torque
is therefore not observable from cadence and current alone at steady state.** Any
working design must break that degeneracy by other means.

The VLSE breaks it with four independent physical channels:

1. a **freewheel anchor** — whenever the clutch opens, rider torque is exactly
   zero, which yields a ground-truth measurement of the load;
2. a **stroke-ripple anchor** — the rider's torque is strongly periodic at the
   pedal frequency, and angle-synchronous demodulation of the driveline torque
   balance recovers its amplitude without needing the grade at all;
3. a **motor-effort bound** — a saturated controller whose cadence is still
   sagging provably implies a minimum rider contribution;
4. an **inertial channel** — the motor cadence is a high-resolution virtual
   accelerometer, and during transients $J\,\dot\omega$ is large and informative.

These are fused in a three-state extended Kalman filter. A scale factor (the
rider's ripple-to-mean torque ratio) is self-calibrated during a short
commissioning ride (or from the first coast).

In a nine-scenario simulation of a 95 kg rider+bike with a 500 W-class mid-drive,
the estimator tracks a 0.25 s-averaged rider torque with **MAE of 1.0 Nm in
crowded low-speed riding, 1.6 Nm at a steady 20 km/h cruise, 1.8 Nm across a
3.5 s pedal pause and resumption, 3.9 Nm on a 7 % climb, and 2.7 Nm during a hard
12→25 km/h acceleration**. Steady-state window RMSE is 1.0–3.4 Nm; across all
44 385 engaged samples the correlation between estimate and truth is $r = 0.809$
with a mean error of −1.94 Nm and a 90th-percentile absolute error of 6.8 Nm.
Critically, in the ghost-pedalling scenario — which is a **matched-cadence
control**: identical speed, gear, cadence target, grade and lock state, differing
only in rider torque (1.5 Nm versus 13.0 Nm) — the estimator reports **3.0 Nm
instead of 12.0 Nm**. The residual is not a modelling failure but the channel's
**demodulation noise floor** (≈1.2 Nm of AC torque, ⇒ ≈2.6 Nm on the mean), which
is analytically predictable and reducible by averaging.

A sensitivity study yields a clean and slightly counter-intuitive result: because
the online bias estimator acts as a free parameter that absorbs any *constant*
load-model mismatch, errors in the torque constant, viscous coefficient and
aerodynamic coefficient are almost entirely self-correcting (MAE moves by less
than 0.1 Nm for ±10–30 % errors in each). The dominant sensitivities are instead the
ripple-ratio calibration, which acts as a multiplicative gain (−20 % calibration
error costs ~40 % in MAE), and crank-sensor timing quality, which beats
everything else (5× pulse jitter roughly triples the MAE). A fourth limit is
structural: the stroke-ripple channel has a noise floor of ≈2.6 Nm on the mean
rider torque, so the estimator resolves whether the rider is working but not
whether they are working *slightly*.

---

## 1. Introduction

### 1.1 The problem

A torque-sensing pedelec measures crank torque directly and scales assist to it.
That gives proportional, intuitive assist, correct behaviour when the rider is
merely turning the pedals, and a natural feel on climbs. The sensor is also the
single most expensive and mechanically intrusive part of the system.

Cheaper bikes use a **cadence (PAS) sensor only**. The controller then has no idea
how hard the rider is pushing. The classic consequences:

* **Ghost pedalling.** The rider rests their feet on the pedals while the motor
  carries the bike; the cadence sensor reads normal cadence and the controller
  applies full assist. Range is wasted and the bike feels like it is running away.
* **No load proportional assist.** A rider sprinting up a 8 % ramp and a rider
  dawdling on the flat in a crowded street receive the same assist level.
* **Poor low-speed manners.** Cadence-only controllers cannot tell "riding slowly
  because I am in traffic" from "riding slowly because I am struggling".

The question this report answers: **can rider load be inferred from the signals a
cadence-only bike already has, well enough to be useful?**

### 1.2 Available signals

| Signal | Symbol | Quality | Rate |
|---|---|---|---|
| Rider pedal cadence | $c_r$ | coarse, one-pulse-latent, dropouts near zero | 1 pulse per $\frac{1}{12}$ rev |
| Motor cadence | $c_m$ | high resolution, low noise | ≈100 Hz |
| Target max motor cadence | $c_m^\star$ | controller setpoint | control rate |
| Motor current | $i_m$ | measured, ≈1–5 % noise | ≈100 Hz |
| Target motor current | $i^\star_m$ | controller command | control rate |

All are already present on a cadence-only pedelec. No new hardware is required —
this is purely an estimation problem.

### 1.3 The coupling constraint

The stated system behaviour is:

> *Rider cadence cannot be faster than motor cadence, and when the two meet they
> lock together. Rider cadence can only be ≤ motor cadence.*

This is exactly the complementarity condition of a **one-way clutch (freewheel)**
placed between the crank and the motor/driveline, with the motor cadence expressed
in crank-equivalent units. Two consequences dominate the entire design:

* When $c_r < c_m$ the clutch is **open**. The rider's pedals are not connected to
  the driveline: **transmitted rider torque is identically zero.**
* When $c_r = c_m$ the clutch is **locked**. Rider and motor now share one degree
  of freedom. The rider can push harder only by accelerating *both* of them.

There is no third case. In particular, there is no regime in which the rider
contributes torque while freewheeling: a rider who pushes while below the ceiling
simply locks up and drives the driveline.

### 1.4 Contributions

1. A formal **observability analysis** (§3) showing precisely what is and is not
   knowable, and why cadence alone can never separate "struggling" from "ghost
   pedalling".
2. A complete estimator, the **VLSE** (§4), built from four physical channels and
   a self-calibrating scale factor.
3. A **stroke-ripple demodulation** technique that makes the grade bias observable
   even during a climb with no freewheel events — the channel that directly
   addresses ghost pedalling.
4. A **nine-scenario simulation study** (§5) with quantitative results, including
   the failure cases, plus a robustness sweep (§6).
5. A discussion of how to convert the estimate into assist (§7) and an honest
   account of the remaining failure modes (§8).

---

## 2. System model

### 2.1 Coordinates and notation

Everything is expressed in **crank-referenced SI units**, since that is the only
frame in which "rider cadence ≤ motor cadence" is dimensionally meaningful.

| Symbol | Meaning | Unit |
|---|---|---|
| $\omega_r,\ \omega_m$ | rider crank speed, motor cadence referred to the crank | rad/s |
| $\tau_r$ | rider torque **transmitted through the clutch** | Nm |
| $\tau_m = K_t i_m$ | motor torque at the crank | Nm |
| $\tau_L$ | total load torque at the crank | Nm |
| $J_c$ | crank + rider legs inertia | kg m² |
| $J_d$ | reflected vehicle inertia $= m\rho^2 + J_{rot}$ | kg m² |
| $\rho$ | metres travelled per crank radian | m/rad |
| $v = \rho\,\omega_m$ | road speed | m/s |
| $b$ | lumped driveline viscous coefficient | Nm/(rad/s) |

The gear ratio $\rho$ absorbs the crank:wheel ratio; $J_d$ is the entire bike and
rider reflected to the crank and is therefore two to three orders of magnitude
larger than $J_c$. For a 95 kg system in a mid gear, $J_d \approx 55$ kg m² versus
$J_c \approx 0.15$ kg m². This asymmetry is important: the crank alone spins up in
tens of milliseconds, the vehicle in seconds.

### 2.2 Driveline dynamics

**Clutch open** ($\omega_r < \omega_m$):

$$J_c \dot\omega_r = \tau_r - b_c\omega_r, \qquad
  J_d \dot\omega_m = \tau_m - \tau_L - b\,\omega_m,
  \qquad \tau_r^{\text{trans}} \equiv 0$$

**Clutch locked** ($\omega_r = \omega_m = \omega$):

$$(J_c + J_d)\,\dot\omega = \tau_m + \tau_r - \tau_L - (b_c + b)\,\omega$$

with the clutch torque required to maintain the constraint,

$$\tau_{cl} = \tau_r - b_c\omega - J_c\dot\omega \ \ge 0 .$$

If $\tau_{cl}$ would go negative — the rider backs off, or the driveline
accelerates away downhill — the clutch releases. This is the whole model.

### 2.3 Load model

Expressing the road load at the crank gives

$$\tau_L = \underbrace{\rho\, m g (C_{rr}\cos\theta + \sin\theta)}_{b_0}
        + \underbrace{b}_{b_1}\,\omega
        + \underbrace{\tfrac12 \rho_{air} C_d A\,\rho^{3}}_{b_2}\,\omega^{2}.$$

* $b_1$ and $b_2$ are stable machine constants, independent of route. In this
  study they are treated as **known** (as they would be after a commissioning
  ride); §6 shows the estimator is almost insensitive to them, so identifying
  them on line would add little.
* $b_0$ is the problem child. It contains rolling resistance (nearly constant) and
  **grade**, which changes as the road changes and is not measured. It is the only
  load parameter identified at run time.

For the simulated vehicle $b_1 = 0.30$ Nm/(rad/s) and $b_2 = 0.13$ Nm/(rad/s)²
(the latter scales as $\rho^3$ and is therefore gear-dependent).

### 2.4 Motor and controller

A conventional cascaded controller: an outer cadence loop produces a current
command, which the inner current loop tracks with a small lag,

$$i^\star_m = \mathrm{clamp}\!\big(K_p(\omega_m^\star - \omega_m) + K_i\textstyle\int(\omega_m^\star-\omega_m),\ I_{min}, I_{max}\big),
\qquad \tau_e \dot i_m = i^\star_m - i_m .$$

$I_{max}$ is a hard torque ceiling and is the physical origin of the motor-effort
channel in §4.5. In this study the controller is **assist-only** ($I_{min}=0$), so
it cannot brake the rider; that is the normal behaviour of a cadence-limited
pedelec and it makes the rider's own torque responsible for any overspeed.

### 2.5 Rider model (plant only)

Riders do not regulate speed so much as **power**, so the simulated rider is a
power feed-forward with a modest speed-error correction:

$$\tau_r = \mathrm{clamp}\!\Big(\frac{P^\star}{\omega_r} + K_{p,r}(v^\star - v),\ 0,\ \tau_{max}(\omega_r)\Big)\;
\underbrace{\big(1 + a_1\sin(\varphi+\psi_1) + a_2\sin(2\varphi+\psi_2)\big)}_{s(\varphi)}$$

with a force–velocity ceiling $\tau_{max}(\omega)=\tau_{peak}(1-\omega/\omega_{max})$.
The bracketed stroke term is the crux of the whole method. A real two-legged
cyclist produces **two power pulses per revolution** plus a smaller once-per-
revolution gravity/asymmetry term. With $a_1=0.25$, $a_2=0.55$, the peak-to-mean
ratio is 1.8 and the RMS of the AC part is

$$\rho_{rms} \;=\; \sqrt{\tfrac{a_1^2+a_2^2}{2}} \;=\; 0.427 .$$

Published crank torque–angle data put the peak/mean ratio at 1.7–2.5 with a
2/rev-dominant harmonic content, so this is representative [1], [2].

### 2.6 Sensors

The crank cadence sensor is modelled as a realistic 12-magnet PAS: cadence is
computed from the interval between pulses, so it is **quantised and one pulse
late** (≈70 ms latency at 70 rpm), it is jittery at high cadence, and it reports
zero below the pulse timeout. Motor cadence is modelled as high-resolution with
0.02 rpm noise, current with 0.15 A RMS noise. Crank inertia and the electrical
lag are modelled explicitly. The estimator never sees the plant's internal states.

---

## 3. Observability analysis

This section is the intellectual core. Everything else follows from it.

### 3.1 The locked regime is rank-deficient

Consider steady, locked operation ($\omega_r=\omega_m=\omega$, $\dot\omega=0$).
The driveline equation becomes

$$0 = K_t i_m + \tau_r - b_0 - b_1\omega - b_2\omega^2 . \tag{3.1}$$

The measurements are $\omega$ (excellent) and $i_m$ (good). The unknowns are
$\tau_r$ and $b_0$. **One equation, two unknowns.** All the cadence information in
the world does not help: $\omega_r$ and $\omega_m$ are equal and carry no
independent content about the split.

This is the precise sense in which *"if rider cadence matches motor cadence we
cannot know whether the rider is struggling or ghost pedalling."* It is not a
tuning problem; it is a structural rank deficiency. Two corollaries:

* **Cadence deficit is not an effort signal.** A rider in a crowded street may sit
  deliberately at low cadence with low torque; a rider on a climb may sit at the
  same cadence with three times the torque. Cadence cannot distinguish them.
* **Motor current alone is not an effort signal either.** High current means high
  *total* load (grade × rider), not high *rider* load. Indeed, when the rider
  pushes harder at constant speed, motor current *falls*.

### 3.2 What breaks the degeneracy

Four physically independent mechanisms supply the missing equation.

**(1) Freewheel.** When $\omega_r<\omega_m$ the clutch is open and
$\tau_r^{\text{trans}}\equiv 0$ *by construction*. Substituting into the driveline
equation gives $b_0$ directly:

$$b_0 = K_t i_m - b_1\omega - b_2\omega^2 - J_d\dot\omega_m . \tag{3.2}$$

Every coast, every pedal pause, every descent is a **free ground-truth calibration
sample**. It is the only channel that is *exact* — no scale factor, no
physiological assumption — which makes it the reference that calibrates channel
(2). It is also the only channel that observes anything when the rider is not
pedalling.

**(2) Stroke ripple.** While locked, take the angle-synchronous AC component of
the torque balance. The load in (3.1) is smooth in $\varphi$; the rider's torque
is not. Hence

$$J_{tot}\,\dot\omega^{AC}(\varphi) = K_t\,i_m^{AC}(\varphi) + \tau_r^{AC}(\varphi)$$

so

$$\boxed{\ \tau_r^{AC}(\varphi) = J_{tot}\,\dot\omega^{AC}(\varphi) - K_t\,i_m^{AC}(\varphi)\ } \tag{3.3}$$

Both terms are measurable: $\omega_m$ is a high-quality virtual accelerometer, and
$i_m$ is measured. Equation (3.3) recovers the rider's **oscillating** torque with
no knowledge of the grade whatsoever. Its RMS amplitude $A$ then gives the
rider's *mean* torque through the rider's ripple-to-mean ratio:

$$\bar\tau_r = A / \rho_{rms}. \tag{3.4}$$

Substituting (3.4) into (3.1) **makes $b_0$ observable during a climb with no
freewheel events at all.** This is the single most important result in the report.

**(3) Motor effort.** When the controller is saturated ($|i^\star_m|\ge I_{max}$)
*and* the motor cadence is still sagging below its target, the motor is doing
everything it can. Whatever else is happening,

$$\tau_r \ \ge\ \hat\tau_L - K_t I_{max}. \tag{3.5}$$

This is a genuine one-sided bound, and it is the only use of the *target* current
and *target* cadence signals, which otherwise carry no extra information.

**(4) Inertia.** During transients the $J_{tot}\dot\omega$ term is huge: a
0.5 rad/s² acceleration on $J_{tot}=55$ kg m² is 27 Nm, comparable to the rider's
entire output. Accelerations therefore supply a load-model-independent view of the
*change* in rider torque. Note that the acceleration is taken from the **motor**
cadence, not the rider's: the motor signal is far cleaner, and when locked they are
the same signal.

### 3.3 The residual unobservability

Even with all four channels there is a fundamental limitation. Channels (1) and (2)
are the only absolute anchors, and both are low-rate: freewheel events are
opportunistic, and the ripple anchor arrives once per pedal revolution and is only
valid at quasi-steady speed. Between anchors the estimator integrates a drift-free
but *bias-sensitive* model. If the rider climbs for two minutes without coasting
*and* the road grade changes, the bias can only follow as fast as the ripple anchor
allows — roughly a few seconds. This residual is quantified in §5 and §6.

---

## 4. The Virtual Load Sensing Estimator (VLSE)

### 4.1 Structure

A three-state extended Kalman filter,

$$\mathbf{x} = [\,\omega_m,\ \tau_r,\ b_0\,]^T$$

with the standard constant-velocity/random-walk models for the two unknown
torques. The regime-dependent dynamics follow §2.2 exactly:

$$f(\mathbf{x}, i_m) = \Big[\ \frac{K_t i_m + \mathbb{1}_{locked}\tau_r - (b_0 + b_1\omega + b_2\omega^2)}{J_d + \mathbb{1}_{locked}J_c},\ 0,\ 0\ \Big]^T .$$

Process noise is tuned to the physical bandwidth of each quantity: the rider's
torque is allowed to move at pedal-stroke bandwidth ($q_\tau$ large), while the
grade bias is nearly constant ($q_{b_0}\ll q_\tau$).

Four scalar measurements update the filter. All are $1\times3$ rows, so the
per-sample cost is a few dozen floating-point operations. A fifth channel (M5)
acts on the output rather than on the state, for the stability reason given in
§4.5.

| # | Channel | Kind | $H$ | When |
|---|---|---|---|---|
| M1 | motor cadence $\omega_m$ | EKF update | $[1,0,0]$ | always |
| M2 | $\tau_r = 0$ | EKF update | $[0,1,0]$ | clutch open for ≥3 samples |
| M3 | $b_0 = \hat b_0^{free}$ | EKF update | $[0,0,1]$ | clutch open, anchor valid |
| M4 | $b_0 = \hat b_0^{ripple}$ | EKF update | $[0,0,1]$ | one valid revolution, locked, quasi-steady |
| M5 | $\tau_r \ge \hat\tau_L - K_tI_{max}$ | output floor | — | saturated + sagging |

### 4.2 Regime classification

Locked/free is decided from the two cadences with hysteresis
(return-to-locked at $\omega_m-\omega_r<0.20$ rad/s, enter-free at $>0.80$ rad/s)
and a minimum cadence gate. Hysteresis matters because a coasting crank near the lock
boundary chatters at the stroke frequency.

The crank sensor is the principal hazard, for two distinct reasons.

**(a) Latency.** When a rider re-engages, the sensor does not know for up to
~80 ms, during which the driveline is already being driven. Naively trusting the
classification poisons the load estimate. Four defences are used:

* the crank cadence is smoothed (~4 Hz) before classification, and **clamped to
  the motor cadence**, because $\omega_r \le \omega_m$ holds physically and any
  measured excess is therefore noise;
* the freewheel gate requires a clear disengagement margin
  ($\omega_m-\omega_r>0.8$ rad/s), which rejects the moment of re-lock;
* the anchor is a **median over a 41-sample (0.41 s) window** rather than an EMA,
  so a ≤10-sample corruption burst cannot move it;
* the anchor is only marked valid after **15 consecutive** clean samples, and the
  estimator refuses to form an anchor at all until at least one real crank pulse
  has been seen (otherwise a stationary, pulse-less crank looks "freewheeling" at
  start-up).

**(b) Noise magnitude.** The classifier is a threshold test on the *difference* of
two cadences. If the crank-pulse jitter is large enough that the cadence noise
exceeds the threshold, the classifier intermittently declares a freewheel and
applies the $\tau_r \equiv 0$ constraint while the rider is in fact driving. In
testing this is the single most destructive failure in the system: 5× pulse
jitter (≈7.5 ms) collapses the estimate toward zero (−8.9 Nm bias, §6). Filtering
the crank cadence and exploiting the physical clamp together recover about a third
of that, but a genuinely noisy PAS cannot be fully compensated — see §8.

Each of these defences was empirically necessary. Without them the calibrated
scale factor drifts by 50 %, the estimate acquires an 8–10 Nm bias, and the
estimator is not usable.

### 4.3 Freewheel anchor

While the gate of §4.2 is satisfied, accumulate

$$b_0^{(k)} = K_t i_m - b_1\omega_m - b_2\omega_m^2 - J_d\dot\omega_m$$

where $\dot\omega_m$ is a 3 Hz low-pass-filtered derivative of the motor cadence.
The anchor is the running median of the last 41 samples (0.41 s at 100 Hz);
the median window must be long enough that a sensor-latency burst is a minority
of it. Note that this is a
*complete* load identification, not a partial one: it fixes the grade bias
absolutely, with no scale factor involved.

### 4.4 Stroke-ripple anchor

While locked, accumulate motor cadence and current into 24 **angle bins** over one
crank revolution. At each revolution boundary:

1. Form the binned profile $\bar\omega(\varphi_b)$, $\bar\imath(\varphi_b)$.
2. Estimate the angular acceleration profile by central differences across bins,
   $\dot\omega_b = (\bar\omega_{b+1}-\bar\omega_{b-1})/(2\Delta t_b)$, where
   $\Delta t_b = (2\pi/N)/\bar\omega$. Binning *before* differentiating is what
   makes this work: it averages the cadence noise over every sample in the
   revolution and yields a clean acceleration profile from a signal that would be
   unusable if differentiated raw.
3. Compute $\tau_r^{AC}(\varphi_b)$ from (3.3) and its RMS amplitude $A$.
4. Form the bias observation
   $$\hat b_0^{ripple} = K_t\bar\imath + A/\rho_{rms} - b_1\bar\omega - b_2\bar\omega^2 - J_{tot}\,\overline{\dot\omega},$$
   applied once per revolution with $\sigma = 0.15\,(A/\rho_{rms}) + 1.0$.

The revolution is rejected unless the speed was quasi-steady
($|\overline{\dot\omega}|<0.06$ rad/s² and peak-to-peak spread $<25\%$ of mean)
and at least 85 % of the revolution was locked. This gate is essential: during a
hard acceleration the binned derivative is invalid and produces amplitudes of
100+ Nm, which corrupt the bias for tens of seconds afterwards.

Because the ripple comes from the rider's own biomechanics, it is present at
essentially all times when the rider is working, and absent when they are not —
which is exactly the discriminator §1.1 needs.

The channel has a **noise floor** set by the binned-difference acceleration
estimate, $\sigma_A \approx J_{tot}\sigma_\omega/(\Delta t_b\sqrt{n_b})\approx
1.2$ Nm for the modelled sensors at 70 rpm (it scales roughly with cadence).
Dividing by $\rho_{rms}$ gives a resolution floor of ≈2.6 Nm RMS on the mean rider
torque; see §5.4 and §8. Averaging the per-revolution amplitude over $M$
revolutions reduces it as $\sqrt{M}$.

### 4.5 Motor-effort bound

Evaluated every sample. If the commanded current is at ≥97 % of $I_{max}$ and the
motor cadence is more than 1 rpm below target, the bound (3.5) is computed and the
*output* is pulled toward it by a factor `sat_blend`:

$$\hat\tau_r^{out} = \hat\tau_r + \beta\,\max(0,\ \hat\tau_L - K_tI_{max} - \hat\tau_r),\quad \beta \approx 0.5 .$$

It is deliberately **not** fed back into the filter. A load-model-derived bound
fed back into the bias state closes a positive loop
(estimate → bias → bound → estimate) that diverges; in testing this drove the
uphill estimate to 81 Nm within seconds. Applied as an output floor it is stable.
A sweep (§5.6) shows $\beta\in[0.4,0.6]$ minimises MAE.

### 4.6 Self-calibration of the ripple ratio

$\rho_{rms}$ is rider-specific. It is calibrated automatically: on each valid
locked revolution, if a freewheel anchor was seen within the last 45 s, solve
(3.4) backwards using the *anchored* bias,

$$\rho_{rms}^{obs} = \frac{A}{\hat b_0^{free} - K_t\bar\imath + b_1\bar\omega + b_2\bar\omega^2 + J_{tot}\overline{\dot\omega}},$$

and blend it in with a slow gain. After the calibration converges, the freewheel
anchor is no longer needed for the scale — only for slow re-referencing.

A short **commissioning ride** is the recommended deployment procedure: coast for
a few seconds (freewheel anchor), then pedal steadily for 20–30 s (ripple ratio).
In simulation this converges to $\rho_{rms}=0.450$ against a true value of 0.427 —
a **5.4 % error** — and to $b_0 = 5.03$ Nm against a true 5.67 Nm. Most of the
residual calibration error traces to the small $b_0$ anchor error, which is why
the freewheel gate of §4.2 is worth its complexity.

### 4.7 Pseudocode

```
init:
    x  = [w_m_meas, 0, b0_nominal]          # b0_nominal ~ rolling resistance
    P  = diag(sig_w^2, 400, 400)
    anchors, ripple bins, rho_hat = nominal, empty, rho_0
    warmup_revolutions = 4

each sample (100 Hz), inputs c_r, c_m, i_m, i_cmd, c_m_star:
    # ---- 0. filtered derivative of motor cadence
    wdot_f += a * ((c_m - c_m_prev)/dt - wdot_f)

    # ---- 1. regime
    free = hysteresis(c_r, c_m, seen_crank)

    # ---- 2. freewheel anchor (median-of-window, gated)
    if free and free_count >= 2 and seen_crank
            and (c_m - c_r) > 0.5 and |wdot_f| < 2.0:
        push(anchor_buf, Kt*i_m - b1*c_m - b2*c_m^2 - Jd*wdot_f)
        b0_free = median(anchor_buf);  valid if len >= 15
    else:
        clear(anchor_buf)

    # ---- 3. EKF predict
    Jtot = Jd + (Jc if locked else 0)
    x    = propagate(x, i_m, locked, Jtot)
    P    = F P F' + Q dt

    # ---- 4. updates
    update(z = c_m,   H = [1,0,0], R = sig_w^2)
    if free_count >= 3:
        update(z = 0,        H = [0,1,0], R = sig_free^2)
        if anchor_valid: update(z = b0_free, H = [0,0,1], R = sig_b0_free^2)
    if locked and ripple_new and revolution_valid:
        z_tau = ripple_amp / rho_hat
        update(z = Kt*i_bar + z_tau - b1*w_bar - b2*w_bar^2 - Jtot*wdot_rev,
               H = [0,0,1], R = (0.15*z_tau + 1.0)^2)
        if anchor_recent:  rho_hat <- blend(rho_hat, ripple_amp / (b0_free - ...))

    # ---- 5. output floor
    tau_out = x[tau_r]
    if locked and |i_cmd| >= 0.97*Imax and (c_m_star - c_m) > 1.0:
        bound   = max(0, (x[b0] + b1*w + b2*w^2) - Kt*Imax)
        tau_out = tau_out + 0.5*max(0, bound - tau_out)

    # ---- 6. stroke-angle accumulator -> ripple_amp at each revolution
    return tau_out, sqrt(P[1,1]), rho_hat
```

Complexity: $\mathcal{O}(1)$ per sample, no allocation, no trigonometry except the
optional angle binning. The only state is a 3×3 covariance, a 24×2 ripple
accumulator and a 41-sample anchor buffer.

---

## 5. Simulation study

### 5.1 Setup

The plant of §2 is integrated at 1 kHz with an event-correct one-way clutch;
the estimator runs at 100 Hz on the sensor outputs only. A 95 kg rider+bike with
$C_{rr}=0.008$, $C_dA=0.50$ and a 500 W-class mid-drive
($K_t = 1.0$ Nm/A at the crank, $I_{max}=40$ A → 40 Nm peak) is used. Only the
*product* $K_tI_{max}$ is physically meaningful, and 40 Nm at the crank is
representative of a 500 W-class mid-drive; published crank-side figures for real
units are 2.0–2.9 Nm/A with 11–30 A limits [6], which gives a similar ceiling
with a smaller current. Full parameters are in Appendix A. Other choices are
anchored to published values: 100 W at 70 rpm corresponds to 13.6 Nm [1], which
is the steady-cruise operating point used here; the reflected inertia of 55 kg m²
sits inside the 11–176 kg m² band for real road gearing [1]; the 12-magnet PAS is
the common commercial configuration [5].

### 5.2 Scenarios

| # | Scenario | What it probes |
|---|---|---|
| 0 | Commissioning (coast, then steady pedal) | can $\rho_{rms}$ and $b_0$ be identified? |
| 1 | Steady flat, 20 km/h | baseline split between rider and motor |
| 2 | Crowded slow, 6 km/h, low gear, 3 % rise | does low cadence cause a false high-load reading? |
| 3 | 0 → 7 % climb, 15 km/h | motor saturates; rider takes the balance |
| 4 | −7 % descent | driveline runs away; rider must read ~zero |
| 5 | Steady, then 3.5 s pedal pause, then resume | freewheel anchor and re-engagement |
| 6 | Hard acceleration 12 → 25 km/h | inertial channel dominates |
| 7 | Ghost pedalling — **matched-cadence control for #1** | the decisive discriminator |
| 8 | Flat → climb → descent → flat with a pedal pause | composite |
| 9 | Continuous climb with a grade step and *no* freewheel | bias-observer stress test |

Scenarios 1 and 7 are deliberately constructed to be identical in speed, gearing,
cadence target, grade, duration and lock state — both run at 69.9 rpm — and differ
**only** in rider torque (12.99 Nm versus 1.50 Nm). The comparison is therefore a
clean matched control, not an analogy. Reference torque is the true
clutch-transmitted torque; errors are reported against a 0.25 s moving average,
which is the timescale relevant to assist control.

### 5.3 Results

**Table 1 — per-scenario accuracy.** MAE and bias of the 0.25 s-averaged estimate
against the 0.25 s-averaged truth, over engaged samples (truth > 3 Nm) after the
first 5 s. Headline steady-state window RMSEs are in Table 2.

| Scenario | MAE (Nm) | bias (Nm) | clutch classification |
|---|---|---|---|
| 1 steady flat 20 km/h | **1.63** | −1.14 | 99.6 % |
| 2 crowded slow 6 km/h | **1.01** | −1.01 | 99.6 % |
| 3 uphill 7 % | **3.91** | −2.25 | 99.7 % |
| 4 downhill −7 % | **0.00**† | 0.00 | 98.8 % |
| 5 stop & resume | **1.78** | −1.34 | 98.3 % |
| 6 hard acceleration | **2.65** | −0.77 | 99.2 % |
| 7 ghost pedalling | **1.54**† | +1.48 | 99.5 % |
| 8 mixed | **5.55** | −0.81 | 98.2 % |
| 9 climb, grade step, no freewheel | **3.87** | −3.34 | 99.8 % |

† scenarios 4 and 7 have no samples above the 3 Nm "engaged" threshold, so their
window RMSE is quoted instead of an engaged-sample MAE. Scenario 7's window is
truth 1.50 Nm, estimate 2.98 Nm.

Across all scenarios: **MAE 3.39 Nm, mean error −1.94 Nm, $r = 0.809$, P90
absolute error 6.8 Nm over 44 385 engaged samples** (`results/fig_summary.png`,
middle panel). The clutch classifier is correct 98.2–99.8 % of the time in every
scenario. Note that scenario 8's MAE (5.55 Nm) is almost entirely *variance*, not
bias (−0.81 Nm): the error is concentrated in the seconds around each regime
transition, while its steady windows are accurate to 1.9–3.4 Nm.

**Table 2 — steady-state windows (the operating points that matter for assist).**

| Scenario | window | true (Nm) | est. (Nm) | RMSE (Nm) |
|---|---|---|---|---|
| 1 | cruise 25–45 s | 12.98 | 11.98 | **1.89** |
| 2 | slow cruise 25–45 s | 6.73 | 5.77 | **0.98** |
| 3 | early climb 20–32 s | 20.67 | 20.77 | **2.57** |
| 3 | steady climb 55–70 s | 20.86 | 20.84 | **2.52** |
| 5 | before pause | 12.99 | 12.00 | **1.87** |
| 5 | after resume | 12.87 | 11.63 | **2.21** |
| 6 | acceleration 9–16 s | 27.23 | 28.99 | **3.73** |
| 6 | settled 30–45 s | 20.07 | 18.49 | **2.55** |
| 8 | flat | 13.61 | 12.44 | **2.06** |
| 8 | climb | 21.05 | 18.69 | **3.44** |
| 8 | descent | 0.00 | 0.00 | **0.00** |
| 8 | recovered | 12.27 | 11.27 | **1.93** |
| 9 | before grade step | 20.94 | 19.00 | **3.17** |
| 9 | after grade step (no freewheel) | 27.57 | 24.98 | **4.20** |

**Figures.** `results/fig_gallery.png` shows estimate versus truth for all nine
scenarios, with the three-channel cadence traces overlaid;
`results/fig_summary.png` gives the per-scenario MAE, the estimate-vs-truth
scatter and the error histogram; `results/<scenario>.png` gives a four-panel
diagnostic per scenario (cadences, torques with the ±2σ envelope, true vs
estimated clutch state, and the bias/ripple internals).

### 5.4 The scenarios, discussed

**Crowded riding (MAE 1.01 Nm, window RMSE 0.98 Nm).** This is the sharpest
demonstration that the method is not a disguised cadence heuristic. The rider sits
at 53 rpm — low enough that a cadence-based load proxy would report high effort —
but the actual load is only ~7 Nm, and the estimator reports 5.8 Nm. The ripple
amplitude is small because the rider is not working, and the residual against the
load model is small because the load is small. Both channels agree on "low load",
independent of cadence.

**Uphill (MAE 3.91 Nm, window RMSE 2.5 Nm).** The grade ramps from 0 to 7 % over
8 s. The motor saturates within ~4 s. The ripple anchor tracks the bias up to
~54 Nm (true 55.2) and the estimate follows the rider's rising effort. Breaking
the error down by phase is instructive:

| phase | true (Nm) | MAE (Nm) | bias (Nm) | share of total abs. error |
|---|---|---|---|---|
| flat, 5–8 s (cold start) | 12.67 | 5.45 | −5.45 | 6 % |
| grade transition, 8–18 s | 18.04 | **12.02** | −12.02 | **47 %** |
| climb after the ramp, 18–70 s | 20.87 | **2.26** | −0.19 | 46 % |

Once the ramp is over the estimate is essentially unbiased (−0.19 Nm), and in the
55–70 s window it is 20.84 Nm against a truth of 20.86 Nm. The scenario's MAE is
therefore dominated by the two *learning* phases — a 40 Nm change in load must be
absorbed by the bias observer, which takes the eight seconds of the ramp — plus a
cold-start transient. This is the price of the observability structure: the bias
cannot be known faster than an anchor can measure it. The motor-effort bound is
active 56.5 % of the time overall and 73 % of the climb (binding on 32 % overall,
42 % of the climb); in a controlled A/B with everything else held fixed it moves
the scenario from MAE 4.14 / bias −3.58 Nm (β = 0) to MAE 3.91 / bias −2.25 Nm
(β = 0.5).

**Downhill (0.00 Nm).** The driveline runs away, the crank freewheels, the
classifier detects it, and the estimate correctly reads zero throughout the
descent. This is also a perfect calibration event. Note the physical subtlety: the
rider is *pedalling* in this scenario (their legs are moving) — they simply cannot
transmit torque, because the freewheel is open. The estimator gets this right,
which a cadence-only system fundamentally cannot.

**Stop and resume (RMSE 1.87 → 2.21 Nm, MAE 1.78 Nm).** During the 3.5 s pause the estimate
collapses to zero and the bias is re-anchored with ground truth. On resumption the
crank spins up and re-locks within ~40 ms; the sensor-latency burst is rejected by
the median anchor and the margin gate, and the estimate recovers to within
2.21 Nm. Critically, *the pause makes the estimate better, not worse* — it is a
calibration gift.

**Ghost pedalling — the decisive test (est. 2.98 vs true 1.50 Nm).** Speed,
gear, cadence target, grade, duration and lock state are identical to scenario 1;
both run at 69.9 rpm. Only the rider torque differs: 1.50 Nm here versus
12.99 Nm there — a factor of 8.7. A cadence-based controller would apply the
*same* assist in both cases. The VLSE reports 2.98 Nm instead of 12.00 Nm: the
assist error falls from 10.5 Nm to 1.5 Nm.

The residual +1.5 Nm has a specific and instructive cause. The measured ripple
amplitude in this scenario is **1.32 Nm RMS, whereas the rider's true AC torque is
only 0.64 Nm** — a *demodulation noise floor*. The floor is not a mystery: the
binned central-difference acceleration estimate has noise
$\sigma_{\dot\omega}\approx\sigma_\omega/(\Delta t_b\sqrt{n_b})$, so its torque
equivalent is

$$\sigma_A \approx J_{tot}\,\frac{\sigma_\omega}{\Delta t_b\sqrt{n_b}}
            \approx 55 \cdot \frac{0.0021}{0.036\cdot 1.9} \approx 1.2\ \text{Nm},$$

and $\sqrt{0.64^2+1.2^2}=1.36$ Nm, which is what is measured. Dividing by
$\rho_{rms}$ gives a **resolution floor of ≈2.6 Nm RMS on the mean rider torque**,
so the estimate is $\sqrt{1.5^2+2.6^2}=3.0$ Nm — exactly what is reported. The
floor scales roughly with cadence (it was ≈1.7 Nm of AC torque at 87 rpm).

Above the floor the ripple channel is accurate to 5–10 % (measured 5.38 vs
5.55 Nm at cruise, 8.54 vs 8.93 Nm on the climb); below it, the estimate is
dominated by demodulation noise. The estimator therefore cannot honestly
distinguish "0 Nm" from "3 Nm" — but it can distinguish either from 13 Nm, which
is the distinction that matters.

The floor is reducible by roughly $\sqrt{M}$ by coherently averaging the
per-revolution amplitude estimate over $M$ revolutions (at the cost of response
time), or by reducing $\sigma_\omega$ or the bin width. In the current
implementation the floor is deliberately accepted in exchange for
one-revolution response.

**Hard acceleration (MAE 2.65 Nm, window RMSE 3.73 / 2.55 Nm).** Early versions of
the estimator were worst here (MAE 6.5 Nm), because the ripple channel is
correctly gated out during high $\dot\omega$ and the motor current falls to
essentially **zero** in the settled phase — the rider is supplying the entire
load. Hardening the lock classifier (§4.2) turned this into one of the better
scenarios: with a reliable regime decision, the inertial channel
$J\dot\omega$ carries the transient and the load model carries the settled state,
where the estimate lands at 18.49 Nm against a 20.07 Nm truth.

**Climb with a grade step and no freewheel (RMSE 3.17 → 4.20 Nm, MAE 3.87 Nm).** The grade
steps from 5.5 % to 7.8 % at t = 100 s and the rider never coasts again. This is
the worst case for a bias observer. The estimator absorbs the step over a few
seconds — the ripple anchor needs only one quasi-steady revolution to see the new
torque level — and lands within 2.6 Nm. Without the ripple channel this scenario
fails completely (Table 3).

### 5.5 Ablation: which channel actually matters?

**Table 3 — MAE (Nm) with each anchor disabled.** The ripple ratio is calibrated
once by the commissioning ride and then held fixed in every row, so the ablation
isolates each anchor's *online* contribution rather than its calibration value.

| Configuration | steady | crowded | uphill | stop/resume | accel | drift |
|---|---|---|---|---|---|---|
| full (both anchors) | **1.63** | **1.01** | **3.91** | **1.78** | **2.65** | **3.87** |
| no ripple anchor | 6.37 | 2.50 | 10.57 | 5.30 | 10.61 | 11.81 |
| no freewheel anchor | 1.63 | 1.01 | 3.91 | 1.86 | 2.65 | 3.87 |
| neither anchor | 6.37 | 2.50 | 10.57 | 6.26 | 10.61 | 11.81 |

The result is unambiguous, and it overturned the author's own prior expectation:
**the stroke-ripple anchor is the primary bias observer.** Disabling it degrades
accuracy by 2–4× everywhere (cruise 1.6 → 6.4 Nm; uphill 3.9 → 10.6 Nm; the
no-freewheel drift test 3.9 → 11.8 Nm). Disabling the freewheel anchor, by
contrast, changes essentially nothing — because whenever the rider is pedalling
steadily the ripple anchor supplies the same information at a comparable rate.

This does **not** make the freewheel channel optional. Its irreplaceable roles are:

* **calibration.** It is the only reference for $\rho_{rms}$ that does not itself
  depend on $\rho_{rms}$. Without it the scale factor has no absolute anchor and
  the estimate inherits a proportional gain error (§6).
* **the non-pedalling case.** On a long descent, or during a pedal pause, the
  ripple channel has nothing to measure. Only the freewheel anchor observes $b_0$
  then — and it is precisely the moment the estimate must read zero.
* **start-up.** Before the first revolution completes, the ripple anchor cannot
  fire.

The correct statement is therefore: *the ripple channel carries the day-to-day
estimate; the freewheel channel makes it calibratable and keeps it alive when the
rider is not pedalling.* A design that omitted either one would fail in a
different, and in both cases severe, way.

### 5.6 Motor-effort blend sweep

*(Scenarios 1, 2, 3, 5, 6; single noise seed.)*

| `sat_blend` β | overall MAE (Nm) | overall bias (Nm) |
|---|---|---|
| 0.0 (channel disabled) | 2.45 | −1.79 |
| 0.4 | **2.37** | −1.49 |
| 0.5 (used) | 2.38 | −1.41 |
| 0.6 | 2.40 | −1.33 |
| 1.0 (hard floor) | 2.56 | −1.02 |

The channel trades variance for bias, and the effect is modest: the bias improves
by 0.8 Nm between β = 0 and β = 1 while the MAE worsens by 0.1 Nm. β = 0.5 is a
reasonable compromise; systems that prefer a conservative (never-under-report)
assist should use β = 1.0 and accept the extra noise on saturated climbs.

---

## 6. Robustness and sensitivity

*(Sweep configuration: scenarios 1, 2, 3, 5, 6; two noise seeds; MAE and bias of
the 0.25 s-averaged estimate over engaged samples. The ripple ratio is
re-calibrated by the commissioning ride for every case, so the "no commissioning"
rows are the exception.)*

**Table 4 — sensitivity to model error, calibration error and sensor quality.**
The nominal row is the reference; everything else is one perturbation at a time.

| Perturbation | MAE (Nm) | bias (Nm) | P90 abs. err (Nm) |
|---|---|---|---|
| **nominal** | **2.40** | −1.42 | 4.50 |
| $K_t$ −10 % | 2.46 | −1.44 | 4.61 |
| $K_t$ +10 % | 2.38 | −1.39 | 4.52 |
| $J_d$ −20 % | 4.05 | −3.99 | 8.04 |
| $J_d$ +20 % | 3.03 | +1.12 | 7.17 |
| aero $b_2$ −25 % | 2.40 | −1.45 | 4.50 |
| aero $b_2$ +25 % | 2.41 | −1.38 | 4.55 |
| viscous $b_1$ −30 % | 2.40 | −1.42 | 4.50 |
| viscous $b_1$ +30 % | 2.40 | −1.41 | 4.50 |
| $
ho_{rms}$ forced to 0.35 (−20 %) | 3.48 | +1.90 | 7.88 |
| $
ho_{rms}$ forced to 0.45 (no commissioning) | 2.40 | −1.41 | 4.49 |
| $
ho_{rms}$ forced to 0.55 (+22 %) | 3.67 | −3.56 | 6.86 |
| crank pulse jitter ×5 | **8.90** | −8.88 | 16.20 |
| motor cadence noise ×10 | 4.22 | +2.91 | 8.56 |
| current noise ×5 | 2.38 | −1.31 | 4.43 |
| all sensors degraded at once | 5.61 | −1.19 | 12.06 |

The results are cleaner and more interesting than the author expected, and they
change the engineering priorities:

**Load-model coefficients are almost entirely self-correcting.** ±10 % on $K_t$,
±25 % on the aerodynamic coefficient and ±30 % on the viscous coefficient each
move the MAE by less than 0.1 Nm. The reason is structural: $b_0$ is a *free*
parameter that absorbs any constant mismatch between the model and reality, and
the freewheel and ripple anchors both *measure* $b_0$ rather than assuming it.
This is the single most valuable property of the architecture — it means the
system does not need a carefully calibrated vehicle model, only a roughly right
one. (The corollary is that a coefficient error only bites where it is *not*
constant, i.e. when speed varies widely; a 25 % $b_2$ error is worth ~2.8 Nm
between 15 and 25 km/h, which is why scenario 6 remains the hardest.)

**Inertia is the exception.** $J_d$ enters the *transient* balance directly and
cannot be absorbed by a constant bias. A −20 % error costs +1.7 Nm of MAE and
−2.6 Nm of bias. $J_d = m\rho^2$ is easy to compute from mass and gearing, so
this is a non-issue in practice — but it must be got approximately right, and it
changes with gear.

**The ripple-ratio calibration is the dominant multiplicative error.** Because
$\bar\tau_r = A/\rho_{rms}$, a fractional error in $\rho_{rms}$ is a fractional
error in the estimate at every operating point. −20 % gives +1.90 Nm of bias,
+22 % gives −3.56 Nm, and both roughly increase MAE by 40–50 %. This is exactly
why the commissioning ride exists. Note that an *uncalibrated* fixed value of
0.45 — close to the true 0.427 — performs as well as a full calibration, which
means the practical requirement is a roughly-right prior, not a precise one.

**Crank-sensor quality dominates everything else.** Multiplying the PAS pulse
jitter by five (to ~7.5 ms) triples the MAE to 8.9 Nm with a −8.9 Nm bias, i.e.
the estimate collapses toward zero. The mechanism is worth stating because it was
found the hard way: the lock classifier compares the two cadences against a
threshold, and if the crank-cadence *noise* exceeds that threshold the classifier
intermittently declares a freewheel and applies the $\tau_r \equiv 0$ constraint
when the rider is in fact driving. The hardening in §4.2 — low-pass-filtering the
crank cadence and exploiting the fact that $\omega_r \le \omega_m$ is a hard
physical constraint, so any measured excess is noise — reduces the 5×-jitter MAE
from 11.8 to 8.9 Nm, and is what allowed the nominal MAE to improve from 2.9 to
2.4 Nm at the same time. A jittery PAS remains the most likely field failure
mode, and the practical recommendation is a mechanically rigid magnet ring and,
if the budget allows, a higher pulse count.

**Motor cadence noise damages calibration rather than the estimate.** A 10×
increase (to 0.2 rpm RMS) pushes the self-calibrated $\rho_{rms}$ to its upper
clamp (0.85) because noise inflates the measured ripple amplitude, and the MAE
rises to 4.22 Nm. Clamping $\rho_{rms}$ to a physiological band
(0.20–0.85) bounds the damage; a principled fix would be to subtract the
estimated noise floor from the ripple amplitude before forming $\rho_{rms}^{obs}$.
Current noise is essentially irrelevant (2.38 Nm at 5×).

---

## 7. From estimate to assist

The VLSE output is a rider torque, so it can drive a **torque-sensor emulation**
rather than a cadence switch:

$$\tau_{assist} = \mathrm{clip}\big(k_s\,\hat\tau_r,\ 0,\ \tau_{assist,max}\big)$$

with $k_s$ the assist ratio (e.g. 1.0–3.0, i.e. 100–300 % of rider torque). Three
practical refinements:

1. **Confidence gating.** The filter's $\sqrt{P_{11}}$ is a live uncertainty
   estimate. Scale the assist ratio down as uncertainty rises, or fall back to a
   low, fixed cadence-based assist when it exceeds a threshold. This is how the
   system behaves safely during the first seconds after power-on and immediately
   after a sensor dropout.
2. **Rate limiting and blanking on regime change.** On a lock→free transition the
   estimate must fall to zero; a short output ramp avoids a lurch. Conversely, on
   re-engagement the estimate can overshoot for one stroke while the anchor
   settles.
3. **Use the bound, not the point estimate, for safety.** On a saturated climb the
   lower bound (3.5) is the conservative quantity; an assist law that never
   under-assists a struggling rider should key off the bound, while one optimising
   range should key off the point estimate.

Because the estimator is purely observational (it never actuates), it cannot
destabilise the cadence loop. What *can* change the plant is the assist itself,
which is a closed-loop consideration outside this report; note only that raising
assist reduces measured motor current at fixed speed, which the residual channel
interprets correctly as "the rider now needs less help", so the loop is negative
feedback rather than positive.

---

## 8. Limitations and failure modes

Honest enumeration, in rough order of severity:

1. **Crank-sensor timing quality.** The single largest sensitivity (§6). The lock
   classifier is a threshold test on the difference of two cadences, and pulse
   jitter that approaches the threshold intermittently forces the estimate to zero.
   5× jitter triples the MAE. Mitigations: a rigid magnet ring, a higher pulse
   count, and the filtering and physical clamping of §4.2. This is the failure mode
   most likely to be met in the field.
2. **Rider-specific ripple ratio.** The absolute calibration of the ripple channel
   depends on how pulsatile a given rider's stroke is. Riders who pull up on the
   pedals, or who pedal very smoothly (high cadence, low torque), have a different
   $\rho_{rms}$. The self-calibration handles the common case but needs a
   freewheel event plus a steady pedalling interval to work. −20 % calibration
   error costs ~40 % in MAE. A rider-adaptive (rather than fixed) estimate is the
   recommended mitigation.
3. **Resolution floor.** The demodulation noise floor (≈1.2 Nm of AC torque at
   70 rpm, ⇒ ≈2.6 Nm RMS on the mean) means the estimator cannot resolve rider
   torques below roughly 2–3 Nm. It reads 3.0 Nm for a rider producing 1.5 Nm.
   For assist scaling this is second-order; for a rider-power *display* it should
   be clamped to zero below the floor rather than shown as a number.
4. **Aerodynamics and wind.** $b_2$ is calibrated on a still day. A headwind adds
   an unmodelled load that the bias observer absorbs slowly and that is
   indistinguishable from a grade. Because $b_0$ self-corrects, this is much less
   damaging than expected — but it matters at high speed where the estimate leans
   on the aero term, and it is worth several Nm in strong wind.
5. **Long climbs with no freewheel and a changing grade.** Quantified in scenario
   9: 3.2–4.2 Nm RMSE, with the error concentrated in the few seconds after a
   grade change. Bounded, but real.
6. **Braking and drivetrain changes.** Brake drag looks exactly like a grade.
   A gear change changes $\rho$, hence $b_2$ and $J_d$; the estimator must be given
   the current gear or must re-identify. A belt/chain efficiency change
   (temperature, wear) shifts $b_1$ — although Table 4 shows the system is
   insensitive to $b_1$ precisely because $b_0$ absorbs it.
7. **Ghost-pedal bias.** ~+1.5 Nm, and now explained: it *is* the resolution
   floor of item 3, not a modelling error. Harmless for assist, since it is a
   small fraction of the reference torque, but it means the system will never
   report exactly zero.
8. **Regime-transition transients.** Scenario 8's 5.5 Nm MAE is almost entirely
   variance around regime changes: when the lock state flips, the estimator must
   re-solve the rider/load split, and it takes a few seconds of pedalling for the
   ripple anchor to re-establish it. Assist laws should ramp rather than step
   across these events.
9. **Simulated, not measured.** All results here are from a model with a
   particular rider model and a particular drivetrain topology. The qualitative
   conclusions — the observability structure, the need for an absolute anchor, the
   self-correcting role of $b_0$ — are model-independent. The numeric accuracies
   are not. Validation on a real bike with an instrumented crank is the obvious and
   necessary next step.
10. **Topology assumption.** If the real system is the mirror image (the motor
   *drives* the crank through a clutch rather than the crank driving the
   driveline), the sign of "who is contributing" in the freewheel regime changes,
   but the four-channel architecture is unchanged: the freewheel anchor still gives
   a known-torque state, and the ripple channel is unaffected.

---

## 9. Implementation notes

* **Rates.** Plant/estimator at 100 Hz is sufficient. The cadence loop may run
  faster; the estimator only needs the motor cadence and current.
* **Cost.** $\mathcal{O}(1)$ per sample: a 3×3 predict/update (≈60 FLOPs), plus
  angle binning. No matrix libraries, no allocation, no transcendentals in the hot
  path. Comfortably runs on a Cortex-M0 at 100 Hz with the 3×3 covariance in
  fixed point; a float build is easier to tune.
* **Memory.** $3\times3$ covariance, 3-state vector, a $24\times2$ ripple
  accumulator, and a 41-sample anchor ring buffer — under 100 floats total.
* **Binning detail.** Accumulate *raw* motor cadence and current per bin, not
  derived quantities; take means at the revolution boundary, then differentiate.
  Do not differentiate raw cadence.
* **Anti-windup.** Both the freewheel median window and the ripple accumulator
  must be cleared on a regime change; a stale particle is worse than a missing
  one.
* **Tuning order.** (i) $q_\tau$ sets output bandwidth; (ii) $q_{b_0}$ and the
  ripple $\sigma$ set bias-tracking speed versus jitter; (iii) $b_0$ nominal
  initialisation removes the start-up transient; (iv) `sat_blend` last.

---

## 10. Conclusion

A cadence-limited pedelec with no torque sensor cannot, even in principle, know
the rider's torque from cadence and motor current at steady state: the locked
driveline gives one equation in the two unknowns (rider torque, grade). That is a
rank deficiency, not a tuning problem, and it is the formal statement of the
user's observation that "cadence matching" cannot distinguish struggling from
ghost pedalling.

It becomes solvable once two physical facts are exploited: **when the clutch is
open the rider's transmitted torque is exactly zero** (an exact, opportunistic
calibration), and **the rider's torque is strongly periodic at the pedal
frequency** (a periodic, grade-free measurement of its amplitude). Adding a
motor-saturation bound and the inertial term for transients yields an estimator
that, in simulation, tracks rider torque to 1.0–3.9 Nm MAE across crowded riding,
cruising, climbing, coasting, pedal pauses and hard acceleration, with steady-state
window RMSE of 1.0–3.4 Nm and a pooled 90th-percentile absolute error of 6.8 Nm.

The decisive result for the original question is scenario 7, a matched-cadence
control: at 69.9 rpm, the *same* cadence and clutch state as the 13 Nm cruise, the
estimator reports 3.0 Nm for a ghost-pedalling rider — collapsing an 8.7× assist
error to 1.5 Nm. A cadence sensor says "70 rpm"; the virtual torque sensor says
"70 rpm and you are doing almost nothing".

Two findings from the sensitivity study deserve to travel with the design. First,
the architecture does **not** need an accurate vehicle model: a free bias parameter
plus two independent anchors that *measure* rather than assume it make ±10–30 %
errors in the torque constant and resistance coefficients nearly irrelevant. The
thing that must be right is the **ripple-ratio calibration**, a single
rider-specific scalar, and the **crank cadence sensor**, which is the dominant
practical risk. Second, the method's accuracy is ultimately limited not by the
algorithm but by how well two physical quantities can be observed at all: the
rider's stroke pulsatility and the quality of a cheap magnet ring.

---

## Appendix A — simulation parameters

| Symbol | Value | Note |
|---|---|---|
| $m$ | 95 kg | rider + bike |
| $C_{rr}$ | 0.008 | e-bike tyre range 0.005–0.010 |
| $C_dA$ | 0.50 m² | upright rider 0.40–0.60 |
| $\rho_{air}$ | 1.20 kg/m³ | |
| $\rho$ | 0.76 m/rad (0.30 low gear) | mid / low gear |
| $J_c$ | 0.15 kg m² | crank + legs |
| $J_d$ | $m\rho^2 + 0.05$ ≈ 55 kg m² | reflected vehicle |
| $b_c$ / $b_1$ | 0.05 / 0.30 Nm/(rad/s) | |
| $b_2$ | $0.5\rho_{air}C_dA\rho^3$ ≈ 0.13 | gear-dependent |
| $K_t$ | 1.0 Nm/A | crank-referenced |
| $I_{max}$ / $I_{min}$ | 40 A / 0 (assist-only) | → 40 Nm peak |
| $\tau_e$ | 15 ms | current loop |
| $K_p,K_i$ (cadence loop) | 60 A/(rad/s), 120 A/rad | |
| $\tau_{peak}$ | 55 Nm | rider force–velocity ceiling |
| $\omega_{max}$ (rider) | 21 rad/s ≈ 200 rpm | zero-torque cadence |
| $a_1,a_2$ | 0.25, 0.55 | → peak/mean 1.8, $\rho_{rms}=0.427$ |
| PAS | 12 pulses/rev, 1.5 ms jitter | |
| motor cadence noise | 0.02 rpm RMS | |
| current noise | 0.15 A RMS | |

## Appendix B — sources used for parameter anchoring

1. Bicycle performance / power and cadence reference data (mean crank torque
   $T = 9.5493\,P/\text{rpm}$; gearing and reflected inertia; $C_{rr}$, $C_dA$,
   drivetrain efficiency, air density).
   <https://en.wikipedia.org/wiki/Bicycle_performance>
2. Sprint crank torque and peak power in trained cyclists (peak 207 ± 38 Nm;
   theoretical $T_0$ 165 ± 25 Nm).
   <https://pmc.ncbi.nlm.nih.gov/articles/PMC11534655/>
3. Pedalling cadence preference and its dependence on power and experience.
   <https://pubmed.ncbi.nlm.nih.gov/9309635/>
   and low-cadence classification 50–70 rpm.
   <https://pmc.ncbi.nlm.nih.gov/articles/PMC11559993/>
4. Crank inertial load in cycling ergometry (does not affect muscle activity;
   0.2–5 kg m² for fixed ergometers).
   <https://pubmed.ncbi.nlm.nih.gov/16032416/>
5. 12-magnet / 12-pulse-per-revolution PAS sensors (commercial specification).
   <https://www.noon.com/qatar-en/assistant-sensor-12-magnets-12-pulses-per-revolution-e-bike-assistant-sensor-speed-sensor-for-electric-bicycle-pedal/Z2BC46D6477BB16C6EF1BZ/p/>
6. Mid-drive controller current limits (TSDZ2 ≈11 A; BBS02 25 A stock / 30 A
   custom) used to bound $K_t I_{max}$.
   <https://github.com/danielnilsson9/bbs-fw/discussions/198>
7. Rolling resistance of e-bike/touring tyres measured on drums (≈0.0047–0.010).
   <https://www.bicyclerollingresistance.com/tour-reviews>
8. **Prior art — sensorless pedalling torque observation using unknown-input
   observers**, IEEE 2025. The closest published approach; it also treats rider
   torque as an unknown input to a driveline model, but relies on a driveline
   model and does not use stroke-synchronous ripple demodulation or a
   freewheel-anchored bias identification.
   <https://ieeexplore.ieee.org/abstract/document/10839373/>
9. Torque measurement and control for electric-assisted bicycles under varying
   external load (MDPI Sensors 23(10):4657, 2023) — a torque-sensored benchmark.
   <https://www.mdpi.com/1424-8220/23/10/4657>
10. Contactless clutch-race torque sensing patent (SRAM EP4438917A2), which
    documents the cost/complexity of the hardware this report seeks to replace.
    <http://data.epo.org/publication-server/rest/v1.2/patents/EP4438917NWA2/document.html>
