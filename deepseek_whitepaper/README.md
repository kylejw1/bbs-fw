# Virtual Load Sensing Estimator (VLSE)

Design report, reference implementation and simulation study for a **software
pedal-torque / rider-load sensor** for an e-bike that has no torque sensor.

* **Whitepaper:** [`whitepaper.md`](whitepaper.md) — the full design report,
  observability analysis, algorithm, results and limitations.
* **Code:** [`sim/`](sim/) — the plant simulator, the estimator, the nine
  scenarios, and the experiment runners.

## The problem in one paragraph

A cadence-only pedelec knows rider cadence, motor cadence, target max motor
cadence, motor current and target motor current. Because the drivetrain contains a
one-way clutch, the rider can never be faster than the motor, and when the two are
locked they share one degree of freedom. In that locked regime the driveline
torque balance has two unknowns — rider torque and the grade/rolling load — and
constant-speed data gives one equation. **Rider torque is structurally
unobservable.** The VLSE breaks the degeneracy with four physical channels:
a freewheel anchor (open clutch ⇒ rider torque is exactly zero), a stroke-ripple
anchor (the rider's torque is periodic at pedal frequency, so its amplitude is
measurable without knowing the grade), a motor-saturation bound, and the inertial
term during transients. These are fused in a 3-state EKF.

## Quick start

```bash
python3 -m sim.run              # full scenario suite -> results/run.txt, figures
python3 -m sim.summary_plots    # gallery + summary figures
python3 -m sim.robustness       # sensitivity sweep (slow, ~15 min)
python3 -m sim.ablation         # anchor ablation study
```

Requires Python 3.10+ with `numpy` and `matplotlib`. No other dependencies.

## Files

| Path | What it is |
|---|---|
| `whitepaper.md` | the design report |
| `sim/model.py` | plant: bike, rider, motor + cadence controller, one-way clutch, sensor models |
| `sim/estimator.py` | the VLSE itself (regime classifier, EKF, four channels, self-calibration) |
| `sim/scenarios.py` | the nine operating scenarios |
| `sim/run.py` | runs every scenario, computes metrics, writes per-scenario figures |
| `sim/summary_plots.py` | gallery and summary figures |
| `sim/robustness.py` | model-error / calibration-error / sensor-quality sweeps |
| `sim/ablation.py` | disables each anchor in turn |
| `results/` | outputs (generated) |

## Conventions

All quantities are **crank-referenced SI**: angles and speeds in rad/s, torques in
Nm at the crank, and `rho` (metres travelled per crank radian) carries the gear
ratio. "Motor cadence" therefore means the motor speed *expressed at the crank*,
which is the only frame in which the constraint `cadence_rider <= cadence_motor`
is meaningful.

## Headline results

0.25 s-averaged rider torque, engaged samples, nine scenarios:

| Scenario | MAE (Nm) | bias (Nm) |
|---|---|---|
| steady 20 km/h | 1.63 | −1.14 |
| crowded 6 km/h | 1.01 | −1.01 |
| 7 % climb | 3.91 | −2.25 |
| −7 % descent | 0.00 | 0.00 |
| 3.5 s pedal pause + resume | 1.78 | −1.34 |
| hard acceleration | 2.65 | −0.77 |
| **ghost pedalling (matched-cadence control vs the 13 Nm cruise)** | **1.54** | **+1.48** |
| mixed | 5.55 | −0.81 |
| climb, grade step, no freewheel | 3.87 | −3.34 |

Overall r = 0.809, mean error −1.94 Nm, P90 |error| 6.8 Nm over 44 385 samples.
Scenarios 1 and 7 are identical in speed, gear, cadence target and lock state
(69.9 rpm); only rider torque differs, 12.99 Nm vs 1.50 Nm.

## Status

Simulation-only. The qualitative conclusions (observability structure, the
self-correcting role of the bias state, the need for an absolute anchor) are
model-independent; the numeric accuracies are not. Validation against an
instrumented crank on a real bike is the necessary next step.
