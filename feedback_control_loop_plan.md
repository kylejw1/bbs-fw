# Closed-Loop Pedal Assist for BBSHD — Plan

Status: design + step 1 (hall sensor telemetry) implemented.

## 1. Goal

Replace the current open-loop cadence→current map with a **closed-loop pedal assist**
that holds a target motor/output speed by modulating motor current, so that:

- heavy load (hill, headwind, tall gear) no longer makes cadence sag,
- assist does not switch on and off (the current "lugs off/on" behaviour),
- behaviour is predictable and tunable rather than a pile of empirical thresholds.

Non-goals: replacing the NEC motor controller firmware, torque-sensor-grade force
measurement, and any change to the display/config protocols beyond extra telemetry.

## 2. Topology

```
        ┌─────────────────────┐  1200 baud  ┌───────────────┐
        │  STC15W4K61S4       │◄───────────►│ Bafang display│
        │  (this firmware)    │  extcom     │  / config tool│
        │                     │             └───────────────┘
        │  PAS  ── cadence    │
        │  SPEED ── wheel rpm │  4800 baud  ┌───────────────┐
        │  HALL ── motor rpm  │◄───────────►│ NEC 79F9211   │──► 3-phase bridge
        │  ADC  ── throttle   │  0x63/0x64  │ (closed FW)   │
        └─────────────────────┘             └───────────────┘
```

The NEC is a black box that accepts two setpoints over a 4800 baud link and never
reports its own speed:

| Opcode | Meaning |
|---|---|
| `0x61` | absolute max current, in ADC counts, set once at boot |
| `0x64` | target current, **% of the `0x61` limit** (our actuator) |
| `0x63` | target speed, 0–250 steps (0–100 % of motor max) |
| `0x40`/`0x41`/`0x42` | status flags / battery current / battery voltage |

`0x63` is a **limiter**, not a servo setpoint (pre-2021 firmware sent `0xFF` under the
comment "never limit motor rotation speed"). `0x64` is a torque/effort command.

### Key enabling fact

The three motor hall signals are wired to **both** MCUs through a shared 3 kΩ series
resistor (`drawings/pcb/bbshd.sch`, motor connector JP4):

| JP4 | Signal | STC pin | NEC pin |
|---|---|---|---|
| 3 | W (grey) | P0.6 | NEC_P121 |
| 4 | U (white) | P5.0 | NEC_P120 |
| 5 | V (blue) | P3.4 | NEC_P122 |

So the stock firmware's `PIN_HALL_U/V/W` in `bbsx/pins.h` (commented out, never used)
are real and give us **true motor rpm feedback at zero hardware cost**. The STC must
read them as high-impedance inputs; the NEC shares the line.

Because all three halls are captured, we get **6 state transitions per electrical
revolution** (vs 2 from a single hall), and the transition *sequence* also yields
direction for free.

### 2.1 Measurement design (implemented)

The hall pins are sampled inside the existing 100 µs timer0 ISR. Counting any change
of the 3 bit hall state means the absolute pin order is irrelevant — every electrical
revolution produces the same 6 transitions. A window is closed and published when
**12 edges or 500 ticks (50 ms)** have accumulated, whichever comes first, which
keeps the update rate tied to the edge rate at low speed (3.4 ms at full speed, 50 ms
at low speed). The rpm division happens outside the ISR.

Two gotchas that cost real accuracy and are easy to reintroduce:

- **The tick counter must increment on every tick including the one carrying an
  edge.** Incrementing only on non-edge ticks makes every interval measure one tick
  short. That is negligible for the PAS and speed sensors (intervals of thousands of
  ticks) but dominates the hall interval, which is only ~3.4 ticks at full speed —
  it read ~40 % high before the fix.
- **Sum the ticks over the whole window rather than using a single interval.** With
  integer tick resolution, a true 3.4 tick interval can only be sampled as 3 or 4;
  only the window sum recovers the true elapsed time.

With both fixes the reported speed is within ±1.5 % from 1 to 250 output rpm
(verified by simulating the ISR logic against synthetic edge streams).

## 3. Plant model

```
u (0x64 %) ──► NEC current loop (fast, closed, unknown bandwidth) ──► T ≈ k·u
T ──► rotor speed ω :   J·dω/dt = T − T_load(grade, drag, rider, gears)
```

Three properties dominate the design:

1. **The plant is an integrator.** Inertia dominates; load torque is a disturbance.
2. **The actuator is a torque command, not a velocity command.** The NEC already
   closes a current loop; we should treat `u` as torque and not try to outrun it.
3. **The load gain is time-varying and unknown.** Gear shifts change it by 3–4×,
   grade and wind change it continuously, and the rider is a torque source with their
   own dynamics inside the loop.

Measured quantities and their latency:

| Quantity | Source | Latency | Use |
|---|---|---|---|
| pedal cadence | PAS sensor | ~ one pulse | outer setpoint, engage/disengage |
| motor rotor speed | halls | window closes at 12 edges or 50 ms | **inner loop feedback** |
| wheel speed | speed sensor | 1 pulse/rev (slow at low speed) | outer speed limit only |
| battery current/voltage | NEC `0x41`/`0x42` | ~600 ms round-robin | slow supervision only |
| battery temp, motor temp | ADC | ~100 ms | slow clamps |

## 4. Why a PID oscillates here

- **P-only on an integrator plant is unconditionally stable.** `L(s) = Kp·k/(J s)`
  gives a first-order closed loop: monotonic, no overshoot, for any `Kp > 0`.
  Its only flaw is load-dependent droop, `e_ss = T_load/(Kp·k)`.
- **Adding an integral term makes the loop type-2** — two integrators, −180° phase at
  low frequency, phase margin recoverable only with a well-placed zero. This is the
  classic source of the oscillation we want to avoid. The temptation to add I comes
  from wanting to kill the droop.
- **Derivative is worse.** The rpm estimate is quantized edge timing on a signal that
  also reflects the rider's legs; differentiating it injects noise into the current
  command. If lead is wanted, put it on the **setpoint** (smooth) never on the
  feedback.
- **Never put the 600 ms current measurement in the critical path.** A loop closed on
  `0x41` cannot exceed ~0.2 Hz. Battery current is for slow supervision only.

Rules that follow:

1. Proportional gain only, in the error path. No I, no D on feedback.
2. Exactly **one** speed loop in the system. Keep `0x63` at 250 (the v6 code already
   does) — two cascaded speed loops on the same variable is a guaranteed oscillator.
3. Each loop at least 5–10× slower than the loop inside it.

## 5. Architecture

### 5.1 Droop without an integrator: disturbance observer / feedforward

Estimate the load torque from the current we *measured* and the acceleration we
*observed*, then feed it forward:

```
T_load_hat ← LPF( k·u_meas − J·dω/dt )          // ~0.3 Hz, slow and safe
u_cmd      =  clamp( (T_load_hat + Kp·(ω* − ω)) / k , 0, u_max )
```

This gives zero steady-state error (hills stop sagging cadence) **without an
integrator in the error path**, so there is nothing to wind up, and nothing to add
phase lag until the loop limit-cycles. Because the observer is driven by a measured
disturbance rather than by the error, it can be made arbitrarily slow — the fast P
term handles transients, the slow observer handles steady state. **No anti-windup
logic is required**, which is a large simplification.

### 5.2 Two timescales: keep the human out of the loop

The rider must be the *reference*, not a disturbance inside the fast loop.

- **Fast inner loop (1–2 Hz):** hall rpm → `u`. Fights load, hills, wind.
- **Slow outer loop (0.1–0.2 Hz):** the rider's *intended* cadence → `ω*`. Derived
  from a heavily filtered cadence (τ ≈ 5–10 s) or a per-assist-level nominal.

If `ω*` tracked the rider's *instantaneous* cadence, a cadence sag on a climb would
drag the setpoint down and the motor would do nothing. With the two-timescale split,
a transient sag is an error the fast loop cancels by adding current.

Set `ω* ≈ cadence` (offset 0–5 rpm) and the motor becomes a **load-cancelling stoker
holding the rider's chosen cadence** — torque-sensor feel from a cadence sensor, with
no power cliff at a cadence threshold.

### 5.3 Supervisory state machine

An rpm loop needs a continuous actuator and hysteresis, not an instantaneous on/off.

```
IDLE ──pedal pulses──► ASSIST ──no pulses > stop_delay, or brake/shift──► COAST
  ▲                                                                        │
  └──────────────── u ramps to 0, power stage opens ◄──────────────────────┘
```

Only `ASSIST` runs the loop. Keep `motor_disable()` (P2.0) as a **fault contactor**,
not as a control element. The existing bug pattern — `min_current = 0` → current
reaches 0 → power stage off → 3 s ramp back up — is exactly the limit cycle to remove.

### 5.4 Loop budget

| Stage | Rate | Bandwidth |
|---|---|---|
| NEC current loop | unknown (fast) | — |
| `u` update (0x64) | 32 ms min spacing → 31 Hz | — |
| hall rpm measurement | 3.4–50 ms (12 edges or 50 ms window) | 20–250 Hz |
| fast P loop | 100–200 Hz | **1–2 Hz** |
| disturbance observer | ~50 ms | 0.3 Hz |
| current/thermal/LVC/speed-limit clamps | 200–600 ms | 0.1 Hz |

### 5.5 As built

**Status: parked, not in the current tree.** The firmware was reverted to the
config v5 base (`10cadba`) plus hall rpm telemetry only. A complete, simulated
implementation of the loop exists on the branch `backup/v6-and-speed-loop`
(commit `11f8d46`); the table below describes it. See "Porting it forward".

| Piece | As built |
|---|---|
| Setpoint | the assist level's **Taper Start** cadence, not the measured cadence |
| Feedback | hall motor rpm (same shaft as the chainring) |
| Controller | proportional (2.0 %/rpm) plus a slow bounded trim (1.0 %/rpm/s) |
| Anti-windup | conditional integration against own ceiling, external limiters and zero |
| Actuator | slew limited 200 %/s up, 400 %/s down, 1 % quantization |
| Deadband | 3.0 rpm |
| Backstop | NEC motor speed limiter programmed to setpoint + 20 rpm |
| Fault | stall / lost hall feedback latches the loop off until pedalling stops |

Measured against a simulated drivetrain and rider, it held the setpoint on a
70 Nm hill step with no oscillation at the design gain, followed a sprint up and
settled back, and reduced the pull-away overshoot from 28 % to 10 % once the
anti-windup was added.

**Porting it forward to v5.** The only v6 dependency was the setpoint source: the
implementation read `assist_level_data.level.taper_start_cadence_rpm`. In v5 the
equivalent field is `max_cadence_percent` (a percentage), so the setpoint becomes
`max_cadence_percent * MAX_CADENCE_RPM_X10 / 100` — which is also exactly what
`target_cadence` (the NEC `0x63` command) is set from in v5. Everything else in
the loop is config-version independent. Note v5 already has a global keep-current
feature (`pas_keep_current_percent` / `pas_keep_current_cadence_rpm`) that the loop
would replace for cadence based levels.

Note that the slow trim is a departure from §5.1: the measured-disturbance
observer needs `k` and `J` identified from step tests, and a trim with a time
constant ~10x slower than the proportional loop achieves the same steady state
result now. It is the piece to replace once `k/J` are known.

**Setpoint subtlety that the simulation caught.** The crank is locked to the
chainring by a sprag clutch, so while the rider drives, the measured cadence *is*
the controlled variable. Deriving the setpoint from it (an early version of this
design did) makes the loop chase its own tail: the setpoint decays with the load
and the assist bleeds away on a long climb. Then, once a sprint had raised the
setpoint, it could never come back down, because the motor was holding the cadence
up and the setpoint kept following it. The setpoint must therefore be an
*independent* configured value, with the rider only able to raise it while the
motor is not itself doing the work (`RATCHET_MAX_CURRENT`).

Two more things only the simulation showed: without anti-windup the trim fills up
while the actuator is saturated during the pull away from a standstill and the loop
overshoots by ~28 % (now ~10 %); and a 2.0 rpm deadband leaves a residual
limit cycle that disappears by 3.0 rpm.

### 5.6 Anti-oscillation measures

- P-only; no I, no D in the error path.
- Slew-limit `u` (≈ ±200 %/s up, −400 %/s down) to avoid step-exciting the NEC and to
  protect the gearbox.
- Slew-limit and low-pass `ω*` so a rider cadence step never steps the loop.
- Reject outliers in the hall period (median-of-3 or implausible-interval rejection);
  one bad edge otherwise produces a huge false rpm spike.
- Deadband of 2–3 rpm on the error, to stay above the 1 % current quantization
  limit-cycle amplitude (`≈ 1 %/Kp`).
- Gain scheduling / **oscillation supervisor**: count sign changes of `dω/dt` per
  second; if it exceeds ~6, halve `Kp`, log an event, and re-raise slowly. This makes
  the loop robust against unmodelled gain changes (gear shifts, different riders).
- Gate the observer during shift/brake interrupts, where the load model is invalid.

## 6. Calibration constants

Hall edges give **electrical** speed with no assumptions. Converting to output-shaft
(crank-equivalent) rpm needs two constants, both currently guesses:

```c
MOTOR_POLE_PAIRS        // rotor pole PAIRS, not poles (8 magnets => 4 pairs)
MOTOR_GEAR_RATIO_X10    // rotor : output reduction, x10 (guess: 219 → 21.9:1)
```

Note the units trap: a BLDC is specified by its pole *pairs*. A rotor with 8
magnets has 4 pole pairs, and the electrical frequency is `pole_pairs × mechanical
rpm`, so passing 8 here would report double the true speed.

```
output_rpm = elec_rpm / (MOTOR_POLE_PAIRS × MOTOR_GEAR_RATIO)      [ratio not x10]
elec_rpm   = 10 / T_edge_seconds        (6 edges per electrical revolution)
```

**Calibration procedure:** pedal with the motor barely assisting so the crank and
chainring are locked by the sprag clutch, then compare reported motor rpm with PAS
cadence. If the two differ by a constant factor, that factor is the error in
`MOTOR_POLE_PAIRS × MOTOR_GEAR_RATIO`; adjust one constant and reflash once.

For loop *stability* the absolute value does not matter — only consistency — so
tuning can proceed before calibration is perfect.

## 7. Instrumentation (prerequisite for tuning)

Telemetry currently rides the display link at **1200 baud** (120 bytes/s). The 0xEC
frame is 8 bytes every 500 ms (≈13 % duty). That is enough to *verify* the hall
reading but **nowhere near enough to tune a 1–2 Hz loop** (a 50 Hz log at 8–20 bytes
per sample needs 400–1000 bytes/s).

Options, in order of preference:

1. **Burst capture then dump.** Record N samples into the 3840 bytes of XRAM at
   100–200 Hz on a trigger, then stream them out slowly over the existing link.
   10 s at 100 Hz × 3 bytes = 3 kB — fits. No hardware change; gives a full step
   response and enough data to fit `k/J`.
2. **Spare UART.** The STC15W4K has UART3/UART4 and the schematic notes an unused
   UART brought out to pins with a "debug terminal" at 9600 baud (P0.0–P0.3). At
   9600 baud a 20-byte frame gets ~48 Hz. Needs a probe on those pads.
3. Temporarily raise the external UART baud for bench work (display stops working).

Recommended: option 1, since it needs no hardware access and directly answers
"how does the loop respond to a step".

Also worth doing before anything else: **probe the NEC for an undocumented speed
read opcode.** The wiki's opcode table is incomplete by the author's admission and
the NEC demonstrably computes rotor speed internally. Sweep unclaimed opcodes
(`0x43`–`0x5F`, `0x6F`–`0x7F`) at 4800 baud with the wheel off the ground. If one
answers with speed, sensors become unnecessary.

## 8. Implementation phases

| Phase | Work | Risk |
|---|---|---|
| 0 | NEC opcode probe | low (unknown opcodes may set parameters — wheel off ground) |
| 1 | **Hall rpm measurement + telemetry** (done) | low |
| 2 | **Verify rpm vs cadence; confirm pole pairs and reduction** (done) | low |
| 3 | High-rate instrumentation (burst capture) | low |
| 4 | Open-loop step tests → estimate `k/J`, delay, quantization | low |
| 5 | **P-only loop, configured setpoint, slew limits, deadband, fault latch** (done) | **medium** |
| 6 | Field tuning of `Kp`, deadband and slew rates | medium |
| 7 | Disturbance observer / measured-current feedforward to replace the trim | medium |
| 8 | Oscillation supervisor, gain scheduling | medium |

Phase 3 has become the blocker for phase 6: the 1200 baud display link carries the
0xEC frame at 2 Hz, which is fine for watching whether the loop holds cadence but
useless for tuning gain. The existing telemetry values are otherwise the right ones
(`targetCurrentPercent` is now the loop output and `motorRpm` its feedback).

## 9. Safety

- Hard clamp on `ω*` and on `u`; independent of loop state.
- Stall / lost hall feedback: the motor shaft is backdriven by the pedals, so with
  the rider pedalling it can never legitimately sit at zero rpm. If it does while
  the loop is pushing current for `SPEED_LOOP_STALL_TIMEOUT_MS`, the loop latches
  off until pedalling stops. This matters because the loop would otherwise drive
  full current blind.
- Keep the brake input's hardware path to the NEC.
- Keep `motor_disable()` (P2.0) as the fault contactor. Note the loop now keeps the
  power stage enabled whenever it is engaged, even at zero current, to stop the
  stage toggling and restarting the current ramp.
- Bench first: wheel off the ground, then a trainer with a controllable load.
- Keep a known-good hex and the reflash procedure ready.

## 10. Open questions

- NEC inner current-loop bandwidth and its internal ramp behaviour (phase 4).
- The loop currently reuses the per-level **Taper Start** field as its setpoint, so
  the tool's label is misleading in loop mode. It should either be renamed to
  something like "Target Cadence" or become a field of its own, which means a
  config version bump. `Max Cadence` / `0x63` is also still missing (removed in v6).
- Whether the sprint ratchet is wanted at all: setting
  `SPEED_LOOP_TARGET_FOLLOW_UP_RPM_PER_S` to 0 gives a pure fixed setpoint, which
  is simpler and has no positive feedback path. Worth A/B testing on the bike.
- Whether the slew rates should follow the configured `current_ramp_amps_s` rather
  than being independent constants, so the existing "Current Ramp" knob keeps
  meaning what it says.
- Whether `0x63` should be a backstop at all (currently set to setpoint + margin),
  given its percent-to-rpm mapping shifts with battery voltage.
