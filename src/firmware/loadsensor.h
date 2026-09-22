/*
 * bbs-fw
 *
 * Released under the GPL License, Version 3
 *
 * Virtual load sensing estimator: estimates the rider's transmitted crank
 * torque from signals the BBSHD already has, so that a torque-sensor-like
 * quantity can be logged over telemetry without any additional hardware.
 *
 * This is a *sensor only*. It never influences motor control.
 *
 * ---------------------------------------------------------------------------
 * Physics
 * ---------------------------------------------------------------------------
 * The BBSHD drivetrain is a freewheel: the motor output shaft drives the
 * chainring directly, and the cranks drive the chainring through a sprag
 * clutch. Therefore
 *
 *     crank_rpm <= chainring_rpm   always
 *
 * and the two are rigidly locked whenever equal. While they are locked the
 * driveline torque balance at the crank is
 *
 *     (Jc + Jd) * dw/dt = tau_m + tau_r - tau_load
 *
 * with tau_r (rider) and tau_load (grade/rolling/aero) both unknown. At steady
 * speed that is one equation in two unknowns, so the rider torque is *not*
 * observable without extra information. This module breaks the degeneracy the
 * only way this hardware allows:
 *
 *   1. FREEWHEEL ANCHOR.  When the clutch is open the rider's transmitted
 *      torque is exactly zero, so the balance yields the load directly:
 *
 *          b0 = tau_m - J*dw/dt - b1*w - b2*w^2
 *
 *      Every coast, descent and pedal pause is a free, exact calibration
 *      sample. This is the anchor the whole estimator rests on.
 *
 *   2. RESIDUAL + INERTIAL.  While locked, the rider torque follows from the
 *      same balance with b0 known:
 *
 *          tau_r = J*dw/dt + b0 + b1*w + b2*w^2 - tau_m
 *
 *   3. MOTOR-EFFORT BOUND.  A commanded current at the controller limit
 *      implies tau_m <= Kt*Imax, hence
 *
 *          tau_r >= J*dw/dt + tau_load_hat - Kt*Imax
 *
 *      which is applied as a one-sided floor on the output.
 *
 * ---------------------------------------------------------------------------
 * Channels deliberately NOT implemented, and why
 * ---------------------------------------------------------------------------
 * The design this is ported from also used an angle-synchronous "stroke
 * ripple" channel: the rider's torque is strongly periodic at pedal frequency,
 * so demodulating the driveline torque balance recovers its amplitude without
 * knowing the grade. That channel is NOT viable on this hardware:
 *
 *   - The rider's AC torque (order 9 Nm at a 20 Nm mean) produces a chainring
 *     acceleration ripple of only ~0.3 rad/s^2 because the reflected vehicle
 *     inertia is large (tens of kg m^2).
 *   - The only chainring speed source is hall_get_motor_rpm_x10(), whose
 *     measurement window closes on a fixed 12-edge count, i.e. a fixed
 *     *angle*. The 100 us tick resolution therefore gives a speed quantisation
 *     that grows linearly with speed: about 0.09 rpm at 50 rpm but 0.8 rpm at
 *     150 rpm. Differentiating that over one sixteenth of a revolution yields
 *     1.6 Nm of torque noise at 50 rpm rising to 44 Nm at 150 rpm, against a
 *     9 Nm signal. Measured SNR: ~5.5 at 50 rpm, ~2.0 at 70 rpm, ~0.7 at 100
 *     and ~0.2 at 150 rpm.
 *   - The commanded current cannot substitute: bbs-fw sets a target current
 *     from pedal cadence, not from a torque error, so it carries no pedal-
 *     frequency ripple. The measured battery current updates only every
 *     ~600 ms, below the pedal frequency, so it aliases.
 *
 * The consequence is that b0 is only observable while the rider is coasting.
 * On a normal ride that happens often (every stop, descent and pedal pause),
 * but a continuous climb with no coasting will let the estimate drift. The
 * telemetry exposes b0 and the anchor age so this can be seen in the field.
 */
#ifndef _LOADSENSOR_H_
#define _LOADSENSOR_H_

#include "intellisense.h"

#include <stdint.h>
#include <stdbool.h>

// ---------------------------------------------------------------------------
// Machine constants. These are crank-referenced and MUST be commissioned
// against logged telemetry; the defaults are estimates for a BBSHD on a
// ~100 kg bike in a mid gear.
// ---------------------------------------------------------------------------

// Motor torque at the crank per amp of controller current, x10 Nm/A.
// tau_m[Nm] = KT_NM_PER_AMP_X10 * max_current_amps * percent / 10000
// A BBSHD at 25 A produces roughly 140 Nm at the crank.
#ifndef LOADSENSOR_KT_NM_PER_AMP_X10
#define LOADSENSOR_KT_NM_PER_AMP_X10				55
#endif

// Reflected vehicle inertia at the crank, x100 kg m^2. Scales with the square
// of the gear ratio, so this is the least stable constant here (roughly 12 in a
// low gear and 100 in a high gear on a 100 kg bike).
#ifndef LOADSENSOR_INERTIA_X100
#define LOADSENSOR_INERTIA_X100				3000
#endif

// Lumped load coefficients:  tau_load = b0 + b1*w + b2*w^2  [Nm, w in rad/s]
// Stored as Q16 / Q8 respectively so that the per-sample integer math keeps
// sub-percent accuracy (see loadsensor.c for the derivations).
#ifndef LOADSENSOR_B1_Q16
#define LOADSENSOR_B1_Q8				8		// 0.30 Nm/(rad/s), folded to Q8
#endif
#ifndef LOADSENSOR_B2_Q8
#define LOADSENSOR_B2_Q8				37		// 0.13 Nm/(rad/s)^2

// Prior for the load bias b0 before any coast has been observed, Nm x10. This
// is just the rolling term (rho * m * g * Crr) for a ~95 kg bike, so that the
// estimate is roughly right on the flat from the first pedal stroke. It is
// replaced outright by the first freewheel anchor and flagged as low
// confidence until then.
#define LOADSENSOR_B0_NOMINAL_DNM		57
#endif

// ---------------------------------------------------------------------------
// Tuning
// ---------------------------------------------------------------------------

// Regime classifier thresholds, in rpm x10. These are hysteresis bands on
// (motor_rpm - crank_rpm); entering freewheel needs the larger margin so that
// crank sensor jitter cannot masquerade as a freewheel, and returning to locked
// needs only a small one.
#define LOADSENSOR_FREE_MARGIN_X10		76		// 0.8 rad/s equivalent
#define LOADSENSOR_LOCK_MARGIN_X10		19		// 0.2 rad/s equivalent

// Consecutive samples required before a regime change is accepted.
#define LOADSENSOR_FREE_DEBOUNCE		4
#define LOADSENSOR_LOCK_DEBOUNCE		2

// Consecutive samples of settled freewheel required before the load anchor is
// allowed to update. This must comfortably outlast start-up, where the crank
// sensor reports zero until its first pulse and the driveline therefore looks
// like an open clutch for a few hundred milliseconds. 15 samples at the 50 ms
// estimator period is 750 ms; a real coast lasts seconds.
#define LOADSENSOR_ANCHOR_ESTABLISH		8

// Minimum motor speed for the freewheel anchor to be trusted, rpm x10.
#define LOADSENSOR_MIN_RPM_X10			20

// Reject an anchor sample whose implied acceleration is implausible, rpm10/s.
#define LOADSENSOR_MAX_ACCEL_X10		600

// Freewheel anchor tracking rate: the anchor moves at most 1/8 of the way to
// each new observation, and no faster than ANCHOR_SLEW deci-Nm per sample.
// This is what stops the crank sensor's one-pulse latency at re-engagement from
// poisoning the anchor.
#define LOADSENSOR_ANCHOR_SHIFT			3
#define LOADSENSOR_ANCHOR_SLEW			40		// 4 Nm per sample

// Derivative low pass, right shift applied per sample at the process rate.
#define LOADSENSOR_ACCEL_SHIFT			3

// Output smoothing, right shift per sample.
#define LOADSENSOR_TAU_SHIFT			2

// Motor-effort floor: threshold as a percentage of the current limit.
#define LOADSENSOR_SAT_PERCENT			97

// Anchor freshness, ms, mapped to the 2-bit confidence field.
#define LOADSENSOR_CONF_GOOD_MS			5000
#define LOADSENSOR_CONF_FAIR_MS			20000
#define LOADSENSOR_CONF_POOR_MS			60000

// ---------------------------------------------------------------------------
// Status flags returned in telemetry
// ---------------------------------------------------------------------------
#define LOADSENSOR_FLAG_LOCKED		0x01	// clutch engaged this sample
#define LOADSENSOR_FLAG_ANCHORED	0x02	// a freewheel anchor has been taken
#define LOADSENSOR_FLAG_FRESH		0x04	// anchor still trusted (not stale)
#define LOADSENSOR_FLAG_SATURATED	0x08	// motor-effort floor applied
#define LOADSENSOR_FLAG_VALID		0x10	// output is meaningful

// Confidence occupies bits 5-6 as a 0..3 value.
#define LOADSENSOR_CONF_SHIFT		5
#define LOADSENSOR_CONF_MASK		0x60


void loadsensor_init(uint8_t max_current_amps);

// Called from app_process() at a fixed rate (see LOADSENSOR_INTERVAL_MS in
// app.c), after the control decisions have been made. Read-only with respect to
// motor control.
void loadsensor_process(uint16_t cadence_rpm_x10, uint16_t motor_rpm_x10,
	uint8_t target_current_percent, uint32_t now_ms);

// Estimated rider torque transmitted through the clutch, Nm x10.
int16_t loadsensor_get_rider_torque_dnm();

// Identified load bias b0 (rolling + grade), Nm x10. Diagnostic.
int16_t loadsensor_get_load_bias_dnm();

// LOADSENSOR_FLAG_* bits plus the confidence field.
uint8_t loadsensor_get_flags();

#endif
