/*
 * bbs-fw
 *
 * Released under the GPL License, Version 3
 *
 * Virtual load sensing estimator. See loadsensor.h for the physics, for what is
 * implemented and for why the stroke-ripple channel is not.
 *
 * ---------------------------------------------------------------------------
 * Memory and CPU notes (read this before editing)
 * ---------------------------------------------------------------------------
 * The STC15 target has no spare internal RAM: the baseline firmware fills it
 * completely (bbs-fw.mem: "No spare internal RAM space left"). SDCC puts both
 * non-reentrant locals and the spill slots it invents for deep 32 bit
 * expression trees in internal RAM. A first cut of this module written with
 * int32_t arithmetic cost 33 bytes of internal RAM in spill slots alone and
 * would not link. Hence two hard rules:
 *
 *   1. All state and scratch is explicitly __xdata. External RAM has kilobytes
 *      free and this module runs at 20 Hz, so DPTR indirection is irrelevant.
 *   2. Everything below is 16 bit. There is not one 32 bit multiply or divide,
 *      so the linker needs no spill slots at all. The Q scales are chosen so
 *      that every intermediate stays inside int16; the comments give the worst
 *      case for each. 32 bit is used only for millisecond timestamps, which are
 *      add/subtract only.
 *
 * Unit conventions:
 *
 *   cadence   rpm x10                (matches pas_get_cadence_rpm_x10)
 *   accel     rpm x10 per second
 *   torque    deci-Nm (Nm x 10)      (matches the telemetry encoding)
 *
 * Fixed point, with every coefficient pre-scaled in loadsensor_init():
 *
 *   tau_m  = (k_kt_q4  * percent) >> 4                     worst case  3500
 *   tau_J  = (k_acc_q4 * accel)   >> 4                     worst case  1875
 *   tau_b1 = (8 * w10) >> 8                                worst case    94
 *   tau_b2 = ((((w10+16)>>5)^2 >> 3) * 37) >> 5            worst case   573
 */
#include "loadsensor.h"

// Worst case cadence fed into the load model, rpm x10 (200 rpm). Above this the
// quadratic aero term would overflow the 16 bit intermediate.
#define LOADSENSOR_W10_MAX		2000

// Worst case cadence change per sample, rpm x10. At the 50 ms nominal period
// this is 6000 rpm/s, far beyond anything a bicycle does; it exists only so the
// 16 bit derivative cannot overflow.
#define LOADSENSOR_DW_MAX		300

// ---- run time constants ---------------------------------------------------
static __xdata int16_t k_kt_q4;		// deci-Nm per percent of current, Q4
static __xdata int16_t k_acc_q4;	// deci-Nm per (rpm10/s), Q4

static __xdata bool s_initialised;

// ---- sample timing and derivative state -----------------------------------
static __xdata bool s_have_prev;
static __xdata uint32_t s_prev_ms;
static __xdata int16_t s_w10_prev;
static __xdata int16_t s_accel_lp;

// ---- regime classifier ----------------------------------------------------
static __xdata int16_t s_wr_lp;
static __xdata bool s_locked;
static __xdata uint8_t s_free_ct;
static __xdata uint8_t s_lock_ct;
static __xdata uint8_t s_free_run;		// consecutive samples spent freewheel

// ---- load bias anchor -----------------------------------------------------
static __xdata int16_t s_b0_dnm;
static __xdata bool s_anchored;
static __xdata uint32_t s_anchor_ms;

// ---- outputs --------------------------------------------------------------
static __xdata int16_t s_tau_out_dnm;
static __xdata uint8_t s_flags;

// ---- per sample scratch ---------------------------------------------------
// File scope so the update function needs no 16 bit locals of its own either.
static __xdata int16_t s_w10;
static __xdata int16_t s_dw;
static __xdata int16_t s_accel;
static __xdata int16_t s_tau_m;
static __xdata int16_t s_tau_j;
static __xdata int16_t s_tau_load;
static __xdata int16_t s_tau_r;
static __xdata int16_t s_tmp;
static __xdata int16_t s_b2;		// aero term scratch
static __xdata bool s_freewheel;		// regime result for the sample
static __xdata bool s_seen_crank;		// a real pedal pulse has been observed
static __xdata uint16_t s_dt_ms;
static __xdata uint32_t s_age;



void loadsensor_init(uint8_t max_current_amps)
{
	// tau_m[deci-Nm] = KT_NM_PER_AMP_X10/10 * max_amps * percent/100 * 10
	//                = KT_NM_PER_AMP_X10 * max_amps * percent / 100
	// as a Q4 coefficient: KT_NM_PER_AMP_X10 * max_amps * 16 / 100.
	// (55 * 25 * 16) / 100 = 220, i.e. 13.75 deci-Nm per percent.
	k_kt_q4 = (int16_t)(((int32_t)LOADSENSOR_KT_NM_PER_AMP_X10 *
		(int32_t)max_current_amps * 16) / 100);

	// tau_J[deci-Nm] = J[kg m^2] * (2*pi/60) * accel[rpm10/s]
	//                = (INERTIA_X100/100) * 0.104720 * accel
	// as a Q4 coefficient: INERTIA_X100 * 0.104720 * 16 / 100
	//                    = INERTIA_X100 * 1676 / 100000
	// (3000 * 1676) / 100000 = 50, i.e. 3.125 deci-Nm per unit. 0.5% low.
	k_acc_q4 = (int16_t)(((int32_t)LOADSENSOR_INERTIA_X100 * 1676) / 100000);

	s_have_prev = false;
	s_prev_ms = 0;
	s_w10_prev = 0;
	s_accel_lp = 0;

	s_wr_lp = 0;
	s_locked = true;		// assume the clutch is engaged until proven otherwise
	s_freewheel = false;
	s_seen_crank = false;
	s_free_ct = 0;
	s_lock_ct = 0;
	s_free_run = 0;

	s_b0_dnm = LOADSENSOR_B0_NOMINAL_DNM;
	s_anchored = false;
	s_anchor_ms = 0;

	s_tau_out_dnm = 0;
	s_flags = 0;

	s_initialised = true;
}

static void ls_update_derivative(void)
{
	uint16_t inv_dt;

	// accel = dw * 1000 / dt, evaluated as dw * (1000/dt) so the only division
	// is a 16 bit one and dw can be bounded independently of dt.
	s_dw = s_w10 - s_w10_prev;
	s_w10_prev = s_w10;
	if (s_dw > LOADSENSOR_DW_MAX) s_dw = LOADSENSOR_DW_MAX;
	else if (s_dw < -LOADSENSOR_DW_MAX) s_dw = -LOADSENSOR_DW_MAX;

	inv_dt = (uint16_t)(1000u / s_dt_ms);
	s_accel = (int16_t)(s_dw * (int16_t)inv_dt);
	if (s_accel > LOADSENSOR_MAX_ACCEL_X10) s_accel = LOADSENSOR_MAX_ACCEL_X10;
	else if (s_accel < -LOADSENSOR_MAX_ACCEL_X10) s_accel = -LOADSENSOR_MAX_ACCEL_X10;

	// Division rather than >>: an arithmetic shift floors toward minus infinity,
	// which biases a zero-mean derivative low by up to one LSB and shows up as a
	// constant torque offset. Truncation toward zero is symmetric.
	s_accel_lp += (s_accel - s_accel_lp) / (1 << LOADSENSOR_ACCEL_SHIFT);
}

static void ls_update_regime(int16_t cadence_rpm_x10)
{
	// The one-way clutch guarantees crank <= chainring, so any measured excess
	// is sensor noise. Clamping stops crank sensor jitter from looking like a
	// freewheel.
	s_wr_lp += (cadence_rpm_x10 - s_wr_lp) / 2;
	if (s_wr_lp > s_w10)
	{
		s_wr_lp = s_w10;
	}

	if (s_locked)
	{
		if ((s_w10 - s_wr_lp) > LOADSENSOR_FREE_MARGIN_X10)
		{
			if (s_free_ct < 255) s_free_ct++;
		}
		else
		{
			s_free_ct = 0;
		}

		if (s_free_ct >= LOADSENSOR_FREE_DEBOUNCE)
		{
			s_locked = false;
			s_lock_ct = 0;
		}
	}
	else
	{
		if ((s_w10 - s_wr_lp) < LOADSENSOR_LOCK_MARGIN_X10)
		{
			if (s_lock_ct < 255) s_lock_ct++;
		}
		else
		{
			s_lock_ct = 0;
		}

		if (s_lock_ct >= LOADSENSOR_LOCK_DEBOUNCE)
		{
			s_locked = true;
			s_free_ct = 0;
		}
	}

	s_freewheel = !s_locked;

	// How long the clutch has been open, used to tell a real coast from the
	// start-up window in which the crank sensor has simply not pulsed yet.
	if (s_freewheel)
	{
		if (s_free_run < 255) s_free_run++;
	}
	else
	{
		s_free_run = 0;
	}
}

static void ls_compute_torques(uint8_t target_current_percent)
{
	s_tau_m = (int16_t)((k_kt_q4 * (int16_t)target_current_percent) >> 4);
	s_tau_j = (int16_t)((k_acc_q4 * s_accel_lp) >> 4);

	// b1 term: 8/256 = 0.03125 deci-Nm per rpm10 (0.5% low against 0.031416).
	// Aero term in explicit steps rather than one nested expression: SDCC needs
	// no spill slots this way. q = w10/32 rounded, q*q = w10^2/1024, then
	// 37/256 folds in the 0.13 Nm/(rad/s)^2 coefficient.
	s_tmp = (int16_t)((8 * s_w10) >> 8);
	s_b2 = (int16_t)((s_w10 + 16) >> 5);
	s_b2 = (int16_t)(s_b2 * s_b2);
	s_b2 = (int16_t)((s_b2 >> 3) * 37);
	s_tmp = (int16_t)(s_tmp + (s_b2 >> 5));

	s_tau_load = (int16_t)(s_b0_dnm + s_tmp);
}

static void ls_update_anchor(uint32_t now_ms)
{
	// While the clutch is open the rider transmits nothing, so the balance
	// solves for the load directly. Only a *clean* freewheel is accepted: an
	// unambiguous disengagement margin, a turning motor and a plausible
	// acceleration. The margin is what survives the crank sensor's one-pulse
	// latency when the rider re-engages.
	// The crank sensor reads zero until the first pulse of a revolution, so
	// before pedalling has ever been observed the estimator would otherwise
	// mistake a stationary crank for an open clutch and anchor the load from
	// locked-regime data. That one bad anchor then persists for the rest of the
	// ride, because a rider who never stops pedalling never provides another.
	if (!s_seen_crank ||
		!s_freewheel ||
		s_free_run < LOADSENSOR_ANCHOR_ESTABLISH ||
		s_w10 < LOADSENSOR_MIN_RPM_X10 ||
		(s_w10 - s_wr_lp) < LOADSENSOR_FREE_MARGIN_X10 ||
		s_accel_lp >= LOADSENSOR_MAX_ACCEL_X10 ||
		s_accel_lp <= -LOADSENSOR_MAX_ACCEL_X10)
	{
		return;
	}

	s_tmp = (int16_t)(s_tau_m - s_tau_j - s_tmp);

	if (!s_anchored)
	{
		s_b0_dnm = s_tmp;
		s_anchored = true;
	}
	else
	{
		s_tmp = (int16_t)((s_tmp - s_b0_dnm) / (1 << LOADSENSOR_ANCHOR_SHIFT));

		if (s_tmp > LOADSENSOR_ANCHOR_SLEW)
		{
			s_tmp = LOADSENSOR_ANCHOR_SLEW;
		}
		else if (s_tmp < -LOADSENSOR_ANCHOR_SLEW)
		{
			s_tmp = -LOADSENSOR_ANCHOR_SLEW;
		}

		s_b0_dnm = (int16_t)(s_b0_dnm + s_tmp);
	}

	s_anchor_ms = now_ms;
}

static void ls_update_output(uint32_t now_ms, uint8_t target_current_percent)
{
	uint8_t conf;

	// ---- residual + inertial rider torque ------------------------------
	if (s_freewheel)
	{
		// Nothing can be transmitted through an open clutch.
		s_tau_r = 0;
	}
	else
	{
		s_tau_r = (int16_t)(s_tau_j + s_tau_load - s_tau_m);
		if (s_tau_r < 0)
		{
			s_tau_r = 0;
		}
	}

	// ---- flags and confidence ------------------------------------------
	s_flags = 0;

	if (s_locked)
	{
		s_flags |= LOADSENSOR_FLAG_LOCKED;
	}

	if (s_anchored)
	{
		s_flags |= LOADSENSOR_FLAG_ANCHORED;
		s_age = now_ms - s_anchor_ms;

		if (s_age < LOADSENSOR_CONF_GOOD_MS)
		{
			conf = 3;
		}
		else if (s_age < LOADSENSOR_CONF_FAIR_MS)
		{
			conf = 2;
		}
		else if (s_age < LOADSENSOR_CONF_POOR_MS)
		{
			conf = 1;
		}
		else
		{
			conf = 0;
		}

		if (conf > 0)
		{
			s_flags |= LOADSENSOR_FLAG_FRESH;
		}
	}
	else
	{
		conf = 0;
	}

	// The estimator needs the motor turning: the hall sensors are on the motor
	// rotor, so with the motor off there is no chainring speed at all. Without
	// a fresh anchored load reference the split between rider and grade is
	// unknowable, so report nothing rather than a confident wrong number.
	// The physics only applies while the motor is actually turning: the hall
	// sensors are on the motor rotor, so with the motor off there is no
	// chainring speed at all. Without a coast the load bias is still running on
	// its prior, which the confidence field reports as zero rather than
	// suppressing the reading entirely.
	if (s_w10 < LOADSENSOR_MIN_RPM_X10)
	{
		s_flags &= (uint8_t)~LOADSENSOR_FLAG_VALID;
		s_tau_r = 0;
	}
	else
	{
		s_flags |= LOADSENSOR_FLAG_VALID;
	}

	// ---- motor-effort lower bound --------------------------------------
	// A commanded current at the controller limit means the motor can supply no
	// more than Kt*Imax, which bounds the rider torque from below. Applied as a
	// floor on the output, never fed back into the anchor: a load-model-derived
	// bound fed back into the bias estimate closes a divergent loop.
	if (s_locked && target_current_percent >= LOADSENSOR_SAT_PERCENT &&
		(s_flags & LOADSENSOR_FLAG_VALID))
	{
		s_tmp = (int16_t)(s_tau_j + s_tau_load - ((k_kt_q4 * 100) >> 4));

		if (s_tmp > s_tau_r)
		{
			s_tau_r = (int16_t)(s_tau_r + ((s_tmp - s_tau_r) >> 1));
			s_flags |= LOADSENSOR_FLAG_SATURATED;
		}
	}

	s_flags |= (uint8_t)(conf << LOADSENSOR_CONF_SHIFT);

	// ---- output smoothing ----------------------------------------------
	// Mostly to suppress the quantisation noise of the differentiated hall
	// cadence, which is the weakest input in the whole scheme.
	s_tau_out_dnm = (int16_t)(s_tau_out_dnm +
		((s_tau_r - s_tau_out_dnm) / (1 << LOADSENSOR_TAU_SHIFT)));
}

void loadsensor_process(uint16_t cadence_rpm_x10, uint16_t motor_rpm_x10,
	uint8_t target_current_percent, uint32_t now_ms)
{
	uint32_t elapsed;

	if (!s_initialised)
	{
		return;
	}

	// The crank sensor reads zero until its first pulse completes, which would
	// otherwise look exactly like an open clutch for the first few hundred
	// milliseconds. Seed the smoothing filter from the first real measurement so
	// there is no false ramp, and treat the clutch as engaged until then.
	if (cadence_rpm_x10 > 10 && !s_seen_crank)
	{
		s_seen_crank = true;
		s_wr_lp = (int16_t)cadence_rpm_x10;
	}

	// Motor cadence, bounded so the quadratic load term cannot overflow.
	s_w10 = (motor_rpm_x10 > LOADSENSOR_W10_MAX)
		? LOADSENSOR_W10_MAX : (int16_t)motor_rpm_x10;

	if (!s_have_prev)
	{
		s_prev_ms = now_ms;
		s_w10_prev = s_w10;
		s_have_prev = true;
		return;
	}

	elapsed = now_ms - s_prev_ms;
	if (elapsed < 10)
	{
		return;		// two calls inside the same app tick
	}
	if (elapsed > 500)
	{
		// Resynchronise after a long stall rather than producing a wild
		// derivative from a stale interval.
		elapsed = 500;
	}
	s_dt_ms = (uint16_t)elapsed;
	s_prev_ms = now_ms;

	ls_update_derivative();
	ls_update_regime((int16_t)cadence_rpm_x10);
	ls_compute_torques(target_current_percent);
	ls_update_anchor(now_ms);
	ls_update_output(now_ms, target_current_percent);
}

int16_t loadsensor_get_rider_torque_dnm()
{
	return s_tau_out_dnm;
}

int16_t loadsensor_get_load_bias_dnm()
{
	return s_b0_dnm;
}

uint8_t loadsensor_get_flags()
{
	return s_flags;
}
