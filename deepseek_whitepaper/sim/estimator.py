"""
Virtual Load Sensing Estimator (VLSE).

Estimates the rider's *transmitted* crank torque tau_r (and hence rider power)
from only:
    rider pedal cadence, motor cadence, motor current, target motor current,
    target max motor cadence.

Why it is hard
--------------
The driveline is a freewheel: the crank can never be faster than the motor,
and when the two are locked they share one speed.  In the locked regime the
driveline torque balance is

    (Jc + Jd) dw/dt = Kt*i_m + tau_r - tau_load - b*w

with BOTH tau_r (rider) and tau_load (grade) unknown.  At constant speed this
is one equation in two unknowns: the rider torque is *unobservable*.

The estimator breaks the degeneracy with four independent channels:

  (1) FREEWHEEL ANCHOR.   When the clutch is open tau_r == 0 *exactly*, so the
      driveline equation yields the load directly.  Every coast / pedal pause
      is a free ground-truth calibration sample.

  (2) STROKE-RIPPLE ANCHOR.  The rider's torque is strongly periodic at the
      pedal frequency (two power strokes per revolution).  Angle-synchronous
      demodulation of the driveline torque balance recovers the *amplitude* of
      the rider's oscillating torque.  Dividing by a per-rider ripple/mean
      ratio rho gives an absolute measurement of the rider's mean torque that
      is completely independent of the grade.  This makes the grade bias b0
      observable even during a climb with no freewheel events, and it is what
      separates a struggling rider from a coasting ghost-pedaller.

  (3) MOTOR-EFFORT CHANNEL.  When the controller is at its current limit and
      the motor cadence is still sagging below its target, the motor is doing
      everything it can, so the rider must be carrying the rest:
          tau_r  >=  tau_load_hat - Kt*I_max.
      This is a one-sided (bounding) observation that uses both the "target
      motor current" and the "target max motor cadence" inputs.

  (4) INERTIAL (ACCELERATION) CHANNEL.  The motor cadence is a high-resolution
      virtual accelerometer.  During transients J*dw/dt is large and gives a
      direct, load-model-independent view of the *change* in rider torque.

State  x = [w_m, tau_r, b0]:  b0 is the lumped grade/rolling bias that is
identified on-line by channels (1) and (2).  Channel (3) acts as a one-sided
floor on the OUTPUT (deliberately not fed back: a load-model-derived bound fed
back into the bias state closes a divergent positive loop).  Channel (4) is
implicit in the EKF process model.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, Optional

import numpy as np

TWO_PI = 2.0 * np.pi


RAD_S_TO_RPM = 60.0 / TWO_PI
RPM_TO_RAD_S = TWO_PI / 60.0


@dataclass
class EstParams:
    # ---- assumed (commissioned) machine constants --------------------------
    Kt: float = 1.00          # crank-referenced torque constant [Nm/A]
    J_c: float = 0.15         # crank + legs
    J_d: float = 55.0         # reflected vehicle inertia at the crank
    b1: float = 0.30          # lumped viscous coefficient [Nm/(rad/s)]
    b2: float = 0.13          # lumped aero coefficient [Nm/(rad/s)^2]
    I_max: float = 40.0       # controller current limit [A] (from the target)
    sat_blend: float = 0.5    # how hard the saturation floor is applied
    # --- ablation switches (all True in normal operation) -------------------
    use_ripple_anchor: bool = True
    use_free_anchor: bool = True
    # ---- noise / tuning ----------------------------------------------------
    sig_w: float = 0.02 / RAD_S_TO_RPM     # motor cadence noise [rad/s]
    sig_tau_free: float = 0.60             # freewheel tau_r == 0 pseudo-meas
    sig_b0_free: float = 2.0               # freewheel b0 anchor
    sig_ripple_scale: float = 0.15         # b0-from-ripple sigma, proportional
    sig_ripple_base: float = 1.0
    rev_warmup: int = 4                    # ignore the first revs after start
    q_w: float = 1.0e-5
    q_tau: float = 2.0e2
    q_b0: float = 4.0
    rho_hat0: float = 0.45                 # nominal ripple/mean ratio
    rho_learn: float = 0.05
    rho_min: float = 0.20       # physiological band for the ripple ratio
    rho_max: float = 0.85       # (peak/mean torque ~1.5-3.2)
    wdot_lp_hz: float = 3.0
    lock_hyst_in: float = 0.20             # rad/s, return-to-locked
    lock_hyst_out: float = 0.80            # rad/s, enter-free
    crank_lp_hz: float = 4.0               # crank cadence smoothing for classification
    min_cadence: float = 1.0               # rad/s -> "not pedalling"
    nbins: int = 24
    anchor_hold: float = 45.0              # s, how long a freewheel anchor
                                           # stays usable for rho calibration


class VLSE:
    def __init__(self, ep: EstParams, dt: float = 0.01):
        self.ep = ep
        self.dt = dt
        self.x = np.array([0.0, 0.0, 0.0])
        self.P = np.diag([1.0e-3, 100.0, 400.0])
        self._inited = False

        # regime
        self.locked = False
        self.free_ct = 0

        # speed derivative (low-pass filtered)
        self.w_prev = 0.0
        self.wdot_f = 0.0
        self.w_r_f = 0.0

        # freewheel bias anchor
        self.b0_free = 0.0
        self._anchor_buf: list = []
        self.seen_crank = False
        self.b0_free_valid = False
        self.t_free_last = -1e9
        self.t_now = 0.0

        # ripple channel
        self.phi = 0.0
        self.rho_hat = ep.rho_hat0
        self.ripple_amp = 0.0
        self.ripple_valid = False
        self.ripple_new = False
        self.rev = 0
        nb = ep.nbins
        self.sw = np.zeros(nb); self.si = np.zeros(nb); self.sc = np.zeros(nb)
        self.n_sum = 0
        self.w_bar_prev: Optional[float] = None
        self.lock_ct = 0
        self._rev_i = 0.0; self._rev_w = 0.0; self._rev_wdot = 0.0

    # ------------------------------------------------------------------ #
    def _finish_rev(self):
        nb = self.ep.nbins
        good = self.sc >= 1
        self.rev += 1
        min_bins = max(6, int(0.6 * nb))
        ok = (good.sum() >= min_bins and self.n_sum > min_bins
              and self.lock_ct / max(self.n_sum, 1) >= 0.85)
        if ok:
            w = np.where(good, self.sw / np.maximum(self.sc, 1), np.nan)
            i = np.where(good, self.si / np.maximum(self.sc, 1), np.nan)
            w_bar = float(np.nanmean(w))
            i_bar = float(np.nanmean(i))
            if (w_bar * RAD_S_TO_RPM >= 20.0) and np.isfinite(w_bar):
                dbin_dt = (TWO_PI / nb) / w_bar
                wdot = (np.roll(w, -1) - np.roll(w, 1)) / (2.0 * dbin_dt)
                Jt = self.ep.J_c + self.ep.J_d
                tau_ac = Jt * wdot - self.ep.Kt * (i - i_bar)
                m = np.isfinite(tau_ac)
                if m.sum() >= min_bins:
                    self.ripple_amp = float(np.sqrt(np.nanmean(tau_ac[m] ** 2)))
                    self.ripple_valid = True
                    self.ripple_new = True
                    T_rev = max(self.n_sum * self.dt, 1e-3)
                    wdot_rev = (0.0 if self.w_bar_prev is None
                                else (w_bar - self.w_bar_prev) / T_rev)
                    self._rev_i = i_bar
                    self._rev_w = w_bar
                    self._rev_wdot = float(np.clip(wdot_rev, -5.0, 5.0))
                    spread = float(np.nanmax(w) - np.nanmin(w))
                    self._rev_steady = bool(
                        abs(wdot_rev) < 0.06 and spread / max(w_bar, 0.1) < 0.25)
                    self.w_bar_prev = w_bar
        self.sw[:] = 0; self.si[:] = 0; self.sc[:] = 0
        self.n_sum = 0
        self.lock_ct = 0

    # ------------------------------------------------------------------ #
    def step(self, c_r_rpm: float, c_m_rpm: float, i_m: float,
             i_cmd: float, c_m_target_rpm: float) -> Dict[str, float]:
        ep = self.ep
        dt = self.dt
        w_r = c_r_rpm * RPM_TO_RAD_S
        w_m_meas = c_m_rpm * RPM_TO_RAD_S
        if c_r_rpm > 1.0:
            self.seen_crank = True
        if not self._inited:
            self.x[0] = w_m_meas
            self.P[0, 0] = ep.sig_w ** 2
            self.w_prev = w_m_meas
            self._inited = True

        # ---------------- 0. filtered derivative of motor cadence --------
        alpha = min(1.0, ep.wdot_lp_hz * 2.0 * np.pi * dt)
        self.wdot_f += alpha * ((w_m_meas - self.w_prev) / dt - self.wdot_f)
        self.w_prev = w_m_meas

        # ---------------- 1. regime classifier (hysteresis) --------------
        # The one-way clutch guarantees w_r <= w_m physically, so any measured
        # excess is sensor noise.  Smooth the crank cadence and clamp it to the
        # motor cadence before comparing: this keeps a jittery cheap PAS from
        # masquerading as a freewheel.
        a_r = min(1.0, ep.crank_lp_hz * 2.0 * np.pi * dt)
        self.w_r_f += a_r * (w_r - self.w_r_f)
        w_r_cmp = min(self.w_r_f, w_m_meas)
        if self.locked:
            free_now = (w_m_meas - w_r_cmp) > ep.lock_hyst_out or w_r_cmp < ep.min_cadence
        else:
            free_now = (w_m_meas - w_r_cmp) > ep.lock_hyst_in or w_r_cmp < ep.min_cadence
        locked = not free_now
        self.locked = locked
        self.free_ct = self.free_ct + 1 if free_now else 0

        # ---------------- 2. freewheel bias anchor ----------------------
        # Only trust a *clean* freewheel: the crank must be unambiguously
        # disengaged (margin large enough to survive the one-pulse latency of
        # the crank sensor) and not being spun up by the rider.
        if (free_now and self.free_ct >= 2 and self.seen_crank
                and w_m_meas > 1.0 and (w_m_meas - w_r_cmp) > 0.8
                and abs(self.wdot_f) < 2.0):
            b0_obs = (ep.Kt * i_m - ep.b1 * w_m_meas
                      - ep.b2 * w_m_meas ** 2 - ep.J_d * self.wdot_f)
            # Median-of-window: rejects the short burst of corrupted samples
            # produced while the one-pulse-latent crank sensor catches up with
            # a re-engagement.
            self._anchor_buf.append(b0_obs)
            if len(self._anchor_buf) > 101:
                self._anchor_buf.pop(0)
            self.b0_free = float(np.median(self._anchor_buf))
            if len(self._anchor_buf) >= 15 and ep.use_free_anchor:
                self.b0_free_valid = True
                self.t_free_last = self.t_now
        else:
            self._anchor_buf.clear()

        # ---------------- 3. EKF predict --------------------------------
        w = self.x[0]
        tau_load = self.x[2] + ep.b1 * w + ep.b2 * w * w
        Jt = ep.J_d + (ep.J_c if locked else 0.0)
        xp = self.x.copy()
        xp[0] = w + dt * (ep.Kt * i_m + (self.x[1] if locked else 0.0)
                          - tau_load) / Jt
        F = np.zeros((3, 3))
        F[0, 0] = 1.0 + dt * (-(ep.b1 + 2.0 * ep.b2 * w)) / Jt
        F[0, 1] = dt * (1.0 / Jt if locked else 0.0)
        F[0, 2] = -dt / Jt
        F[1, 1] = 1.0; F[2, 2] = 1.0
        self.P = F @ self.P @ F.T + np.diag(
            [ep.q_w, ep.q_tau, ep.q_b0]) * dt
        self.x = xp

        # ---------------- 4. measurement updates ------------------------
        self._update(np.array([w_m_meas]), np.array([[1.0, 0.0, 0.0]]),
                     np.array([[ep.sig_w ** 2]]))

        if self.free_ct >= 3:
            self._update(np.array([0.0]), np.array([[0.0, 1.0, 0.0]]),
                         np.array([[ep.sig_tau_free ** 2]]))
            if self.b0_free_valid:
                self._update(np.array([self.b0_free]),
                             np.array([[0.0, 0.0, 1.0]]),
                             np.array([[ep.sig_b0_free ** 2]]))

        sat_bound, sat_active = self._sat_bound(ep, i_cmd, c_m_target_rpm,
                                                c_m_rpm)

        # ripple -> direct observation of the *bias*, once per revolution
        if locked and self.ripple_new:
            self._ripple_update(ep)
        self.ripple_new = False

        self.x[1] = min(max(self.x[1], 0.0), 250.0)
        self.x[2] = min(max(self.x[2], -150.0), 150.0)
        self.P = 0.5 * (self.P + self.P.T)

        # ---------------- 5. stroke-angle accumulator -------------------
        if locked:
            self.phi += w_m_meas * dt
            if self.phi >= TWO_PI:
                self.phi -= TWO_PI
                self._finish_rev()
            nb = ep.nbins
            b = min(nb - 1, int(self.phi / TWO_PI * nb))
            self.sw[b] += w_m_meas
            self.si[b] += i_m
            self.sc[b] += 1
            self.n_sum += 1
            self.lock_ct += 1

        self.t_now += dt
        sig = float(np.sqrt(max(self.P[1, 1], 0.0)))
        tau_out = float(self.x[1])
        if sat_active and sat_bound > tau_out:
            tau_out += ep.sat_blend * (sat_bound - tau_out)
        return {
            "tau_r": tau_out,
            "tau_r_kf": float(self.x[1]),
            "sat_active": 1.0 * sat_active,
            "sat_bound": sat_bound,
            "sigma": sig,
            "b0": float(self.x[2]),
            "b0_free": float(self.b0_free),
            "w_m": float(self.x[0]),
            "locked": 1.0 * locked,
            "rho_hat": self.rho_hat,
            "ripple_amp": self.ripple_amp,
            "ripple_valid": 1.0 * (self.ripple_valid and locked),
            "conf": 1.0 / (1.0 + sig / 5.0),
        }

    # ------------------------------------------------------------------ #
    def _sat_bound(self, ep: EstParams, i_cmd, c_m_target_rpm, c_m_rpm):
        """Motor-effort channel.

        A saturated controller whose cadence is still sagging below its target
        is doing everything it can, so the rider must be carrying at least the
        balance of the load:

            tau_r >= tau_load_hat - Kt*I_max

        This is a *bound*, not a measurement.  It is applied as an output-level
        floor and deliberately NOT fed back into the filter: feeding a
        load-model-derived bound back into the bias state closes a positive
        feedback loop (estimate -> bias -> bound -> estimate) that diverges.
        """
        if not self.locked:
            return 0.0, False
        if abs(i_cmd) < 0.97 * ep.I_max:
            return 0.0, False
        if (c_m_target_rpm - c_m_rpm) <= 1.0:
            return 0.0, False
        w = self.x[0]
        tau_load_hat = self.x[2] + ep.b1 * w + ep.b2 * w * w
        z = float(np.clip(tau_load_hat - ep.Kt * ep.I_max, 0.0, 150.0))
        return z, True

    def _ripple_update(self, ep: EstParams):
        """Turn one valid revolution of stroke-ripple data into information
        about the rider torque and the load bias."""
        if not ep.use_ripple_anchor:
            return
        A = self.ripple_amp
        if not (np.isfinite(A) and 0.0 < A < 60.0 and self.rho_hat > 0.05):
            return
        if self.rev <= ep.rev_warmup:
            return          # warm-up: the first revolutions are not settled
        if not getattr(self, "_rev_steady", False):
            # the angle-synchronous demodulation is only valid at quasi-steady
            # speed; during hard transients the inertial channel carries the
            # information and the slow bias must be left alone.
            return
        Jt = ep.J_c + ep.J_d
        z_tau = A / self.rho_hat                    # absolute rider mean torque
        i_bar, w_bar, wdot_rev = self._rev_i, self._rev_w, self._rev_wdot
        z_b0 = (ep.Kt * i_bar + z_tau - ep.b1 * w_bar
                - ep.b2 * w_bar ** 2 - Jt * wdot_rev)
        sig_b0 = ep.sig_ripple_scale * abs(z_tau) + ep.sig_ripple_base
        if abs(z_b0) < 200.0:
            self._update(np.array([z_b0]), np.array([[0.0, 0.0, 1.0]]),
                         np.array([[sig_b0 ** 2]]))

        # --- calibrate rho against a recent freewheel anchor -----------
        if (self.b0_free_valid
                and (self.t_now - self.t_free_last) < ep.anchor_hold):
            denom = (self.b0_free - ep.Kt * i_bar + ep.b1 * w_bar
                     + ep.b2 * w_bar ** 2 + Jt * wdot_rev)
            if denom > 3.0:
                rho_obs = A / denom
                if ep.rho_min * 0.5 < rho_obs < ep.rho_max * 1.5:
                    self.rho_hat = float(np.clip(
                        (1.0 - ep.rho_learn) * self.rho_hat
                        + ep.rho_learn * rho_obs, ep.rho_min, ep.rho_max))

    # ------------------------------------------------------------------ #
    def _update(self, z, H, R):
        y = z - H @ self.x
        S = H @ self.P @ H.T + R
        K = self.P @ H.T @ np.linalg.inv(S)
        self.x = self.x + (K @ y).ravel()
        I = np.eye(3)
        A = I - K @ H
        self.P = A @ self.P @ A.T + K @ R @ K.T


# --------------------------------------------------------------------------- #
def run_estimator(S, ep: EstParams, dt: float = 0.01):
    est = VLSE(ep, dt=dt)
    n = len(S["t"])
    out = {k: np.zeros(n) for k in
           ["tau_r", "tau_r_kf", "sat_active", "sat_bound", "sigma", "b0",
            "b0_free", "locked", "rho_hat", "ripple_amp", "conf"]}
    for k in range(n):
        r = est.step(S["c_r_rpm"][k], S["c_m_rpm"][k], S["i_m"][k],
                     S["i_cmd"][k], S["c_m_target_rpm"][k])
        for kk in out:
            out[kk][k] = r[kk]
    return out
