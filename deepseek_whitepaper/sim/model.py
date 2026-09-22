"""
Plant simulation: pedelec with a crank -> driveline one-way clutch (freewheel).

Everything is expressed in CRANK-REFERENCED SI units:

    w_c, w_m : rad/s   crank speed and *motor cadence expressed as the
                       equivalent crank speed* (i.e. already divided by the
                       crank:driveline gear ratio).  This is the only way the
                       statement "rider cadence <= motor cadence" is
                       dimensionally meaningful, so we adopt it.
    tau_*    : Nm      crank-referenced torque
    rho      : m/rad   metres travelled per crank radian  (v = rho * w)
    J_*      : kg m^2  crank-referenced inertia

Clutch (complementarity, standard bicycle freewheel):
    w_c <= w_m  always.
    w_c <  w_m  -> clutch open, no torque path, tau_r_transmitted = 0
    w_c == w_m  -> clutch locked, torque transmitted IFF it is non-negative
                   in the crank->driveline direction.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Callable, List, Optional, Tuple

import numpy as np

TWO_PI = 2.0 * np.pi
RAD_S_TO_RPM = 60.0 / TWO_PI
RPM_TO_RAD_S = TWO_PI / 60.0


# --------------------------------------------------------------------------- #
#  Parameters
# --------------------------------------------------------------------------- #
@dataclass
class Params:
    # ---- vehicle -----------------------------------------------------------
    m: float = 95.0             # rider + bike [kg]
    g: float = 9.81
    Crr: float = 0.008          # rolling resistance coefficient
    CdA: float = 0.50           # drag area [m^2]
    rho_air: float = 1.20       # air density [kg/m^3]
    J_rotor: float = 0.05       # motor/driveline rotating inertia [kg m^2]

    # ---- motor + controller ------------------------------------------------
    Kt: float = 1.00            # crank-referenced torque constant [Nm/A]
    I_max: float = 40.0         # current limit [A]
    I_min: float = 0.0          # assist-only controller (no regen) [A]
    tau_e: float = 0.015        # electrical/current-loop time constant [s]
    Kp_i: float = 60.0          # cadence-loop proportional gain [A/(rad/s)]
    Ki_i: float = 120.0         # cadence-loop integral gain  [A/rad]

    # ---- crank -------------------------------------------------------------
    J_c: float = 0.15           # crank + legs, reflected [kg m^2]
    b_c: float = 0.05           # crank bearing/crank friction [Nm/(rad/s)]
    b_d: float = 0.30           # driveline viscous loss  [Nm/(rad/s)]

    # ---- rider -------------------------------------------------------------
    # Cyclists regulate *power / cadence*, not speed, so the rider model is a
    # power feed-forward with a modest speed-error correction on top.
    tau_peak: float = 55.0      # isometric-ish peak crank torque [Nm]
    w_max_rider: float = 21.0   # cadence at which rider torque -> 0 [rad/s]
    a1: float = 0.25            # 1/rev (gravity/dead-spot) ripple
    a2: float = 0.55            # 2/rev (two power strokes) ripple
    Kp_r: float = 25.0          # rider speed-loop P  [Nm per m/s]
    Ki_r: float = 0.0           # rider speed-loop I  [Nm per m] (no wind-up)

    # ---- sensors -----------------------------------------------------------
    cadence_pulses: int = 12    # crank magnets per revolution
    cadence_jitter: float = 0.0015   # pulse-timestamp jitter std [s]
    cadence_timeout: float = 1.20    # no pulse for this long -> cadence 0
    cm_noise_rpm: float = 0.02       # motor cadence noise [rpm rms]
    i_noise: float = 0.15            # current sensor noise [A rms]

    def rho2J(self, rho: float) -> float:
        """Reflected vehicle inertia at the crank for a given gear ratio."""
        return self.m * rho * rho + self.J_rotor


# --------------------------------------------------------------------------- #
#  Scenario
# --------------------------------------------------------------------------- #
@dataclass
class Scenario:
    name: str
    duration: float
    rho: float                       # m per crank radian (gear selection)
    v_star: Callable[[float], float]         # rider's desired speed [m/s]
    w_target: Callable[[float], float]       # motor cadence target [rad/s]
    p_star: Callable[[float], float] = lambda t: 100.0   # rider power intent [W]
    grade: Callable[[float], float] = lambda t: 0.0
    pedaling: Callable[[float], bool] = lambda t: True
    brake: Callable[[float], float] = lambda t: 0.0     # extra drag force [N]
    rider_gain: float = 1.0
    dt: float = 1.0e-3
    notes: str = ""
    # named windows used for reporting: (t0, t1, label)
    windows: List[Tuple[float, float, str]] = field(default_factory=list)


# --------------------------------------------------------------------------- #
#  Simulation
# --------------------------------------------------------------------------- #
def simulate(sc: Scenario, P: Params, seed: int = 0, sens_dt: float = 0.01):
    """Integrate the plant.  Returns (plant_log, sensor_log)."""
    rng = np.random.default_rng(seed)
    dt = sc.dt
    n = int(round(sc.duration / dt)) + 1
    div = max(1, int(round(sens_dt / dt)))
    n_s = n // div + 1

    J_d = P.rho2J(sc.rho)

    # ---- state -------------------------------------------------------------
    w_m = 0.0
    w_c = 0.0
    if isinstance(sc.v_star(0.0), (int, float)):
        w_m = max(0.0, sc.v_star(0.0) / sc.rho)
    w_c = w_m
    locked = True
    phi_c = 0.0
    i_m = 0.0
    int_m = 0.0
    int_r = 0.0

    # ---- plant log ---------------------------------------------------------
    keys = ["t", "v", "grade", "w_c", "w_m", "tau_r", "tau_load", "tau_m",
            "i_m", "i_cmd", "w_target", "u", "ped", "locked", "tau_c"]
    L = {k: np.zeros(n, dtype=np.float64) for k in keys}

    # ---- sensor log --------------------------------------------------------
    S = {k: np.zeros(n_s, dtype=np.float64)
         for k in ["t", "c_r_rpm", "c_m_rpm", "i_m", "i_cmd", "c_m_target_rpm",
                   "locked", "tau_r_true", "tau_m_true", "tau_load_true",
                   "w_m", "w_c", "v", "grade", "ped"]}
    s_i = 0

    # crank cadence sensor state
    n_pulse_prev = 0
    t_last_pulse = 0.0
    c_r_meas = 0.0
    n_pulse_per_rev = P.cadence_pulses
    pulse_ang = TWO_PI / n_pulse_per_rev

    for k in range(n):
        t = k * dt

        v = w_m * sc.rho
        grade = sc.grade(t)
        F = (P.m * P.g * (P.Crr * np.cos(grade) + np.sin(grade))
             + 0.5 * P.rho_air * P.CdA * v * v
             + sc.brake(t))
        tau_load = sc.rho * F

        # ---------------- rider ----------------
        ped = bool(sc.pedaling(t))
        wmax = P.tau_peak * max(0.0, 1.0 - w_c / P.w_max_rider)
        stroke = 1.0 + P.a1 * np.sin(phi_c + 0.7) + P.a2 * np.sin(2.0 * phi_c + 0.2)
        if ped:
            e = sc.v_star(t) - v
            tau_ff = sc.p_star(t) / max(w_c, 1.0)          # power intent
            cmd = tau_ff + sc.rider_gain * (P.Kp_r * e + int_r)
            tau_cmd = min(wmax, max(0.0, cmd))
            if tau_cmd == cmd:                              # conditional integration
                int_r += P.Ki_r * e * dt
            u = tau_cmd / wmax if wmax > 1e-6 else 0.0
        else:
            tau_cmd = 0.0
            u = 0.0
            int_r = 0.0
        tau_r = tau_cmd * stroke

        # ---------------- motor controller ----------------
        e_m = sc.w_target(t) - w_m
        i_cmd_raw = P.Kp_i * e_m + int_m
        i_cmd = min(P.I_max, max(P.I_min, i_cmd_raw))
        if i_cmd == i_cmd_raw:                 # conditional integration
            int_m += P.Ki_i * e_m * dt
        i_m += dt / P.tau_e * (i_cmd - i_m)
        tau_m = P.Kt * i_m

        # ---------------- clutch + integration ----------------
        tau_c = 0.0
        if locked:
            Jt = P.J_c + J_d
            a = (tau_r + tau_m - tau_load - (P.b_c + P.b_d) * w_m) / Jt
            tau_c = tau_r - P.b_c * w_m - P.J_c * a
            if tau_c >= 0.0:
                w_m = w_m + a * dt
                w_c = w_m
            else:
                locked = False
                tau_c = 0.0
        if not locked:
            a_c = (tau_r - P.b_c * w_c) / P.J_c
            a_d = (tau_m - tau_load - P.b_d * w_m) / J_d
            w_c_n = w_c + a_c * dt
            w_m_n = w_m + a_d * dt
            if w_c_n >= w_m_n and w_c > 0.05:
                w = (P.J_c * w_c + J_d * w_m) / (P.J_c + J_d)
                a = (tau_r + tau_m - tau_load - (P.b_c + P.b_d) * w) / (P.J_c + J_d)
                tau_c = tau_r - P.b_c * w - P.J_c * a
                if tau_c >= 0.0:
                    w_m = max(0.0, w + a * dt)
                    w_c = w_m
                    locked = True
                else:
                    w_c, w_m = max(0.0, w_c_n), max(0.0, w_m_n)
                    tau_c = 0.0
            else:
                w_c, w_m = max(0.0, w_c_n), max(0.0, w_m_n)
                tau_c = 0.0

        phi_c += w_c * dt

        # ---------------- logging ----------------
        L["t"][k] = t; L["v"][k] = v; L["grade"][k] = grade
        L["w_c"][k] = w_c; L["w_m"][k] = w_m
        L["tau_r"][k] = tau_r if locked else 0.0
        L["tau_load"][k] = tau_load; L["tau_m"][k] = tau_m
        L["i_m"][k] = i_m; L["i_cmd"][k] = i_cmd
        L["w_target"][k] = sc.w_target(t); L["u"][k] = u
        L["ped"][k] = 1.0 * ped; L["locked"][k] = 1.0 * locked
        L["tau_c"][k] = tau_c

        # ---------------- crank cadence sensor ----------------
        n_pulse = int(phi_c / pulse_ang)
        if n_pulse > n_pulse_prev:
            dtp = (t - t_last_pulse) / (n_pulse - n_pulse_prev)
            t_last_pulse = t
            n_pulse_prev = n_pulse
            dtp += rng.normal(0.0, P.cadence_jitter)
            if dtp > 1e-4:
                c_r_meas = (pulse_ang / dtp) * RAD_S_TO_RPM
        if (t - t_last_pulse) > P.cadence_timeout:
            c_r_meas = 0.0

        if k % div == 0 and s_i < n_s:
            S["t"][s_i] = t
            S["c_r_rpm"][s_i] = c_r_meas
            S["c_m_rpm"][s_i] = w_m * RAD_S_TO_RPM + rng.normal(0.0, P.cm_noise_rpm)
            S["i_m"][s_i] = i_m + rng.normal(0.0, P.i_noise)
            S["i_cmd"][s_i] = i_cmd
            S["c_m_target_rpm"][s_i] = sc.w_target(t) * RAD_S_TO_RPM
            S["locked"][s_i] = 1.0 * locked
            S["tau_r_true"][s_i] = tau_r if locked else 0.0
            S["tau_m_true"][s_i] = tau_m
            S["tau_load_true"][s_i] = tau_load
            S["w_m"][s_i] = w_m
            S["w_c"][s_i] = w_c
            S["v"][s_i] = v
            S["grade"][s_i] = grade
            S["ped"][s_i] = 1.0 * ped
            s_i += 1

    for d in (L, S):
        for kk in d:
            d[kk] = d[kk][: (n if d is L else s_i)]
    return L, S, J_d
