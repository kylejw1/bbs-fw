"""The nine operating scenarios used to exercise the estimator.

NOTE: every lambda that needs a scenario constant binds it as a default
argument, because Python closures capture the enclosing *variable*, not its
value at definition time.
"""
from __future__ import annotations

from .model import Scenario

KMH = 1.0 / 3.6


def ramp(t, t0, t1, a, b):
    if t <= t0:
        return a
    if t >= t1:
        return b
    return a + (b - a) * (t - t0) / (t1 - t0)


def _grade8(t):
    """flat -> 5.5 % climb -> -4.5 % descent -> flat"""
    if t < 20.0:
        return 0.0
    if t < 26.0:
        return ramp(t, 20.0, 26.0, 0.0, 0.055)
    if t < 50.0:
        return 0.055
    if t < 58.0:
        return ramp(t, 50.0, 58.0, 0.055, -0.045)
    if t < 72.0:
        return -0.045
    if t < 82.0:
        return ramp(t, 72.0, 82.0, -0.045, 0.0)
    return 0.0


def build():
    S = []

    # 0 ------------------------------------------------------- commissioning
    # A short prescribed calibration ride: coast first (freewheel -> b0 anchor)
    # and then pedal steadily (-> ripple/mean ratio rho calibration).
    S.append(Scenario(
        name="0_commissioning",
        duration=40.0, rho=0.76,
        v_star=lambda t: 5.56,
        w_target=lambda t, r=0.76: 5.56 / r,
        p_star=lambda t: 95.0,
        grade=lambda t: 0.0,
        pedaling=lambda t: t >= 6.0,
        notes="Coast 0-6 s (freewheel anchor), then pedal steadily so the "
              "ripple/mean ratio rho can be calibrated.",
        windows=[(25.0, 40.0, "calibrated")],
    ))

    # 1 ------------------------------------------------------------- steady
    S.append(Scenario(
        name="1_steady_flat",
        duration=45.0, rho=0.76,
        v_star=lambda t: 5.56,
        w_target=lambda t, r=0.76: 5.56 / r,
        p_star=lambda t: 95.0,
        grade=lambda t: 0.0,
        notes="20 km/h on the flat, rider and motor share the load.",
        windows=[(25.0, 45.0, "cruise")],
    ))

    # 2 ------------------------------------------------------------- crowded
    S.append(Scenario(
        name="2_crowded_slow",
        duration=45.0, rho=0.30,               # low gear, trickling along
        v_star=lambda t: 1.80,                 # rider wants ~6.5 km/h
        w_target=lambda t, r=0.30: 1.67 / r,   # assist ceiling 6 km/h
        p_star=lambda t: 20.0,
        grade=lambda t: 0.03,                  # slight rise
        notes="Intentionally slow, low gear, rider pushing into a low assist "
              "ceiling.  Low load, low cadence: tests false-positive load.",
        windows=[(25.0, 45.0, "slow cruise")],
    ))

    # 3 ------------------------------------------------------------- uphill
    S.append(Scenario(
        name="3_uphill",
        duration=70.0, rho=0.76,
        v_star=lambda t: 4.17,
        w_target=lambda t, r=0.76: 4.17 / r,
        p_star=lambda t: 100.0,
        grade=lambda t: ramp(t, 8.0, 16.0, 0.0, 0.070),
        notes="0 -> 7 % climb; motor current saturates, rider takes the rest.",
        windows=[(20.0, 32.0, "early climb"), (55.0, 70.0, "steady climb")],
    ))

    # 4 ------------------------------------------------------------- downhill
    S.append(Scenario(
        name="4_downhill",
        duration=50.0, rho=0.76,
        v_star=lambda t: 5.00,
        w_target=lambda t, r=0.76: 5.56 / r,
        p_star=lambda t: 15.0,
        grade=lambda t: ramp(t, 5.0, 12.0, 0.0, -0.070),
        notes="-7 % descent.  Rider stops contributing; the driveline runs "
              "away above the rider's cadence -> freewheel / calibration.",
        windows=[(30.0, 50.0, "descent")],
    ))

    # 5 ------------------------------------------------------- stop pedalling
    S.append(Scenario(
        name="5_stop_resume",
        duration=50.0, rho=0.76,
        v_star=lambda t: 5.56,
        w_target=lambda t, r=0.76: 5.56 / r,
        p_star=lambda t: 95.0,
        grade=lambda t: 0.0,
        pedaling=lambda t: not (20.0 <= t < 23.5),
        notes="Rider stops pedalling for 3.5 s and resumes at the same "
              "cadence.  Freewheel window is a ground-truth tau_r = 0 sample.",
        windows=[(10.0, 20.0, "before"), (24.5, 35.0, "after resume")],
    ))

    # 6 ---------------------------------------------------------- accelerate
    S.append(Scenario(
        name="6_accelerate",
        duration=45.0, rho=0.76,
        v_star=lambda t: ramp(t, 8.0, 16.0, 12 * KMH, 25 * KMH),
        w_target=lambda t, r=0.76: ramp(t, 8.0, 16.0, 12 * KMH, 25 * KMH) / r,
        p_star=lambda t: 200.0,
        grade=lambda t: 0.0,
        notes="Hard acceleration 12 -> 25 km/h.  J*dw/dt term dominates.",
        windows=[(9.0, 16.0, "accel"), (30.0, 45.0, "settled")],
    ))

    # 7 ------------------------------------------------------------- ghost
    S.append(Scenario(
        name="7_ghost_pedal",
        duration=40.0, rho=0.76,
        # Deliberately identical to scenario 1 in every respect -- same speed,
        # same gear, same cadence target, same grade, same duration -- except
        # that the rider applies ~1.5 Nm instead of ~13 Nm.
        v_star=lambda t: 5.56,
        w_target=lambda t, r=0.76: 5.56 / r,
        p_star=lambda t: 11.0,                 # ~1.5 Nm: legs turning, no pressure
        rider_gain=0.0,
        grade=lambda t: 0.0,
        notes="Matched-cadence control for scenario 1: motor drags the driveline "
              "at exactly the same speed and cadence; the rider turns the cranks "
              "with almost no pressure.",
        windows=[(20.0, 40.0, "ghost")],
    ))

    # 8 ------------------------------------------------------------- mixed
    S.append(Scenario(
        name="8_mixed",
        duration=115.0, rho=0.76,
        v_star=lambda t: (5.0 if t < 20 else (4.0 if t < 50 else
                          (6.5 if t < 72 else 5.56))),
        w_target=lambda t, r=0.76: (5.0 / r if t < 20 else
                                    (4.0 / r if t < 50 else
                                     (6.5 / r if t < 72 else 5.56 / r))),
        p_star=lambda t: (95.0 if t < 20 else (110.0 if t < 50 else
                          (60.0 if t < 72 else 90.0))),
        grade=_grade8,
        pedaling=lambda t: not (88.0 <= t < 92.0),
        notes="Flat -> climb -> descent -> flat with a pedal pause; the "
              "full monty.",
        windows=[(10.0, 20.0, "flat"), (40.0, 50.0, "climb"),
                 (60.0, 72.0, "descent"), (95.0, 115.0, "recovered")],
    ))

    # 9 -------------------------------------------------- long climb, no freewheel
    S.append(Scenario(
        name="9_climb_bias_drift",
        duration=150.0, rho=0.76,
        v_star=lambda t: 4.17,
        w_target=lambda t, r=0.76: 4.17 / r,
        p_star=lambda t: 115.0 if t < 100 else 150.0,
        grade=lambda t: (ramp(t, 8.0, 16.0, 0.0, 0.055) if t < 100 else
                         ramp(t, 100.0, 104.0, 0.055, 0.078)),
        notes="Continuous climb, grade steps up at t=100 s with NO freewheel "
              "event afterwards.  Stress test for the b0 bias observer.",
        windows=[(60.0, 100.0, "before step"), (120.0, 150.0, "after step")],
    ))
    return S
