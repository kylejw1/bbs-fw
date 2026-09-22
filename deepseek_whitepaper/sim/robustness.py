"""Robustness / sensitivity sweeps for the VLSE.

Perturbs the estimator's assumed machine constants, the sensor quality and the
calibration quality, and reports mean-absolute-error and bias of the (0.25 s
smoothed) rider-torque estimate over the engaged samples of every scenario.
"""
from __future__ import annotations

import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from sim.model import Params, simulate
from sim.estimator import EstParams, run_estimator
from sim.scenarios import build
from sim.run import make_est_params, smooth, OUT

SCEN_KEEP = ["1_steady_flat", "2_crowded_slow", "3_uphill", "5_stop_resume",
             "6_accelerate"]


def commission(P, scs, est_over=None):
    cal = next(s for s in scs if s.name.startswith("0_"))
    _, Scal, _ = simulate(cal, P)
    ep = make_est_params(P, cal.rho, **(est_over or {}))
    E = run_estimator(Scal, ep)
    return float(E["rho_hat"][-1])


def evaluate(P, scs, rho0, est_over=None, seeds=(0, 1, 2), scen_keep=None):
    keep = scen_keep or SCEN_KEEP
    errs, biases, per = [], [], {}
    for sc in scs:
        if sc.name not in keep:
            continue
        e_all = []
        for sd in seeds:
            _, S, _ = simulate(sc, P, seed=sd)
            ep = make_est_params(P, sc.rho, **(est_over or {}))
            ep.rho_hat0 = rho0
            E = run_estimator(S, ep)
            t = S["t"]
            true_m = smooth(S["tau_r_true"], 25)
            est_m = smooth(E["tau_r"], 25)
            m = (true_m > 3.0) & (t > 5.0)
            if m.sum() > 10:
                e_all.append((est_m[m] - true_m[m]))
        if e_all:
            e = np.concatenate(e_all)
            per[sc.name] = (float(np.mean(np.abs(e))), float(np.mean(e)))
            errs.append(np.abs(e)); biases.append(e)
    E = np.concatenate(errs) if errs else np.array([np.nan])
    B = np.concatenate(biases) if biases else np.array([np.nan])
    return float(np.mean(E)), float(np.mean(B)), per


def main():
    P0 = Params()
    scs = build()
    rows = []

    cases = [
        ("nominal",                       {},             {}),
        ("Kt -10%",                       {"Kt": 0.90},   {}),
        ("Kt +10%",                       {"Kt": 1.10},   {}),
        ("J_d -20%",                      {"J_d": 0.80},  {}),
        ("J_d +20%",                      {"J_d": 1.20},  {}),
        ("aero coeff b2 -25%",            {"b2": 0.75},   {}),
        ("aero coeff b2 +25%",            {"b2": 1.25},   {}),
        ("viscous b1 -30%",               {"b1": 0.70},   {}),
        ("viscous b1 +30%",               {"b1": 1.30},   {}),
        ("no commissioning (rho=0.45)",   {},             {"rho_fixed": 0.45}),
        ("no commissioning (rho=0.35)",   {},             {"rho_fixed": 0.35}),
        ("no commissioning (rho=0.55)",   {},             {"rho_fixed": 0.55}),
        ("crank jitter x5",               {},             {"cadence_jitter": 5.0}),
        ("motor cadence noise x10",       {},             {"cm_noise": 10.0}),
        ("current noise x5",              {},             {"i_noise": 5.0}),
        ("all sensors degraded",          {},             {"cadence_jitter": 5.0,
                                                           "cm_noise": 10.0,
                                                           "i_noise": 5.0}),
    ]

    for name, est_mult, misc in cases:
        P = Params()
        # sensor perturbations act on the *plant*
        if "cadence_jitter" in misc:
            P.cadence_jitter *= misc["cadence_jitter"]
        if "cm_noise" in misc:
            P.cm_noise_rpm *= misc["cm_noise"]
        if "i_noise" in misc:
            P.i_noise *= misc["i_noise"]

        est_over = {}
        if "Kt" in est_mult:
            est_over["Kt"] = P0.Kt * est_mult["Kt"]
        if "J_d" in est_mult:
            est_over["J_d"] = None          # handled per-scenario below
        if "b2" in est_mult:
            est_over["b2"] = None
        if "b1" in est_mult:
            est_over["b1"] = P0.b_d * est_mult["b1"]

        # rho: either calibrated (via commissioning) or forced
        if "rho_fixed" in misc:
            rho0 = misc["rho_fixed"]
        else:
            rho0 = commission(P, scs, {})

        # per-scenario J_d / b2 scaling has to be applied inside evaluate
        class _Tweak:
            pass
        keep = SCEN_KEEP
        errs = []
        for sc in scs:
            if sc.name not in keep:
                continue
            for sd in (0, 1):
                _, S, J_d = simulate(sc, P, seed=sd)
                ep = make_est_params(P, sc.rho)
                if "J_d" in est_mult:
                    ep.J_d = J_d * est_mult["J_d"]
                if "b2" in est_mult:
                    ep.b2 = make_est_params(P, sc.rho).b2 * est_mult["b2"]
                for k, v in est_over.items():
                    if v is not None:
                        setattr(ep, k, v)
                ep.rho_hat0 = rho0
                E = run_estimator(S, ep)
                true_m = smooth(S["tau_r_true"], 25)
                est_m = smooth(E["tau_r"], 25)
                m = (true_m > 3.0) & (S["t"] > 5.0)
                if m.sum() > 10:
                    errs.append(est_m[m] - true_m[m])
        e = np.concatenate(errs)
        rows.append((name, float(np.mean(np.abs(e))), float(np.mean(e)),
                     float(np.percentile(np.abs(e), 90)), rho0))
        print(f"{name:>30s}  MAE={rows[-1][1]:6.2f} Nm  bias={rows[-1][2]:+6.2f} "
              f" P90|err|={rows[-1][3]:6.2f}  rho0={rho0:.3f}")

    with open(os.path.join(OUT, "robustness.json"), "w") as f:
        json.dump([dict(case=r[0], mae=r[1], bias=r[2], p90=r[3], rho0=r[4])
                   for r in rows], f, indent=2)
    print(f"\nwrote {OUT}/robustness.json")


if __name__ == "__main__":
    main()
