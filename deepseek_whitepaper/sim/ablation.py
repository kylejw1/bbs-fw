"""Anchor ablation: what happens if each channel is disabled in turn."""
from __future__ import annotations

import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from sim.model import Params, simulate
from sim.estimator import run_estimator
from sim.scenarios import build
from sim.run import make_est_params, smooth, OUT

SCENARIOS = ["1_steady_flat", "2_crowded_slow", "3_uphill", "5_stop_resume",
             "6_accelerate", "9_climb_bias_drift"]
SHORT = ["1_steady", "2_crowded", "3_uphill", "5_stop", "6_accel", "9_drift"]

CONFIGS = [
    ("full (both anchors)", {}),
    ("no ripple anchor", {"use_ripple_anchor": False}),
    ("no freewheel anchor", {"use_free_anchor": False}),
    ("neither anchor", {"use_ripple_anchor": False, "use_free_anchor": False}),
]


def main():
    P = Params()
    scs = {s.name: s for s in build()}

    cal = scs["0_commissioning"]
    _, Scal, _ = simulate(cal, P)
    rho = float(run_estimator(Scal, make_est_params(P, cal.rho))["rho_hat"][-1])
    print(f"ripple/mean ratio calibrated by the commissioning ride: {rho:.3f}")

    header = "".join(f"{n:>10s}" for n in SHORT)
    print(f"{'configuration':>22s}{header}")
    out = {}
    for label, override in CONFIGS:
        row = []
        for name in SCENARIOS:
            sc = scs[name]
            _, S, _ = simulate(sc, P)
            ep = make_est_params(P, sc.rho)
            ep.rho_hat0 = rho
            for k, v in override.items():
                setattr(ep, k, v)
            E = run_estimator(S, ep)
            true_m = smooth(S["tau_r_true"], 25)
            est_m = smooth(E["tau_r"], 25)
            m = (true_m > 3.0) & (S["t"] > 5.0)
            row.append(float(np.mean(np.abs(est_m[m] - true_m[m]))))
        out[label] = row
        print(f"{label:>22s}" + "".join(f"{v:10.2f}" for v in row))

    with open(os.path.join(OUT, "ablation.json"), "w") as f:
        json.dump(out, f, indent=2)
    print(f"\nwrote {OUT}/ablation.json  (values are MAE in Nm)")


if __name__ == "__main__":
    main()
