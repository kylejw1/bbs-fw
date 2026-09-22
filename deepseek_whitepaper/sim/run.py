"""Experiment runner: simulate all scenarios, run the VLSE, report metrics."""
from __future__ import annotations

import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from sim.model import Params, simulate, RAD_S_TO_RPM, RPM_TO_RAD_S, TWO_PI
from sim.estimator import EstParams, run_estimator
from sim.scenarios import build

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

OUT = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "results")
os.makedirs(OUT, exist_ok=True)


def make_est_params(P: Params, rho: float, **over) -> EstParams:
    ep = EstParams(
        Kt=P.Kt,
        J_c=P.J_c,
        J_d=P.rho2J(rho),
        b1=P.b_d,
        b2=0.5 * P.rho_air * P.CdA * rho ** 3,
    )
    for k, v in over.items():
        setattr(ep, k, v)
    return ep


def smooth(x, n=25):
    if n <= 1:
        return x
    k = np.ones(n) / n
    return np.convolve(x, k, mode="same")


def metrics(true, est, mask=None, label=""):
    if mask is None:
        mask = np.ones_like(true, dtype=bool)
    mask = mask & np.isfinite(true) & np.isfinite(est)
    if mask.sum() < 5:
        return dict(label=label, n=int(mask.sum()), rmse=np.nan, mae=np.nan,
                    bias=np.nan, r2=np.nan, mtrue=np.nan, mest=np.nan)
    t, e = true[mask], est[mask]
    err = e - t
    ss_tot = np.sum((t - t.mean()) ** 2)
    r2 = 1.0 - np.sum(err ** 2) / ss_tot if ss_tot > 1e-9 else np.nan
    return dict(label=label, n=int(mask.sum()),
                rmse=float(np.sqrt(np.mean(err ** 2))),
                mae=float(np.mean(np.abs(err))),
                bias=float(np.mean(err)),
                r2=float(r2), mtrue=float(t.mean()), mest=float(e.mean()))


def run_one(sc, P: Params, seed=0, est_over=None, plot=True, rho0=0.45):
    L, S, J_d = simulate(sc, P, seed=seed)
    over = dict(est_over or {})
    over.setdefault("rho_hat0", rho0)
    ep = make_est_params(P, sc.rho, **over)
    E = run_estimator(S, ep)

    t = S["t"]
    true_i = S["tau_r_true"]
    est_i = E["tau_r"]
    true_m = smooth(true_i, 25)
    est_m = smooth(est_i, 25)

    res = {"scenario": sc.name, "metrics": {}, "windows": [], "arrays": (S, E)}
    eng = true_i > 3.0
    res["metrics"]["instantaneous_all"] = metrics(true_i, est_i, label="inst/all")
    res["metrics"]["instantaneous_engaged"] = metrics(true_i, est_i, eng, "inst/engaged")
    res["metrics"]["mean_engaged"] = metrics(true_m, est_m, eng, "mean/engaged")
    res["metrics"]["clutch_class"] = {
        "acc": float(np.mean((E["locked"] > 0.5) == (S["locked"] > 0.5))),
    }
    for (t0, t1, lab) in sc.windows:
        m = (t >= t0) & (t < t1)
        res["windows"].append(metrics(true_m, est_m, m, lab))

    if plot:
        _plot(sc, S, E, true_m, est_m, ep)
    return res


def _plot(sc, S, E, true_m, est_m, ep):
    t = S["t"]
    fig, ax = plt.subplots(4, 1, figsize=(11, 11), sharex=True)

    ax[0].plot(t, S["c_r_rpm"], lw=1.0, label="rider cadence (sensor)")
    ax[0].plot(t, S["c_m_rpm"], lw=1.0, label="motor cadence")
    ax[0].plot(t, S["c_m_target_rpm"], lw=1.0, ls="--", label="target max cadence")
    ax[0].set_ylabel("rpm"); ax[0].legend(fontsize=8, ncol=3)
    ax[0].set_title(f"{sc.name} — {sc.notes}", fontsize=9)

    ax[1].plot(t, S["tau_r_true"], lw=0.8, color="0.6", label="true rider torque")
    ax[1].plot(t, true_m, lw=1.6, color="k", label="true rider torque (0.25 s avg)")
    ax[1].plot(t, est_m, lw=1.6, color="C3", label="ESTIMATED rider torque")
    ax[1].fill_between(t, est_m - 2 * smooth(E["sigma"], 25),
                       est_m + 2 * smooth(E["sigma"], 25), color="C3", alpha=0.15,
                       label="±2σ")
    ax[1].plot(t, S["tau_m_true"], lw=1.0, color="C0", alpha=0.7, label="motor torque")
    ax[1].set_ylabel("Nm at crank"); ax[1].legend(fontsize=7, ncol=2)

    ax[2].plot(t, S["locked"], lw=1.0, color="0.3")
    ax[2].plot(t, E["locked"], lw=1.0, ls="--", color="C2")
    ax[2].set_ylabel("locked\n(1/0)"); ax[2].set_ylim(-0.2, 1.2)
    ax[2].text(0.01, 0.55, "solid=true clutch, dashed=estimated",
               transform=ax[2].transAxes, fontsize=7)

    true_b0 = S["tau_load_true"] + (ep.b1 - ep.b1) * S["w_m"] - ep.b2 * S["w_m"] ** 2
    ax[3].plot(t, E["b0"], lw=1.4, color="C4", label=r"$\hat b_0$ (grade/roll bias)")
    ax[3].plot(t, E["b0_free"], lw=1.0, color="C0", alpha=0.8,
               label=r"$b_0$ freewheel anchor")
    ax[3].plot(t, true_b0, lw=1.0, color="0.4", ls="--", label=r"true $b_0$")
    ax3b = ax[3].twinx()
    ax3b.plot(t, E["ripple_amp"], lw=1.0, color="C1", alpha=0.8,
              label="ripple amplitude")
    ax3b.set_ylabel("ripple amp [Nm]", color="C1")
    ax[3].set_ylabel("Nm"); ax[3].set_xlabel("time [s]")
    ax[3].legend(fontsize=7, loc="upper left")
    ax3b.legend(fontsize=7, loc="upper right")

    fig.tight_layout()
    p = os.path.join(OUT, f"{sc.name}.png")
    fig.savefig(p, dpi=110)
    plt.close(fig)


def main():
    P = Params()
    scs = build()

    # ---- commissioning ride: calibrate the ripple/mean ratio rho ----------
    cal = next(s for s in scs if s.name.startswith("0_"))
    _, Scal, _ = simulate(cal, P)
    Ecal = run_estimator(Scal, make_est_params(P, cal.rho))
    rho_cal = float(Ecal["rho_hat"][-1])
    print(f"[commissioning] rho_hat converged to {rho_cal:.3f} "
          f"({Ecal['rho_hat'][len(Ecal['rho_hat'])//2]:.3f} at midpoint)")

    scs = [s for s in scs if not s.name.startswith("0_")]
    allres = {}
    for sc in scs:
        r = run_one(sc, P, rho0=rho_cal)
        allres[sc.name] = r
        m = r["metrics"]
        print(f"\n=== {sc.name} ===")
        for k, v in m.items():
            if k == "clutch_class":
                print(f"  clutch classification accuracy: {v['acc']*100:.2f} %")
            else:
                print(f"  {v['label']:>15s}  RMSE={v['rmse']:6.2f}  bias={v['bias']:+6.2f}"
                      f"  R2={v['r2']:6.3f}  true_mean={v['mtrue']:6.2f}"
                      f"  est_mean={v['mest']:6.2f}")
        for w in r["windows"]:
            print(f"    window {w['label']:>14s}  true={w['mtrue']:6.2f} "
                  f"est={w['mest']:6.2f}  RMSE={w['rmse']:5.2f}")

    # dump a compact summary
    ser = {}
    for name, r in allres.items():
        ser[name] = {"metrics": r["metrics"], "windows": r["windows"]}
    with open(os.path.join(OUT, "summary.json"), "w") as f:
        json.dump(ser, f, indent=2)
    print(f"\nwrote {OUT}/summary.json and per-scenario figures")


if __name__ == "__main__":
    main()
