"""Summary figures: scenario gallery, true-vs-estimated scatter, error bars."""
from __future__ import annotations

import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from sim.model import Params, simulate
from sim.estimator import run_estimator
from sim.scenarios import build
from sim.run import make_est_params, smooth, OUT

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

def collect():
    P = Params()
    scs = build()

    # calibrate the ripple ratio exactly as the deployment procedure prescribes
    cal = next(s for s in scs if s.name.startswith("0_"))
    _, Scal, _ = simulate(cal, P)
    rho = float(run_estimator(Scal, make_est_params(P, cal.rho))["rho_hat"][-1])
    print(f"[commissioning] rho_hat = {rho:.3f}")

    out = []
    for sc in scs:
        if sc.name.startswith("0_"):
            continue
        L, S, J_d = simulate(sc, P)
        ep = make_est_params(P, sc.rho)
        ep.rho_hat0 = rho
        E = run_estimator(S, ep)
        out.append((sc, S, E))
    return out


def main():
    data = collect()

    # ---------------- 1. gallery ------------------------------------------
    fig, axes = plt.subplots(3, 3, figsize=(16, 11))
    for ax, (sc, S, E) in zip(axes.ravel(), data):
        t = S["t"]
        true_m = smooth(S["tau_r_true"], 25)
        est_m = smooth(E["tau_r"], 25)
        ax.plot(t, true_m, color="k", lw=1.8, label="true rider torque")
        ax.plot(t, est_m, color="C3", lw=1.5, label="VLSE estimate")
        ax.fill_between(t, est_m - 2 * smooth(E["sigma"], 25),
                        est_m + 2 * smooth(E["sigma"], 25),
                        color="C3", alpha=0.15, label=r"$\pm2\sigma$")
        ax2 = ax.twinx()
        ax2.plot(t, S["c_m_rpm"], color="C0", lw=0.8, alpha=0.55)
        ax2.plot(t, S["c_r_rpm"], color="C2", lw=0.8, alpha=0.55)
        ax2.set_ylim(0, 200); ax2.set_yticks([])
        ax.set_ylim(-4, 70)
        ax.set_title(sc.name, fontsize=10)
        ax.grid(alpha=0.25)
        if sc.name == "1_steady_flat":
            ax.legend(fontsize=7, loc="upper right")
    for ax in axes[2]:
        ax.set_xlabel("time [s]")
    for ax in axes[:, 0]:
        ax.set_ylabel("crank torque [Nm]")
    fig.suptitle("VLSE rider-torque estimate vs truth — all scenarios "
                 "(blue/green traces = motor / rider cadence, right axis)",
                 fontsize=11)
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, "fig_gallery.png"), dpi=110)
    plt.close(fig)

    # ---------------- 2. summary ------------------------------------------
    fig, ax = plt.subplots(1, 3, figsize=(16, 4.6))

    names, maes, biases = [], [], []
    all_true, all_est = [], []
    for sc, S, E in data:
        true_m = smooth(S["tau_r_true"], 25)
        est_m = smooth(E["tau_r"], 25)
        m = (true_m > 3.0) & (S["t"] > 5.0)
        if m.sum() < 10:
            continue
        names.append(sc.name.split("_", 1)[1])
        maes.append(np.mean(np.abs(est_m[m] - true_m[m])))
        biases.append(np.mean(est_m[m] - true_m[m]))
        all_true.append(true_m[m]); all_est.append(est_m[m])

    y = np.arange(len(names))
    ax[0].barh(y, maes, color="C3")
    ax[0].set_yticks(y); ax[0].set_yticklabels(names, fontsize=8)
    ax[0].invert_yaxis()
    ax[0].set_xlabel("MAE of rider torque [Nm]")
    ax[0].set_title("(a) accuracy by scenario (engaged samples)")
    for i, (v, b) in enumerate(zip(maes, biases)):
        ax[0].text(v + 0.08, i, f"{v:.1f}  (bias {b:+.1f})", va="center", fontsize=7)
    ax[0].set_xlim(0, max(maes) * 1.55)
    ax[0].grid(alpha=0.25, axis="x")

    T = np.concatenate(all_true); Ee = np.concatenate(all_est)
    ax[1].scatter(T, Ee, s=2, alpha=0.06, color="C0", edgecolors="none")
    lim = [0, 55]
    ax[1].plot(lim, lim, "k--", lw=1)
    ax[1].set_xlim(lim); ax[1].set_ylim(lim)
    ax[1].set_xlabel("true rider torque [Nm]")
    ax[1].set_ylabel("estimated [Nm]")
    r = np.corrcoef(T, Ee)[0, 1]
    ax[1].set_title(f"(b) estimate vs truth  (r = {r:.3f}, n = {len(T)})")
    ax[1].grid(alpha=0.25)

    e = Ee - T
    ax[2].hist(e, bins=np.arange(-25, 25.5, 1.0), color="C3", alpha=0.8)
    ax[2].axvline(0, color="k", lw=1)
    ax[2].axvline(np.mean(e), color="C4", lw=1.5,
                  label=f"mean = {np.mean(e):+.2f} Nm")
    ax[2].set_xlabel("error [Nm]"); ax[2].set_ylabel("samples")
    ax[2].set_title(f"(c) error distribution (MAE = {np.mean(np.abs(e)):.2f} Nm)")
    ax[2].legend(fontsize=8)
    ax[2].grid(alpha=0.25)

    fig.tight_layout()
    fig.savefig(os.path.join(OUT, "fig_summary.png"), dpi=115)
    plt.close(fig)
    print("wrote fig_gallery.png, fig_summary.png")
    print(f"POOLED n={len(T)} MAE={np.mean(np.abs(e)):.3f} bias={np.mean(e):+.3f} "
          f"r={r:.4f} P90|e|={np.percentile(np.abs(e),90):.3f}")
    for n, m_, b in zip(names, maes, biases):
        print(f"  {n:>22s}  MAE={m_:5.2f}  bias={b:+5.2f}")


if __name__ == "__main__":
    main()
