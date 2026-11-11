#!/usr/bin/python3
"""
Plots a motor calibration written by Scripts/motor_calibration.py.

Reads Server/motor_calibration.json (or the file given) and draws, in one
figure:

  * speed lines        w(u) = k*u + c per slot and direction, drawn from the
                       dead band u0 up to the highest duty each was measured
                       at, with the fit range and the encoder ceiling marked
  * correction map     u_target = a*u_ref + b against the identity
  * raw mismatch       (w_t(u) - w_ref(u)) / w_ref(u): how far each motor is
                       from the reference at the same duty, uncorrected
  * dead band          u0 = -c/k, the duty below which the motor does not turn
  * verification       measured turn imbalance, raw vs corrected, per level
  * summary            run metadata, fit quality, repeatability

USAGE
-----
    python Scripts/plot_motor_calibration.py
    python Scripts/plot_motor_calibration.py path/to/motor_calibration.json
    python Scripts/plot_motor_calibration.py --save Scripts/data/motor_cal.png
"""
from __future__ import annotations

import argparse
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
DEFAULT_PATH = os.path.join(REPO, "Server", "motor_calibration.json")

SLOTS = ("m1", "m2", "m3", "m4")
DIRECTIONS = ("fwd", "rev")
DIR_STYLE = {"fwd": "-", "rev": "--"}

# Same look as the Scripts/tuning.py dashboard.
SURFACE, INK, INK_2, MUTED = "#fcfcfb", "#0b0b0b", "#52514e", "#898781"
GRID, AXIS = "#e1e0d9", "#c3c2b7"
SERIES = ("#2a78d6", "#eb6834", "#1baf7a", "#eda100")
THEME = {
    "figure.facecolor": SURFACE, "axes.facecolor": SURFACE,
    "savefig.facecolor": SURFACE,
    "axes.edgecolor": AXIS, "axes.linewidth": 0.8,
    "axes.spines.top": False, "axes.spines.right": False,
    "axes.grid": True, "grid.color": GRID, "grid.linewidth": 0.6,
    "grid.linestyle": "-", "axes.axisbelow": True,
    "axes.titlelocation": "left", "axes.titlesize": 10,
    "axes.titlecolor": INK, "axes.labelcolor": INK_2, "text.color": INK,
    "xtick.color": AXIS, "ytick.color": AXIS,
    "xtick.labelcolor": MUTED, "ytick.labelcolor": MUTED,
    "font.family": "sans-serif",
    "font.sans-serif": ["Segoe UI", "DejaVu Sans", "Arial"],
    "font.size": 9, "legend.frameon": False, "legend.fontsize": 8,
}


def load(path: str) -> dict:
    with open(path, encoding="utf-8") as f:
        return json.load(f)


def slot_label(cal: dict, slot: str) -> str:
    enc = cal.get("slots", {}).get(slot)
    return f"{slot} ({enc})" if enc else slot


def speed(fit: dict, u) -> np.ndarray:
    return fit["k"] * np.asarray(u, dtype=float) + fit["c"]


def shade_fit_range(ax, cal: dict) -> None:
    lo, hi = cal.get("fit_range", (None, None))
    if lo is not None:
        ax.axvspan(lo, hi, color=GRID, alpha=0.45, lw=0, zorder=0)


def plot_speed_lines(ax, cal: dict) -> None:
    curves = cal["curves"]
    for i, slot in enumerate(SLOTS):
        for d in DIRECTIONS:
            fit = curves.get(slot, {}).get(d)
            if not fit:
                continue
            u = np.linspace(fit["u0"], fit["top"], 50)
            ax.plot(u, speed(fit, u), DIR_STYLE[d], color=SERIES[i], lw=1.6,
                    label=f"{slot_label(cal, slot)} {d}")
    shade_fit_range(ax, cal)
    if "max_rps" in cal:
        ax.axhline(cal["max_rps"], color=MUTED, lw=0.8, ls=":")
        ax.text(ax.get_xlim()[0], cal["max_rps"], " encoder ceiling",
                va="bottom", ha="left", color=MUTED, fontsize=8)
    ax.set_title("Wheel speed vs duty  (shaded: fit range)")
    ax.set_xlabel("|duty|")
    ax.set_ylabel("wheel rev/s")
    ax.set_ylim(bottom=0)
    ax.legend(ncol=2, loc="lower right")


def plot_correction_map(ax, cal: dict) -> None:
    corr = cal["correction"]
    lo, hi = cal.get("fit_range", (500, 2000))
    u = np.linspace(min(lo, 500), max(hi, 2000), 50)
    ax.plot(u, u, color=AXIS, lw=0.8, label="identity")
    for i, slot in enumerate(SLOTS):
        for d in DIRECTIONS:
            coef = corr.get(slot, {}).get(d)
            if not coef:
                continue
            ax.plot(u, coef["a"] * u + coef["b"], DIR_STYLE[d],
                    color=SERIES[i], lw=1.4,
                    label=f"{slot} {d}: {coef['a']:.3f}u{coef['b']:+.1f}")
    shade_fit_range(ax, cal)
    ref = cal.get("reference", {})
    ax.set_title(f"Correction map  (reference {ref.get('motor', 'm1')} "
                 f"{ref.get('direction', 'fwd')})")
    ax.set_xlabel("reference |duty|")
    ax.set_ylabel("target |duty|")
    ax.legend(loc="upper left", fontsize=7)


def plot_raw_mismatch(ax, cal: dict) -> None:
    curves = cal["curves"]
    ref = cal.get("reference", {})
    ref_fit = curves[ref.get("motor", "m1")][ref.get("direction", "fwd")]
    lo, hi = cal.get("fit_range", (875, 1625))
    # Start well clear of the reference dead band, where the ratio blows up.
    u_lo = max(ref_fit["u0"] + 150, lo - 250)
    u_hi = max(fit["top"] for by_dir in curves.values()
               for fit in by_dir.values() if fit)
    u = np.linspace(u_lo, u_hi, 80)
    w_ref = speed(ref_fit, u)
    for i, slot in enumerate(SLOTS):
        for d in DIRECTIONS:
            fit = curves.get(slot, {}).get(d)
            if not fit:
                continue
            pct = (speed(fit, u) - w_ref) / w_ref * 100.0
            ax.plot(u, pct, DIR_STYLE[d], color=SERIES[i], lw=1.4,
                    label=f"{slot} {d}")
    ax.axhline(0, color=AXIS, lw=0.8)
    shade_fit_range(ax, cal)
    ax.set_title("Raw speed mismatch vs reference  (model, uncorrected)")
    ax.set_xlabel("|duty|")
    ax.set_ylabel("% of reference speed")
    ax.legend(ncol=2, loc="best", fontsize=7)


def plot_dead_band(ax, cal: dict) -> None:
    curves = cal["curves"]
    x = np.arange(len(SLOTS))
    width = 0.38
    for j, d in enumerate(DIRECTIONS):
        vals = [curves.get(s, {}).get(d, {}).get("u0", np.nan) for s in SLOTS]
        bars = ax.bar(x + (j - 0.5) * width, vals, width, color=SERIES,
                      alpha=1.0 if d == "fwd" else 0.55,
                      edgecolor=SURFACE, label=d)
        for b, v in zip(bars, vals):
            if np.isfinite(v):
                ax.text(b.get_x() + b.get_width() / 2, v, f"{v:.0f}",
                        ha="center", va="bottom", fontsize=7, color=INK_2)
    ax.set_xticks(x, [slot_label(cal, s) for s in SLOTS])
    ax.set_title("Dead band u0 = -c/k  (solid fwd, light rev)")
    ax.set_ylabel("|duty|")
    lo = np.nanmin([curves[s][d]["u0"] for s in SLOTS for d in DIRECTIONS
                    if curves.get(s, {}).get(d)])
    ax.set_ylim(bottom=max(0, lo - 80))


def plot_verification(ax, cal: dict) -> None:
    ver = {k: v for k, v in cal.get("verification", {}).items()
           if k.lstrip("-").isdigit()}
    if not ver:
        ax.set_axis_off()
        ax.text(0.5, 0.5, "no verification data", ha="center", color=MUTED)
        return
    levels = sorted(ver, key=int)
    metrics = (("ccw", SERIES[0]), ("cw", SERIES[1]), ("spread", SERIES[2]))
    x = np.arange(len(levels))
    group = 0.84
    width = group / (2 * len(metrics))
    for m, (name, color) in enumerate(metrics):
        for r, (kind, alpha) in enumerate((("raw", 0.4), ("cor", 1.0))):
            vals = [ver[l].get(f"{name}_{kind}", np.nan) for l in levels]
            off = -group / 2 + (2 * m + r + 0.5) * width
            ax.bar(x + off, vals, width, color=color, alpha=alpha,
                   edgecolor=SURFACE,
                   label=f"{name} {'raw' if kind == 'raw' else 'corrected'}")
    ax.set_xticks(x, [f"duty {l}" for l in levels])
    ax.set_title("Verification: turn imbalance, raw vs corrected")
    ax.set_ylabel("%")
    ax.legend(ncol=3, loc="upper right", fontsize=7)


def plot_summary(ax, cal: dict) -> None:
    ax.set_axis_off()
    rows = [
        f"created      {cal.get('created', '?')}",
        f"battery      {cal.get('battery_v', '?')} V",
        f"fit range    {cal.get('fit_range')}",
        f"max rps      {cal.get('max_rps', '?')}",
        "",
        f"{'slot':<9}{'dir':<5}{'k':>9}{'c':>8}{'u0':>6}{'R²':>7}{'n':>4}"
        f"{'top':>6}",
    ]
    for slot in SLOTS:
        for d in DIRECTIONS:
            f = cal["curves"].get(slot, {}).get(d)
            if not f:
                continue
            rows.append(f"{slot_label(cal, slot):<9}{d:<5}{f['k']:>9.5f}"
                        f"{f['c']:>8.3f}{f['u0']:>6.0f}{f['r2']:>7.4f}"
                        f"{f['n']:>4}{f['top']:>6}")
    rep = cal.get("verification", {}).get("repeatability_pct")
    if rep:
        rows += ["", f"repeatability  mean {rep['mean']}%  "
                     f"worst {rep['worst']}% ({rep['where']})"]
    ax.text(0.0, 1.0, "\n".join(rows), transform=ax.transAxes, va="top",
            ha="left", family="monospace", fontsize=8, color=INK_2)
    ax.set_title("Summary")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("path", nargs="?", default=DEFAULT_PATH,
                    help="calibration JSON (default: Server/motor_calibration.json)")
    ap.add_argument("--save", metavar="PNG",
                    help="write the figure to this file instead of showing it")
    args = ap.parse_args()

    cal = load(args.path)

    import matplotlib
    if args.save:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    plt.rcParams.update(THEME)

    fig, axes = plt.subplots(2, 3, figsize=(17, 9.5), layout="constrained")
    title = f"Motor calibration — {os.path.basename(args.path)}"
    try:
        fig.canvas.manager.set_window_title(title)
    except AttributeError:
        pass
    fig.suptitle(title, x=0.01, ha="left", fontsize=12, fontweight="bold")

    plot_speed_lines(axes[0, 0], cal)
    plot_correction_map(axes[0, 1], cal)
    plot_raw_mismatch(axes[0, 2], cal)
    plot_dead_band(axes[1, 0], cal)
    plot_verification(axes[1, 1], cal)
    plot_summary(axes[1, 2], cal)

    if args.save:
        fig.savefig(args.save, dpi=130)
        print(f"[INFO] saved {args.save}")
    else:
        plt.show()
    return 0


if __name__ == "__main__":
    sys.exit(main())
