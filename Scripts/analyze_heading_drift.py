#!/usr/bin/python3
"""
Heading drift analysis for a parked kart, from a Scripts/plot_heading.py CSV.

    python Scripts/plot_heading.py --ip 192.168.0.167 --duration 300 \
        --csv Scripts/data/heading_still.csv
    python Scripts/analyze_heading_drift.py Scripts/data/heading_still.csv

Reports, from the gyro yaw the Pi integrates:
  * the drift: total, and the least-squares slope (= residual bias, deg/s)
  * how that bias moves over the run (slope over sliding windows) -- a bias
    that wanders is what a one-shot start-up calibration cannot fix
  * the Allan deviation of the yaw rate: its minimum is the bias instability
    and its tau is how long to average for the best bias estimate, which sizes
    ZeroRateFilter.bias_tau_s

Then replays the yaw through Server/standstill.py (StandstillDetector +
ZeroRateFilter) with the recorded encoder motion, and once more with synthetic
turns spliced in (wheels counting) to show that real rotation passes untouched.
"""
from __future__ import annotations

import argparse
import csv
import math
import os
import sys
from dataclasses import replace

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
SERVER = os.path.join(REPO, "Server")
# Server/ is not a package -- see the same note in gray_regression.py.
if SERVER not in sys.path:
    sys.path.insert(0, SERVER)

from config import CONFIG, ENCODER_TAGS                         # noqa: E402
from standstill import StandstillDetector, ZeroRateFilter       # noqa: E402
from plot_motor_calibration import (AXIS, SERIES, THEME)        # noqa: E402

WINDOW_S = 30.0       # sliding window for the bias-over-time panel
SUBSTEPS = 5          # replay at 50 Hz like the gyro thread, from 10 Hz data


def load(path: str) -> dict:
    with open(path, newline="", encoding="utf-8") as f:
        rows = list(csv.DictReader(f))
    if not rows:
        raise SystemExit(f"{path}: no samples")
    col = lambda k, fn=float: np.array([fn(r[k]) for r in rows])
    return {"t": col("t") - float(rows[0]["t"]), "yaw": col("yaw_deg"),
            "moved": col("enc_moved", lambda v: int(float(v)))}


def allan(t: np.ndarray, angle: np.ndarray):
    """Overlapping Allan deviation of the RATE, from the integrated angle.
    Resampled onto a uniform grid first (the telemetry is ~10 Hz, jittery)."""
    dt = float(np.median(np.diff(t)))
    grid = np.arange(t[0], t[-1], dt)
    th = np.interp(grid, t, angle)
    n = len(th)
    ms = np.unique(np.logspace(0, np.log10(max(1, n // 3)), 40).astype(int))
    taus, adev = [], []
    for m in ms:
        if n - 2 * m < 2:
            break
        d = th[2 * m:] - 2 * th[m:-m] + th[:-2 * m]
        tau = m * dt
        taus.append(tau)
        adev.append(math.sqrt(np.mean(d * d) / (2 * tau * tau)))
    return np.array(taus), np.array(adev)


def sliding_slope(t, y, window):
    centres, slopes = [], []
    step = window / 4
    s = t[0]
    while s + window <= t[-1]:
        sel = (t >= s) & (t < s + window)
        if sel.sum() > 10:
            k = np.polyfit(t[sel], y[sel], 1)[0]
            centres.append(s + window / 2)
            slopes.append(k)
        s += step
    return np.array(centres), np.array(slopes)


def replay(t, yaw, moved, cfg):
    """Run the recorded rate through the on-Pi filter.

    The recorded yaw is what the Pi integrated (calibrated bias already
    removed), so diff(yaw)/dt is the residual rate the gyro thread sees. Each
    telemetry interval is replayed as SUBSTEPS gyro steps, the encoder delta
    spread over them and the detector fed at the same rate.
    """
    clock = [0.0]
    det = StandstillDetector(cfg, clock=lambda: clock[0])
    zr = ZeroRateFilter(cfg)
    totals = {tag: 0 for tag in ENCODER_TAGS}
    det.update(totals)
    out = np.empty_like(yaw)
    weight = np.empty_like(yaw)
    out[0], weight[0] = yaw[0], 0.0
    angle = yaw[0]
    tag = ENCODER_TAGS[0]
    for i in range(1, len(t)):
        dt = (t[i] - t[i - 1]) / SUBSTEPS
        if dt <= 0:
            out[i], weight[i] = angle, weight[i - 1]
            continue
        rate = (yaw[i] - yaw[i - 1]) / (t[i] - t[i - 1])
        for k in range(SUBSTEPS):
            clock[0] += dt
            if k == 0:
                totals[tag] += int(moved[i])
            det.note_gyro_rate(rate - zr.bias)
            det.update(totals)
            angle += zr.step(rate, dt, det.stillness()) * dt
        out[i], weight[i] = angle, det.stillness()
    return out, weight, zr


def with_turns(t, yaw, moved):
    """The recording with three in-place turns spliced in: the rate the gyro
    would add, and the wheel counts a turn produces (~500 counts/s total)."""
    yaw = yaw.copy()
    moved = moved.copy()
    added = np.zeros_like(yaw)
    turns = [(0.25, 90.0, 3.0), (0.5, -45.0, 2.0), (0.75, 180.0, 4.0)]
    for frac, deg, dur in turns:
        t0 = t[0] + frac * (t[-1] - t[0])
        sel = (t >= t0) & (t < t0 + dur)
        prog = np.clip((t - t0) / dur, 0, 1)
        added += deg * prog
        moved[sel] += 50
    return yaw + added, moved, added


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("csv")
    ap.add_argument("--save", metavar="PNG",
                    help="write the figure here instead of showing it")
    ap.add_argument("--enabled", type=int, default=1,
                    help="replay with standstill enabled (default 1)")
    args = ap.parse_args()

    d = load(args.csv)
    t, yaw, moved = d["t"], d["yaw"], d["moved"]
    dur = t[-1] - t[0]
    k, c = np.polyfit(t, yaw, 1)
    resid = yaw - (k * t + c)
    taus, adev = allan(t, yaw)
    i_min = int(np.argmin(adev))
    wc, ws = sliding_slope(t, yaw, WINDOW_S)

    print(f"samples        {len(t)} over {dur:.1f} s "
          f"({len(t) / dur:.1f} Hz), encoder counts seen {int(moved.sum())}")
    print(f"drift          {yaw[-1] - yaw[0]:+.2f} deg total, "
          f"{k * 60:+.3f} deg/min  (residual bias {k:+.4f} deg/s)")
    print(f"fit residual   rms {resid.std():.3f} deg")
    if len(ws):
        print(f"bias over time {ws.min():+.4f} .. {ws.max():+.4f} deg/s "
              f"({WINDOW_S:g} s windows)")
    print(f"allan          bias instability {adev[i_min]:.4f} deg/s "
          f"at tau {taus[i_min]:.1f} s; "
          f"{adev[0]:.3f} deg/s at {taus[0]:.2f} s")

    cfg = replace(CONFIG, standstill=replace(CONFIG.standstill,
                                             enabled=bool(args.enabled)))
    filt, weight, zr = replay(t, yaw, moved, cfg)
    print(f"filtered       {filt[-1] - filt[0]:+.2f} deg total; learned bias "
          f"{zr.bias:+.4f} deg/s (ZeroRateFilter, "
          f"tau {cfg.standstill.bias_tau_s:g} s)")

    yaw_t, moved_t, added = with_turns(t, yaw, moved)
    filt_t, weight_t, _ = replay(t, yaw_t, moved_t, cfg)
    raw_t_err = (yaw_t - added) - (yaw_t[0])
    filt_t_err = (filt_t - added) - (filt_t[0])
    print(f"with turns     +90/-45/+180 deg spliced in: end error "
          f"raw {raw_t_err[-1]:+.2f} deg, filtered {filt_t_err[-1]:+.2f} deg")

    import matplotlib
    if args.save:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    plt.rcParams.update(THEME)
    fig, ax = plt.subplots(2, 2, figsize=(15, 8.5), layout="constrained")
    fig.suptitle(f"Heading drift — {os.path.basename(args.csv)}", x=0.01,
                 ha="left", fontsize=12, fontweight="bold")

    a = ax[0, 0]
    a.plot(t, yaw, color=SERIES[0], lw=1.4, label="gyro yaw (as on the Pi)")
    a.plot(t, k * t + c, color=AXIS, lw=0.9, ls="--",
           label=f"fit {k * 60:+.2f} deg/min")
    a.plot(t, filt, color=SERIES[2], lw=1.6, label="ZeroRateFilter replay")
    a.set_title("Parked kart: yaw")
    a.set_xlabel("s")
    a.set_ylabel("deg")
    a.legend(loc="best")

    a = ax[0, 1]
    a.loglog(taus, adev, color=SERIES[0], lw=1.4, marker="o", ms=3)
    a.plot(taus[i_min], adev[i_min], "o", color=SERIES[1], ms=7)
    a.annotate(f"{adev[i_min]:.4f} deg/s @ {taus[i_min]:.0f} s",
               (taus[i_min], adev[i_min]), textcoords="offset points",
               xytext=(8, 8), color=SERIES[1])
    a.set_title("Allan deviation of yaw rate")
    a.set_xlabel("tau  s")
    a.set_ylabel("deg/s")

    a = ax[1, 0]
    if len(ws):
        a.plot(wc, ws, color=SERIES[0], lw=1.4, marker="o", ms=3,
               label=f"slope over {WINDOW_S:g} s windows")
    a.axhline(k, color=AXIS, ls="--", lw=0.9, label="whole-run slope")
    a.axhline(0, color=AXIS, lw=0.8)
    a.set_title("Residual bias over time")
    a.set_xlabel("s")
    a.set_ylabel("deg/s")
    a.legend(loc="best")

    a = ax[1, 1]
    a.plot(t, raw_t_err, color=SERIES[0], lw=1.4, label="raw integration")
    a.plot(t, filt_t_err, color=SERIES[2], lw=1.6, label="filtered")
    a2 = a.twinx()
    a2.fill_between(t, 0, weight_t, color=SERIES[3], alpha=0.15, lw=0,
                    step="post")
    a2.set_ylim(0, 4)
    a2.set_yticks([0, 1])
    a2.set_ylabel("stillness weight", color=SERIES[3])
    a2.grid(False)
    a.set_title("Synthetic turns spliced in: heading error vs truth")
    a.set_xlabel("s")
    a.set_ylabel("deg")
    a.legend(loc="best")

    if args.save:
        fig.savefig(args.save, dpi=110)
        print(f"[INFO] saved {args.save}")
    else:
        plt.show()
    return 0


if __name__ == "__main__":
    sys.exit(main())
