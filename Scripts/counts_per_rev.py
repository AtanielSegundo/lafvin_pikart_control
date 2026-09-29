#!/usr/bin/python3
"""
Fit ``WheelGeometry.motor_counts_per_rev`` from a hand-logged calibration run.

Input CSV (``calibration.csv`` at the repo root by default):

    motor,ticks_i,ticks_f,n_voltas
    1    ,0      ,2556   ,1       ,
    1    ,2556   ,7617   ,2       ,

One row = one observation: motor ``motor`` was turned ``n_voltas`` whole wheel
revolutions by hand, and its lifetime encoder total went from ``ticks_i`` to
``ticks_f``. Rows may chain (each ``ticks_i`` equal to the previous ``ticks_f``)
or restart from anywhere -- only the delta within a row is used, so a reset
counter, a paused session or an out-of-order row costs nothing. Fields may be
space-padded and lines may carry a trailing comma; both are tolerated.

Estimator
---------
Per motor the answer is the **total-weighted** mean -- ``sum(delta) /
sum(n_voltas)`` -- not the mean of the per-row ratios. A 4-revolution row
measures the count-per-rev four times as precisely as a 1-revolution row (the
±1-tick read error at each end is amortised over four turns), and weighting by
turns is what credits that. The unweighted mean and the spread are printed too,
so a motor whose rows disagree is visible rather than averaged away.

M3 and the single-phase caveat
------------------------------
M3's phase B is dead, so it is decoded on phase A alone and counts HALF the
ticks per revolution. ``encoders.resolve_count`` already scales its raw count
x2 to reach the same x4 base as the other three, so the value that belongs in
``motor_counts_per_rev`` is the POST-scale figure -- i.e. the measured ticks
doubled. This script applies that automatically for any motor listed in
``config.SINGLE_PHASE_ENCODERS`` and says so in the report. Feeding the raw
~1290 straight into the config would make the right side read half its true
travel.

Usage
-----
    python Scripts/counts_per_rev.py                     # report
    python Scripts/counts_per_rev.py --csv other.csv
    python Scripts/counts_per_rev.py --write             # patch config.py
"""
from __future__ import annotations

import argparse
import csv
import os
import re
import statistics
import sys
from collections import defaultdict

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
SERVER = os.path.join(REPO, "Server")
sys.path.insert(0, SERVER)

DEFAULT_CSV = os.path.join(REPO, "calibration.csv")
CONFIG_PY = os.path.join(SERVER, "config.py")


def load_single_phase():
    """{tag: scale} for degraded encoders, from config. Empty if unimportable."""
    try:
        from config import SINGLE_PHASE_ENCODERS
        return {tag: float(m.get("scale", 1.0))
                for tag, m in SINGLE_PHASE_ENCODERS.items()}
    except Exception as exc:                                    # noqa: BLE001
        print(f"[warn] could not read SINGLE_PHASE_ENCODERS ({exc}); "
              f"assuming every motor is full quadrature", file=sys.stderr)
        return {}


class Row:
    __slots__ = ("line", "tag", "ticks_i", "ticks_f", "turns")

    def __init__(self, line, tag, ticks_i, ticks_f, turns):
        self.line = line
        self.tag = tag
        self.ticks_i = ticks_i
        self.ticks_f = ticks_f
        self.turns = turns

    @property
    def delta(self) -> int:
        return self.ticks_f - self.ticks_i

    @property
    def per_rev(self) -> float:
        return self.delta / self.turns


def parse(path):
    """Read the CSV into Rows, reporting (rows, problems)."""
    rows, problems = [], []
    with open(path, newline="", encoding="utf-8-sig") as fh:
        for lineno, raw in enumerate(csv.reader(fh), start=1):
            cells = [c.strip() for c in raw]
            while cells and cells[-1] == "":        # trailing comma
                cells.pop()
            if not cells:
                continue
            if lineno == 1 and not cells[0].lstrip("-").isdigit():
                continue                            # header
            if len(cells) < 4:
                problems.append(f"line {lineno}: expected 4 fields, got {cells}")
                continue
            try:
                motor, ticks_i, ticks_f, turns = (int(cells[0]), int(cells[1]),
                                                  int(cells[2]), int(cells[3]))
            except ValueError:
                problems.append(f"line {lineno}: non-integer field in {cells}")
                continue
            if turns <= 0:
                problems.append(f"line {lineno}: n_voltas must be > 0, got {turns}")
                continue
            row = Row(lineno, f"M{motor}", ticks_i, ticks_f, turns)
            if row.delta <= 0:
                # A counter reset mid-run, or the two columns were swapped.
                # Dropping it beats silently averaging in a negative rev count.
                problems.append(f"line {lineno}: {row.tag} ticks went "
                                f"{ticks_i} -> {ticks_f} (delta {row.delta}); "
                                f"dropped")
                continue
            rows.append(row)
    return rows, problems


def fit(rows, single_phase):
    """Per motor: the turn-weighted counts_per_rev, plus dispersion."""
    by_tag = defaultdict(list)
    for row in rows:
        by_tag[row.tag].append(row)

    results = {}
    for tag in sorted(by_tag):
        group = by_tag[tag]
        total_delta = sum(r.delta for r in group)
        total_turns = sum(r.turns for r in group)
        ratios = [r.per_rev for r in group]

        scale = single_phase.get(tag, 1.0)
        weighted = total_delta / total_turns
        results[tag] = {
            "rows": group,
            "n_rows": len(group),
            "total_turns": total_turns,
            "measured": weighted,                  # raw ticks/rev as counted
            "scale": scale,
            "counts_per_rev": int(round(weighted * scale)),   # x4 base
            "plain_mean": statistics.fmean(ratios),
            "stdev": statistics.stdev(ratios) if len(ratios) > 1 else 0.0,
            "min": min(ratios),
            "max": max(ratios),
        }
    return results


def report(results, problems, rows):
    if problems:
        print("Skipped rows")
        print("-" * 72)
        for p in problems:
            print(f"  {p}")
        print()

    print(f"Parsed {len(rows)} usable observations "
          f"({sum(r.turns for r in rows)} wheel revolutions total)")
    print()
    print("Per motor")
    print("-" * 72)
    print(f"{'motor':<6}{'rows':>5}{'revs':>6}{'measured':>11}"
          f"{'spread':>9}{'rel':>8}{'scale':>7}{'counts_per_rev':>16}")
    for tag, r in results.items():
        spread = r["max"] - r["min"]
        rel = spread / r["measured"] * 100.0 if r["measured"] else 0.0
        note = "" if r["scale"] == 1.0 else f" x{r['scale']:g}"
        print(f"{tag:<6}{r['n_rows']:>5}{r['total_turns']:>6}"
              f"{r['measured']:>11.1f}{spread:>9.1f}{rel:>7.1f}%"
              f"{note or '   -':>7}{r['counts_per_rev']:>16}")
    print()

    degraded = [t for t, r in results.items() if r["scale"] != 1.0]
    if degraded:
        print(f"{', '.join(degraded)} decode on a single phase and count half "
              f"the ticks per rev.")
        print("Their measured value is scaled to the x4 quadrature base, which "
              "is what")
        print("motor_counts_per_rev must hold -- resolve_count applies the same "
              "factor at")
        print("runtime, so an unscaled number here would halve that side's "
              "reported travel.")
        print()

    # A motor whose rows disagree by more than a few tenths of a percent is
    # usually a miscounted revolution, not encoder noise: +-1 tick at each end
    # of a 2500-tick turn is 0.08%.
    noisy = [(t, r) for t, r in results.items()
             if r["measured"] and (r["max"] - r["min"]) / r["measured"] > 0.01]
    if noisy:
        print("Check these -- rows disagree by more than 1%:")
        for tag, r in noisy:
            print(f"  {tag}: per-row ticks/rev "
                  f"{', '.join(f'{x.per_rev:.0f}' for x in r['rows'])}")
            print(f"       (lines {', '.join(str(x.line) for x in r['rows'])})")
        print()

    print("Paste into Server/config.py -> WheelGeometry:")
    print()
    body = ", ".join(f'"{t}": {r["counts_per_rev"]}' for t, r in results.items())
    print(f"    motor_counts_per_rev: Dict[str, int] = field("
          f"default_factory=lambda: {{{body}}})")
    print()


def write_config(results, path=CONFIG_PY):
    """Rewrite the motor_counts_per_rev default in config.py, in place."""
    with open(path, encoding="utf-8") as fh:
        source = fh.read()

    body = ", ".join(f'"{t}": {r["counts_per_rev"]}' for t, r in results.items())
    new_line = (f"    motor_counts_per_rev: Dict[str, int] = field("
                f"default_factory=lambda: {{{body}}})")
    pattern = re.compile(r"^ *motor_counts_per_rev *:.*$", re.MULTILINE)

    if not pattern.search(source):
        print(f"[error] no motor_counts_per_rev assignment found in {path}; "
              f"paste the line above by hand", file=sys.stderr)
        return False

    updated = pattern.sub(lambda _m: new_line, source, count=1)
    if updated == source:
        print(f"{path} already up to date")
        return True
    with open(path, "w", encoding="utf-8") as fh:
        fh.write(updated)
    print(f"Updated {path}")
    return True


def main(argv=None):
    ap = argparse.ArgumentParser(
        description="Fit motor_counts_per_rev from a calibration CSV.")
    ap.add_argument("--csv", default=DEFAULT_CSV,
                    help=f"input CSV (default: {DEFAULT_CSV})")
    ap.add_argument("--write", action="store_true",
                    help="patch Server/config.py in place instead of only "
                         "printing the line")
    args = ap.parse_args(argv)

    if not os.path.exists(args.csv):
        print(f"[error] no such file: {args.csv}", file=sys.stderr)
        return 2

    rows, problems = parse(args.csv)
    if not rows:
        print(f"[error] no usable rows in {args.csv}", file=sys.stderr)
        for p in problems:
            print(f"  {p}", file=sys.stderr)
        return 1

    results = fit(rows, load_single_phase())
    report(results, problems, rows)

    if args.write:
        return 0 if write_config(results) else 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
