#!/usr/bin/python3
"""
Live heading plot from the kart's telemetry.

Read-only: listens on the WebSocket /ws the web UI uses and sends nothing, so
it can run next to any other script (motor_calibration.py, tuning.py, the
browser) while the kart is driven from there.

Panels:
  * heading      gyro yaw, odometry theta (unwrapped) and, during a gyro turn,
                 the target yaw
  * yaw rate     d(gyro yaw)/dt, plus the controller's own rate_dps in a turn
  * turn         heading error and the turn duty, only while a turn is active
  * compass      the current yaw, odometry theta and target as needles

Fields (drive.* in the telemetry message, see DriveController.step):
    gyro.yaw_deg, gyro.connected, gyro.source,
    gyro.heading = {target_deg, error_deg, rate_dps, duty} or None,
    pose.theta_deg (wrapped to +-180), control_ts, stale

USAGE
-----
    python Scripts/plot_heading.py --ip 192.168.0.237
    python Scripts/plot_heading.py --window 60 --csv Scripts/data/heading.csv
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import os
import sys
import threading
import time
from collections import deque
from dataclasses import astuple, dataclass, fields
from typing import List, Optional

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
SERVER = os.path.join(REPO, "Server")
# Server/ is not a package -- see the same note in gray_regression.py.
if SERVER not in sys.path:
    sys.path.insert(0, SERVER)

from motor_calibration import DEFAULT_IP, DEFAULT_PORT, _WsConn  # noqa: E402
from plot_motor_calibration import (AXIS, INK_2, MUTED, SERIES,   # noqa: E402
                                    THEME)

WINDOW_S   = 20.0     # seconds of history on screen
REFRESH_MS = 100      # redraw period; telemetry arrives at ~10 Hz
BUFFER     = 6000     # samples kept (10 min at 10 Hz)
NAN = float("nan")


@dataclass
class HeadingSample:
    t: float                  # s; Pi control_ts when sent, else laptop clock
    yaw_deg: float            # gyro yaw
    theta_deg: float          # odometry theta, wrapped to +-180
    target_deg: float         # gyro-turn target, NaN when no turn
    error_deg: float
    rate_dps: float           # controller's measured yaw rate, NaN when none
    duty: float               # turn duty (right side), NaN when no turn
    connected: bool
    source: str               # "gyro" | "encoder": what feeds odometry theta
    stale: bool
    enc_moved: int            # sum |count delta| over all encoders since last
    still: float              # P_sensors standstill: 1 frozen, 0 moving, NaN n/a


def _num(v) -> float:
    return NAN if v is None else float(v)


class HeadingStream:
    """Telemetry reader thread keeping the last BUFFER heading samples."""

    def __init__(self, ip: str, port: int, path: str = "/ws",
                 csv_path: Optional[str] = None):
        self.ip, self.port, self.path = ip, port, path
        self._samples: deque = deque(maxlen=BUFFER)
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._conn: Optional[_WsConn] = None
        self._last_cts = None
        self._last_enc = None
        self._precise = None      # clock kind of the buffered samples
        self.status = "connecting..."
        self._csv = None
        if csv_path:
            f = open(csv_path, "w", newline="", encoding="utf-8")
            self._csv = (f, csv.writer(f))
            self._csv[1].writerow([fl.name for fl in fields(HeadingSample)])
        self._thread = threading.Thread(target=self._run, daemon=True,
                                        name="heading-telemetry")

    def start(self) -> None:
        self._thread.start()

    def close(self) -> None:
        self._stop.set()
        if self._conn is not None:
            self._conn.close()
        if self._csv is not None:
            self._csv[0].close()

    def snapshot(self) -> List[HeadingSample]:
        with self._lock:
            return list(self._samples)

    def _run(self) -> None:
        while not self._stop.is_set():
            try:
                self._conn = _WsConn(self.ip, self.port, self.path, timeout=5.0)
                self.status = "connected"
                self._read_loop(self._conn)
            except (OSError, ConnectionError) as e:
                if not self._stop.is_set():
                    self.status = f"reconnecting ({e})"
                    print(f"[telemetry] {e}; reconnecting...")
            finally:
                if self._conn is not None:
                    self._conn.close()
            self._stop.wait(0.5)

    def _read_loop(self, conn: _WsConn) -> None:
        text = b""
        while not self._stop.is_set():
            fin, opcode, payload = conn.read_frame()
            if opcode == 0x9:                       # ping -> pong
                conn.send_frame(0xA, payload)
            elif opcode == 0x8:                     # close
                return
            elif opcode in (0x1, 0x0):              # text / continuation
                text += payload
                if fin:
                    self._on_text(text)
                    text = b""

    def _on_text(self, data: bytes) -> None:
        try:
            msg = json.loads(data.decode())
        except ValueError:
            return
        if msg.get("type") != "telemetry":
            return
        drive = msg.get("drive") or {}
        gyro = drive.get("gyro") or {}
        pose = drive.get("pose") or {}
        turn = gyro.get("heading") or {}
        cts = drive.get("control_ts") or 0.0
        # The broadcaster (10 Hz) can resend one control snapshot; same stamp.
        if cts and cts == self._last_cts:
            return
        self._last_cts = cts
        enc = drive.get("encoders") or {}
        prev, self._last_enc = self._last_enc, enc
        moved = (sum(abs(int(v) - int(prev.get(k, v))) for k, v in enc.items())
                 if prev else 0)
        still = (msg.get("standstill") or {}).get("moving")
        sample = HeadingSample(
            t=float(cts) if cts else time.monotonic(),
            yaw_deg=_num(gyro.get("yaw_deg")),
            theta_deg=_num(pose.get("theta_deg")),
            target_deg=_num(turn.get("target_deg")),
            error_deg=_num(turn.get("error_deg")),
            rate_dps=_num(turn.get("rate_dps")),
            duty=_num(turn.get("duty")),
            connected=bool(gyro.get("connected", False)),
            source=str(gyro.get("source", "")),
            stale=bool(drive.get("stale", False)),
            enc_moved=moved,
            still=NAN if still is None else float(not still))
        with self._lock:
            # control_ts and the laptop clock are different clocks; never mix
            # them on one time axis (control_ts is 0 until P_control reports).
            if self._precise is not None and self._precise != bool(cts):
                self._samples.clear()
            self._precise = bool(cts)
            self._samples.append(sample)
        if self._csv is not None:
            self._csv[1].writerow(astuple(sample))


class HeadingPlot:
    def __init__(self, stream: HeadingStream, window_s: float, title: str):
        import matplotlib.pyplot as plt
        self.plt, self.stream, self.window = plt, stream, window_s
        plt.rcParams.update(THEME)
        self.fig = plt.figure(figsize=(14, 8), layout="constrained")
        try:
            self.fig.canvas.manager.set_window_title(title)
        except AttributeError:
            pass
        self.fig.suptitle(title, x=0.01, ha="left", fontsize=12,
                          fontweight="bold")
        gs = self.fig.add_gridspec(3, 2, width_ratios=(2.6, 1.0),
                                   height_ratios=(1.6, 1.0, 1.0))
        self.ax_h = self.fig.add_subplot(gs[0, 0])
        self.ax_r = self.fig.add_subplot(gs[1, 0], sharex=self.ax_h)
        self.ax_t = self.fig.add_subplot(gs[2, 0], sharex=self.ax_h)
        self.ax_c = self.fig.add_subplot(gs[:, 1], projection="polar")

        c_yaw, c_odo, c_tgt, c_duty = SERIES
        self.l_yaw, = self.ax_h.plot([], [], color=c_yaw, lw=1.8,
                                     label="gyro yaw")
        self.l_odo, = self.ax_h.plot([], [], color=c_odo, lw=1.2,
                                     label="odometry theta")
        self.l_tgt, = self.ax_h.plot([], [], color=c_tgt, lw=1.4, ls="--",
                                     drawstyle="steps-post", label="target")
        self.ax_h.set_title("Heading")
        self.ax_h.set_ylabel("deg")
        self.ax_h.legend(loc="upper left", ncol=3)

        self.l_rate, = self.ax_r.plot([], [], color=c_yaw, lw=1.4,
                                      label="d(yaw)/dt")
        self.l_crate, = self.ax_r.plot([], [], color=c_tgt, lw=1.2, ls="--",
                                       label="controller rate")
        self.ax_r.axhline(0, color=AXIS, lw=0.8)
        self.ax_r.set_title("Yaw rate")
        self.ax_r.set_ylabel("deg/s")
        self.ax_r.legend(loc="upper left", ncol=2)

        self.l_err, = self.ax_t.plot([], [], color=c_tgt, lw=1.4,
                                     label="error")
        self.ax_t.axhline(0, color=AXIS, lw=0.8)
        self.ax_t.set_title("Gyro turn  (only while a turn is active)")
        self.ax_t.set_ylabel("error deg")
        self.ax_t.set_xlabel("s")
        self.ax_d = self.ax_t.twinx()
        self.ax_d.spines["right"].set_visible(True)
        self.ax_d.grid(False)
        self.l_duty, = self.ax_d.plot([], [], color=c_duty, lw=1.2,
                                      label="duty")
        self.ax_d.set_ylabel("duty", color=INK_2)
        self.ax_t.legend(handles=[self.l_err, self.l_duty], loc="upper left",
                         ncol=2)

        # Compass: 0 deg up, positive yaw counter-clockwise (math convention).
        ax = self.ax_c
        ax.set_theta_zero_location("N")
        ax.set_theta_direction(1)
        ax.set_ylim(0, 1)
        ax.set_yticklabels([])
        ax.grid(color="#e1e0d9")
        self.n_yaw, = ax.plot([], [], color=c_yaw, lw=3, solid_capstyle="round")
        self.n_odo, = ax.plot([], [], color=c_odo, lw=1.6)
        self.n_tgt, = ax.plot([], [], color=c_tgt, lw=1.6, ls="--")
        self.readout = ax.text(0.5, -0.12, "", transform=ax.transAxes,
                               ha="center", va="top", ma="left",
                               family="monospace",
                               fontsize=9, color=INK_2)
        ax.set_title("Compass", pad=18)

        self.status = self.fig.text(0.99, 0.985, "", ha="right", va="top",
                                    fontsize=9, color=MUTED)

    @staticmethod
    def _needle(line, deg: float, length: float = 0.95) -> None:
        if math.isfinite(deg):
            line.set_data([0, math.radians(deg)], [0, length])
        else:
            line.set_data([], [])

    def update(self, _frame=None):
        samples = self.stream.snapshot()
        if not samples:
            self.status.set_text(self.stream.status)
            return
        t_end = samples[-1].t
        samples = [s for s in samples if s.t >= t_end - self.window]
        arr = np.array([(s.t, s.yaw_deg, s.theta_deg, s.target_deg,
                         s.error_deg, s.rate_dps, s.duty) for s in samples])
        t = arr[:, 0] - t_end
        yaw, theta, target, err, crate, duty = arr[:, 1:].T
        theta_u = theta.copy()
        ok = np.isfinite(theta)
        theta_u[ok] = np.degrees(np.unwrap(np.radians(theta[ok])))

        self.l_yaw.set_data(t, yaw)
        self.l_odo.set_data(t, theta_u)
        self.l_tgt.set_data(t, target)
        if len(t) > 1:
            dt = np.diff(t)
            rate = np.full_like(t, NAN)
            rate[1:] = np.where(dt > 0, np.diff(yaw) / np.where(dt > 0, dt, 1),
                                NAN)
        else:
            rate = np.full_like(t, NAN)
        self.l_rate.set_data(t, rate)
        self.l_crate.set_data(t, crate)
        self.l_err.set_data(t, err)
        self.l_duty.set_data(t, duty)

        self.ax_h.set_xlim(-self.window, 0)
        for ax in (self.ax_h, self.ax_r, self.ax_t, self.ax_d):
            ax.relim()
            ax.autoscale_view(scalex=False)

        last = samples[-1]
        self._needle(self.n_yaw, last.yaw_deg)
        self._needle(self.n_odo, last.theta_deg, 0.8)
        self._needle(self.n_tgt, last.target_deg)
        turning = math.isfinite(last.target_deg)
        rows = [f"yaw     {last.yaw_deg:8.2f} deg",
                f"odom    {last.theta_deg:8.2f} deg  ({last.source})"]
        if turning:
            rows += [f"target  {last.target_deg:8.2f} deg",
                     f"error   {last.error_deg:8.2f} deg",
                     f"duty    {last.duty:8.0f}"]
        self.readout.set_text("\n".join(rows))

        flags = [self.stream.status]
        if not last.connected:
            flags.append("GYRO DISCONNECTED")
        if last.stale:
            flags.append("STALE")
        if turning:
            flags.append("turning")
        self.status.set_text("  |  ".join(flags))
        self.status.set_color("#c0392b" if (last.stale or not last.connected)
                              else MUTED)

    def run(self) -> None:
        from matplotlib.animation import FuncAnimation
        # Kept on self: an unreferenced FuncAnimation is garbage collected.
        self._anim = FuncAnimation(self.fig, self.update, interval=REFRESH_MS,
                                   cache_frame_data=False)
        self.plt.show()


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--ip", default=DEFAULT_IP)
    ap.add_argument("--port", type=int, default=DEFAULT_PORT)
    ap.add_argument("--window", type=float, default=WINDOW_S,
                    help=f"seconds of history on screen (default {WINDOW_S:g})")
    ap.add_argument("--csv", metavar="PATH",
                    help="also log every sample to this CSV")
    ap.add_argument("--duration", type=float, metavar="S",
                    help="no window: record --csv for S seconds, then exit")
    args = ap.parse_args()
    if args.duration and not args.csv:
        ap.error("--duration needs --csv")

    stream = HeadingStream(args.ip, args.port, csv_path=args.csv)
    stream.start()
    if args.duration:
        end = time.monotonic() + args.duration
        try:
            while time.monotonic() < end:
                time.sleep(10.0)
                s = stream.snapshot()
                if s:
                    print(f"[{len(s):5d}] yaw {s[-1].yaw_deg:+8.2f} deg  "
                          f"({stream.status})", flush=True)
        except KeyboardInterrupt:
            pass
        finally:
            stream.close()
        return 0
    plot = HeadingPlot(stream, args.window,
                       f"Heading — {args.ip}:{args.port}")
    try:
        plot.run()
    except KeyboardInterrupt:
        pass
    finally:
        stream.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
