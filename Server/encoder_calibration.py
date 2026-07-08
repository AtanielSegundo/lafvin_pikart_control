"""
Encoder calibration / test rig for the Lafvin PiKart.

Drives all four motors at a sweep of PWM levels for a fixed time, captures the
RAW per-motor encoder counts and the odometry pose *before and after* each run
over the telemetry WebSocket, then asks the user for the REAL distance the car
travelled. From that it computes, per motor:

  * counts-per-metre (and flags any motor that under-counts vs. its side-mate),
  * the odometry-reported distance vs. the measured distance (scale error),
  * a suggested `counts_per_rev` and the implied wheel diameter,
  * a V(PWM) = K*(PWM - threshold) velocity model (as in the PWM script).

Transport mirrors the existing tools:
  * motor commands  -> HTTP POST /command   (legacy `CMD_MOTOR#..`)
  * telemetry (raw counts, pose) -> read-only WebSocket /ws

Run from a laptop on the same network as the kart. No Pi libs required.
"""
from __future__ import annotations

import base64
import csv
import json
import math
import os
import socket
import statistics
import struct
import sys
import time
from datetime import datetime
from pathlib import Path

import requests as rq


# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------
class Env:
    IPV4_ADDRESS = "192.168.0.237"
    PORT         = 8080
    COMMAND_ROUTE = "command"     # HTTP POST endpoint for CMD_MOTOR
    WS_PATH       = "/ws"         # WebSocket telemetry endpoint


class TestConfig:
    PWM_RANGE  = [1000, 1500, 2000, 2500, 3000, 3500, 4095]
    TIME       = 2.0     # s, motor-on duration per run
    RUNS       = 3       # repeats per PWM level
    RETRIES    = 3       # HTTP send retries
    SETTLE     = 0.5     # s, wait after STOP before final telemetry snapshot
    OUTPUT_DIR = Path(__file__).parent

    # Keep in sync with Server/config.py (single source of truth on the Pi).
    DIAMETER       = 0.065          # m
    COUNTS_PER_REV = 2340           # x4 quadrature
    TRACK          = 0.151          # m
    MOTORS = ("M1", "M2", "M3", "M4")
    LEFT   = ("M1", "M2")
    RIGHT  = ("M3", "M4")
    SIGNS  = {"M1": 1, "M2": 1, "M3": -1, "M4": -1}

    @classmethod
    def meters_per_count(cls) -> float:
        return math.pi * cls.DIAMETER / cls.COUNTS_PER_REV


# ---------------------------------------------------------------------------
# Motor commands over HTTP  (POST /command)
# ---------------------------------------------------------------------------
def command_url() -> str:
    return f"http://{Env.IPV4_ADDRESS}:{Env.PORT}/{Env.COMMAND_ROUTE}"


def motors_cmd_fmt(m1: int, m2: int, m3: int, m4: int) -> str:
    return f"CMD_MOTOR#{m1}#{m2}#{m3}#{m4}"


def set_motors_pwm(m1: int, m2: int, m3: int, m4: int) -> rq.Response:
    for x in (m1, m2, m3, m4):
        if type(x) is not int:
            raise TypeError(f"Motor PWM must be int, got {type(x).__name__}")
    return rq.post(command_url(), data=motors_cmd_fmt(m1, m2, m3, m4), timeout=1.0)


def send_pwm_with_retries(m1: int, m2: int, m3: int, m4: int, retries: int) -> bool:
    for _ in range(retries):
        try:
            if set_motors_pwm(m1, m2, m3, m4).ok:
                return True
        except rq.RequestException:
            continue
    return False


def stop_motors() -> None:
    # best-effort: retry hard; if it still fails, warn for manual intervention.
    if not send_pwm_with_retries(0, 0, 0, 0, retries=5):
        print("[ERRO] FALHA AO PARAR MOTORES — VERIFIQUE O CARRO MANUALMENTE")


# ---------------------------------------------------------------------------
# Telemetry over a short-lived, read-only WebSocket  (GET /ws)
# ---------------------------------------------------------------------------
def _recvn(sock: socket.socket, n: int) -> bytes:
    buf = b""
    while len(buf) < n:
        chunk = sock.recv(n - len(buf))
        if not chunk:
            break
        buf += chunk
    return buf


def _recv_until(sock: socket.socket, marker: bytes) -> bytes:
    buf = b""
    while marker not in buf:
        chunk = sock.recv(4096)
        if not chunk:
            break
        buf += chunk
    return buf


def _read_frame(sock: socket.socket):
    """Read one WebSocket frame -> (opcode, payload) or None on EOF."""
    hdr = _recvn(sock, 2)
    if len(hdr) < 2:
        return None
    b1, b2 = hdr[0], hdr[1]
    opcode = b1 & 0x0F
    masked = b2 & 0x80
    length = b2 & 0x7F
    if length == 126:
        length = struct.unpack(">H", _recvn(sock, 2))[0]
    elif length == 127:
        length = struct.unpack(">Q", _recvn(sock, 8))[0]
    mask = _recvn(sock, 4) if masked else b""
    payload = _recvn(sock, length)
    if masked:
        payload = bytes(c ^ mask[i % 4] for i, c in enumerate(payload))
    return opcode, payload


def _ws_connect(timeout: float) -> socket.socket:
    s = socket.create_connection((Env.IPV4_ADDRESS, Env.PORT), timeout=timeout)
    key = base64.b64encode(os.urandom(16)).decode()
    req = (
        f"GET {Env.WS_PATH} HTTP/1.1\r\nHost: {Env.IPV4_ADDRESS}:{Env.PORT}\r\n"
        "Upgrade: websocket\r\nConnection: Upgrade\r\n"
        f"Sec-WebSocket-Key: {key}\r\nSec-WebSocket-Version: 13\r\n\r\n"
    )
    s.sendall(req.encode())
    resp = _recv_until(s, b"\r\n\r\n")
    if b"101" not in resp.split(b"\r\n")[0]:
        s.close()
        raise ConnectionError(f"WS handshake failed: {resp[:120]!r}")
    return s


def telemetry_snapshot(settle: float = 0.4, timeout: float = 6.0) -> dict | None:
    """Open a short-lived WS and return the MOST RECENT telemetry message.

    Short-lived so we never have to answer server ping frames between runs.
    """
    try:
        s = _ws_connect(timeout)
    except OSError as e:
        print(f"[ERRO] WS connect: {e}")
        return None
    s.settimeout(timeout)
    latest = None
    end = time.time() + settle
    try:
        while time.time() < end:
            frame = _read_frame(s)
            if frame is None:
                break
            opcode, payload = frame
            if opcode == 0x8:            # close
                break
            if opcode != 0x1:            # want text frames only
                continue
            try:
                msg = json.loads(payload.decode())
            except Exception:
                continue
            if msg.get("type") == "telemetry":
                latest = msg
    except socket.timeout:
        pass
    finally:
        try:
            s.close()
        except OSError:
            pass
    return latest


def parse_snapshot(msg: dict) -> tuple[dict, dict] | None:
    """(pose, raw_counts) from a telemetry message, or None if incomplete."""
    if not msg:
        return None
    d = msg.get("drive", {})
    pose = d.get("pose", {})
    enc = d.get("encoders", {})
    if not enc or not all(m in enc for m in TestConfig.MOTORS):
        return None
    return pose, enc


# ---------------------------------------------------------------------------
# User input
# ---------------------------------------------------------------------------
def prompt_distance() -> float | None:
    while True:
        raw = input("Distancia REAL percorrida em metros (vazio = descartar): ").strip()
        if not raw:
            return None
        try:
            return float(raw.replace(",", "."))
        except ValueError:
            print("Valor invalido, tente novamente.")


# ---------------------------------------------------------------------------
# Per-run measurement
# ---------------------------------------------------------------------------
def wrap_deg(a: float) -> float:
    return (a + 180.0) % 360.0 - 180.0


def run_trial(pwm: int) -> dict | None:
    """Drive at `pwm` for TIME s, capturing encoder + pose deltas.

    Returns a dict of raw deltas / odometry deltas, or None if telemetry or the
    drive command failed (caller should skip the sample).
    """
    before = parse_snapshot(telemetry_snapshot(settle=0.4))
    if before is None:
        print("[ERRO] sem telemetria ANTES do run — pulando amostra")
        return None
    pose0, enc0 = before

    try:
        if not send_pwm_with_retries(pwm, pwm, pwm, pwm, TestConfig.RETRIES):
            print("[ERRO] CONEXAO COM KART PERDIDA — pulando amostra")
            return None
        time.sleep(TestConfig.TIME)
    finally:
        stop_motors()

    time.sleep(TestConfig.SETTLE)   # let the car coast to a stop
    after = parse_snapshot(telemetry_snapshot(settle=0.4))
    if after is None:
        print("[ERRO] sem telemetria DEPOIS do run — pulando amostra")
        return None
    pose1, enc1 = after

    raw = {m: enc1[m] - enc0[m] for m in TestConfig.MOTORS}
    signed = {m: raw[m] * TestConfig.SIGNS[m] for m in TestConfig.MOTORS}
    left = statistics.fmean(signed[m] for m in TestConfig.LEFT)
    right = statistics.fmean(signed[m] for m in TestConfig.RIGHT)

    dx = (pose1.get("x", 0.0) or 0.0) - (pose0.get("x", 0.0) or 0.0)
    dy = (pose1.get("y", 0.0) or 0.0) - (pose0.get("y", 0.0) or 0.0)
    dth = wrap_deg((pose1.get("theta_deg", 0.0) or 0.0)
                   - (pose0.get("theta_deg", 0.0) or 0.0))

    # Live feedback so an under-counting encoder is obvious immediately.
    print("  raw deltas :", {m: raw[m] for m in TestConfig.MOTORS})
    print(f"  side mean  : left={left:.0f}  right={right:.0f}")
    print(f"  odometry   : dist={math.hypot(dx, dy):.3f} m  dtheta={dth:+.1f} deg")

    return {
        "signed": signed, "left": left, "right": right,
        "odom_dist": math.hypot(dx, dy), "odom_dtheta": dth,
    }


# ---------------------------------------------------------------------------
# Persistence + analysis
# ---------------------------------------------------------------------------
CSV_HEADER = ["pwm", "run", "real_distance_m", "time_s", "velocity_mps",
              "odom_dist_m", "odom_dtheta_deg",
              "dM1", "dM2", "dM3", "dM4", "left_mean", "right_mean"]


def row_from(pwm: int, run: int, real_dist: float, trial: dict) -> list:
    s = trial["signed"]
    return [pwm, run, real_dist, TestConfig.TIME, real_dist / TestConfig.TIME,
            round(trial["odom_dist"], 4), round(trial["odom_dtheta"], 2),
            s["M1"], s["M2"], s["M3"], s["M4"],
            round(trial["left"], 1), round(trial["right"], 1)]


def save_csv(path: Path, rows: list[list]) -> None:
    with path.open("w", newline="") as f:
        w = csv.writer(f)
        w.writerow(CSV_HEADER)
        w.writerows(rows)


def analyze(rows: list[list]) -> None:
    """Print per-motor counts/m, under-count flags, scale + model."""
    if not rows:
        return
    cols = {name: i for i, name in enumerate(CSV_HEADER)}
    dist = [r[cols["real_distance_m"]] for r in rows]
    odom = [r[cols["odom_dist_m"]] for r in rows]

    print("\n" + "=" * 60)
    print("ANALISE")
    print("=" * 60)

    # -- per-motor counts per metre (magnitude) --------------------------
    print("\ncounts / metro (por motor):")
    cpm_by_motor = {}
    for m in TestConfig.MOTORS:
        ci = cols["d" + m]
        vals = [abs(r[ci]) / r[cols["real_distance_m"]]
                for r in rows if r[cols["real_distance_m"]] > 0]
        if vals:
            cpm_by_motor[m] = statistics.fmean(vals)
            sd = statistics.pstdev(vals) if len(vals) > 1 else 0.0
            print(f"  {m}: {cpm_by_motor[m]:8.1f}  (+/-{sd:6.1f}, n={len(vals)})")

    # -- under-count / dead-encoder flags --------------------------------
    if cpm_by_motor:
        med = statistics.median(cpm_by_motor.values())
        print(f"\n  mediana = {med:.1f} counts/m  -> checando divergencia:")
        for m, cpm in cpm_by_motor.items():
            ratio = cpm / med if med else 0
            if ratio < 0.5:
                print(f"    [!!] {m} conta {ratio*100:.0f}% da mediana — encoder MORTO/meia-fase?")
            elif ratio < 0.85:
                print(f"    [! ] {m} conta {ratio*100:.0f}% da mediana — SUBCONTAGEM")
            else:
                print(f"    [ok] {m} {ratio*100:.0f}%")

    # -- odometry scale vs. reality --------------------------------------
    good = [(o, d) for o, d in zip(odom, dist) if d > 0]
    if good:
        f = statistics.fmean(o / d for o, d in good)
        print(f"\nescala odometria: reportado/real = {f:.3f}")
        print(f"  -> counts_per_rev sugerido = {TestConfig.COUNTS_PER_REV * f:.0f} "
              f"(atual {TestConfig.COUNTS_PER_REV})")
        if cpm_by_motor:
            med = statistics.median(cpm_by_motor.values())
            circ = TestConfig.COUNTS_PER_REV / med
            print(f"  -> diametro de roda implicito = {circ / math.pi:.4f} m "
                  f"(config {TestConfig.DIAMETER} m)")

    # -- straightness ----------------------------------------------------
    dtheta = [abs(r[cols["odom_dtheta_deg"]]) for r in rows]
    if dtheta:
        print(f"\ndesvio de rumo medio (deveria ~0 em linha reta): "
              f"{statistics.fmean(dtheta):.1f} deg  (max {max(dtheta):.1f})")

    # -- velocity model  V(PWM) = K*(PWM - threshold) --------------------
    fit = fit_velocity(rows, cols)
    if fit:
        K, thr = fit
        print(f"\nModelo: V(PWM) = {K:.6g} * (PWM - {thr:.2f})")
        print(f"  K         = {K:.6g}")
        print(f"  Threshold = {thr:.2f}")


def fit_velocity(rows: list[list], cols: dict) -> tuple[float, float] | None:
    pts = [(r[cols["pwm"]], r[cols["velocity_mps"]]) for r in rows]
    if len(pts) < 2:
        return None
    try:
        import numpy as np
        pwm = np.array([p[0] for p in pts], dtype=float)
        v = np.array([p[1] for p in pts], dtype=float)
        a, b = np.polyfit(pwm, v, 1)
    except Exception:
        # manual least squares fallback (no numpy)
        n = len(pts)
        sx = sum(p[0] for p in pts); sy = sum(p[1] for p in pts)
        sxx = sum(p[0] ** 2 for p in pts); sxy = sum(p[0] * p[1] for p in pts)
        denom = n * sxx - sx * sx
        if denom == 0:
            return None
        a = (n * sxy - sx * sy) / denom
        b = (sy - a * sx) / n
    if a == 0:
        return None
    return float(a), float(-b / a)


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def main() -> None:
    print("CALIBRACAO DE ENCODERS — POSICIONE O CARRO NA REFERENCIA")
    print(f"kart: {command_url()}   ws: ws://{Env.IPV4_ADDRESS}:{Env.PORT}{Env.WS_PATH}\n")

    # Sanity: can we read telemetry at all?
    if parse_snapshot(telemetry_snapshot(settle=0.6)) is None:
        print("[ERRO] Nao recebi telemetria valida do kart. "
              "Verifique IP/porta e se o servidor esta rodando.")
        sys.exit(1)

    rows: list[list] = []
    try:
        for pwm in TestConfig.PWM_RANGE:
            for run in range(TestConfig.RUNS):
                print(f"\n--- PWM {pwm}  RUN {run + 1}/{TestConfig.RUNS} ---")
                input("Enter para acionar (mede a distancia depois)...")
                trial = run_trial(pwm)
                if trial is None:
                    continue
                dist = prompt_distance()
                if dist is None:
                    print("  amostra descartada.")
                    continue
                rows.append(row_from(pwm, run, dist, trial))
                print("  ok. REPOSICIONE O CARRO NA REFERENCIA")
    except KeyboardInterrupt:
        print("\nInterrompido pelo usuario.")
    finally:
        stop_motors()
        if not rows:
            print("\nNenhuma medida coletada.")
        else:                    
            ts = datetime.now().strftime("%Y%m%d_%H%M%S")
            out = TestConfig.OUTPUT_DIR / f"encoder_calibration_{ts}.csv"
            save_csv(out, rows)
            print(f"\nMedidas salvas em {out}")
            analyze(rows)


if __name__ == "__main__":
    main()
