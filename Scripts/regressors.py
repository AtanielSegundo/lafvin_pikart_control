"""
Fit the PiKart plant models from gray_regression.py CSVs and derive PID gains.

Two loops, identical structure, different constants:

    distance:  gamma   / (s*(tau*s + 1))     input = duty, inner output = v_center
    heading :  gamma_w / (s*(tau_w*s + 1))   input = duty, inner output = w_gyro

The CSVs record the INNER (rate) signal, so the fit is first order

    tau * y_dot + y = K * (u - u0)

and the free integrator in the plant above is the rate integrating into
distance / heading. That is why fitting v_center yields the position plant and
fitting w_gyro yields the heading plant -- no extra work needed.

Gains come from placing the closed-loop poles of

    tau*s^3 + (gamma*k_d + 1)*s^2 + gamma*k_p*s + gamma*k_i = 0

onto  tau * (s^2 + 2*zeta*omega_n*s + omega_n^2) * (s + p_3), giving

    p_3 = (gamma*k_d + 1)/tau - 2*zeta*omega_n
    k_p = (tau/gamma) * (omega_n^2 + 2*zeta*omega_n*p_3)
    k_i = (tau/gamma) * (omega_n^2 * p_3)

NOTE on p_3: the +1 sits INSIDE the division by tau. Verified against the
symbolic coefficient match -- reading it as "gamma*k_d + 1/tau" only agrees
when tau == 1, and silently wrecks k_p and k_i otherwise.

k_d is free: (zeta, omega_n) only pin the dominant pair, and k_d is the knob
that sets the third pole. Left unspecified it is SEARCHED for -- the smallest
k_d whose simulated step actually meets OVERSHOOT / SETTLING / RISE (and, with
--max-duty, stays out of saturation). Smallest, because k_p, k_i, derivative
noise gain and peak duty demand all grow with k_d while the response stops
improving once the third pole is out of the way. See choose_kd.

No third-party dependencies: the fit is a 3-parameter least squares, solved
with normal equations, so this runs on the Pi or on a laptop.

    python3 Scripts/regressors.py
    python3 Scripts/regressors.py --overshoot 2 --settling 1.5
"""
import argparse
import csv
import glob
import math
import os
import sys

__HERE   = os.path.dirname(os.path.abspath(__file__))
__PARENT = os.path.dirname(__HERE)
__SERVER = os.path.join(__PARENT, "Server")

if __SERVER not in sys.path:
    sys.path.insert(0, __SERVER)

from config import CONFIG          # pure dataclasses, no hardware imports

DATA_DIR = os.path.join(__HERE, "data")

# ---------------------------------------------------------------------------
# Closed-loop specification
# ---------------------------------------------------------------------------
OVERSHOOT_PCT   = 5.0     # PO, percent
SETTLING_TIME_S = 2.0     # 4.6 / (zeta*omega_n), 1% criterion
RISE_TIME_S     = 1.0     # (pi - acos(zeta)) / omega_d

# gamma = K * WHEEL_SCALE. The spec says gamma = K * wheel diameter, which is
# right when the fitted K is ANGULAR (wheel rad/s per duty). The CSVs here hold
# v_center in m/s, i.e. K is already linear, so scaling by the diameter again
# would double-count. Set to CONFIG.wheel.diameter to follow the spec verbatim.
WHEEL_SCALE = 1.0


def zeta_from_overshoot(po_pct):
    """zeta = -ln(PO/100) / sqrt(pi^2 + ln^2(PO/100))."""
    if not 0.0 < po_pct < 100.0:
        raise ValueError("overshoot must be in (0, 100) percent")
    l = math.log(po_pct / 100.0)
    return -l / math.sqrt(math.pi ** 2 + l ** 2)


def wn_from_settling(zeta, ts):
    """Invert SETTLING_TIME = 4.6 / (zeta * omega_n)."""
    return 4.6 / (zeta * ts)


def wn_from_rise(zeta, tr):
    """Invert RISE_TIME = (pi - acos(zeta)) / omega_d, omega_d = wn*sqrt(1-z^2)."""
    return (math.pi - math.acos(zeta)) / (tr * math.sqrt(1.0 - zeta ** 2))


def damped(wn, zeta):
    return wn * math.sqrt(1.0 - zeta ** 2)


def spec_to_poles(po_pct=OVERSHOOT_PCT, ts=SETTLING_TIME_S, tr=RISE_TIME_S):
    """(zeta, omega_n) from the three specs.

    Only two are independent: zeta comes from the overshoot, and then EITHER
    settling or rise time fixes omega_n. They are over-determined, so take the
    more demanding one -- meeting it also meets the looser one -- and report
    both so a contradictory spec is visible rather than silently half-applied.
    """
    zeta = zeta_from_overshoot(po_pct)
    wn_ts = wn_from_settling(zeta, ts)
    wn_tr = wn_from_rise(zeta, tr)
    wn = max(wn_ts, wn_tr)
    return zeta, wn, wn_ts, wn_tr


def achieved(zeta, wn):
    """Actual settling / rise time for the chosen pair."""
    return 4.6 / (zeta * wn), (math.pi - math.acos(zeta)) / damped(wn, zeta)


# ---------------------------------------------------------------------------
# Least squares (normal equations, 3 unknowns)
# ---------------------------------------------------------------------------
def _solve3(A, b):
    """Gaussian elimination with partial pivoting on a 3x3 system."""
    M = [list(A[i]) + [b[i]] for i in range(3)]
    for col in range(3):
        piv = max(range(col, 3), key=lambda r: abs(M[r][col]))
        if abs(M[piv][col]) < 1e-18:
            raise ValueError("sistema singular -- dados insuficientes ou "
                             "sem excitacao (duty constante?)")
        M[col], M[piv] = M[piv], M[col]
        for r in range(col + 1, 3):
            f = M[r][col] / M[col][col]
            for c in range(col, 4):
                M[r][c] -= f * M[col][c]
    x = [0.0] * 3
    for r in (2, 1, 0):
        x[r] = (M[r][3] - sum(M[r][c] * x[c] for c in range(r + 1, 3))) / M[r][r]
    return x


def _lstsq3(rows_x, rows_y):
    """Least squares for y = x . theta with 3 regressors."""
    A = [[0.0] * 3 for _ in range(3)]
    b = [0.0] * 3
    for x, y in zip(rows_x, rows_y):
        for i in range(3):
            b[i] += x[i] * y
            for j in range(3):
                A[i][j] += x[i] * x[j]
    return _solve3(A, b)


# ---------------------------------------------------------------------------
# CSV loading / first-order identification
# ---------------------------------------------------------------------------
def load_rows(paths):
    rows = []
    for p in paths:
        with open(p, newline="") as fh:
            for r in csv.DictReader(fh):
                r["_src"] = os.path.basename(p)
                rows.append(r)
    return rows


def _f(row, key):
    v = row.get(key, "")
    if v is None or v == "":
        return None
    try:
        return float(v)
    except ValueError:
        return None


def fit_first_order(rows, y_col, phase, dt_tol=0.2):
    """Fit tau*y_dot + y = K*(u - u0) by ARX least squares.

    Discretised at a uniform Ts (the median dt) as

        y[k+1] = a*y[k] + b*u[k+1] + c,   a = exp(-Ts/tau), b = K*(1-a), c = -b*u0

    Note the u INDEX. gray_regression logs each row as "the interval that just
    ended": its `duty` is the input that was in force during that interval and
    its `v_center` / `w_gyro` is the output at the end of it. So the input
    driving y[k+1] is the duty on row k+1, not row k. Pairing y[k+1] with u[k]
    shifts the input one sample late, which the fit absorbs as a slower pole --
    on synthetic data with a known plant it dragged tau from 0.18 down to 0.144
    and K down by 30%, with R^2 still reading 0.98.

    Samples whose dt strays more than `dt_tol` from the median are dropped: the
    uniform-Ts model does not hold across a scheduling hiccup, and one stretched
    interval biases `a` -- which is exactly tau.

    Returns a dict, or None when there is not enough usable data.
    """
    use = [r for r in rows if r.get("phase") == phase]
    if len(use) < 20:
        return None

    dts = sorted(d for d in (_f(r, "dt") for r in use) if d)
    ts = dts[len(dts) // 2]

    # Consecutive pairs from the same run, both at a sane dt.
    xs, ys, dropped = [], [], 0
    for cur, nxt in zip(use, use[1:]):
        if cur.get("run_id") != nxt.get("run_id"):
            continue
        y0, y1, u1 = _f(cur, y_col), _f(nxt, y_col), _f(nxt, "duty")
        d = _f(nxt, "dt")
        if None in (y0, y1, u1, d):
            continue
        if abs(d - ts) > dt_tol * ts:
            dropped += 1
            continue
        xs.append((y0, u1, 1.0))
        ys.append(y1)

    if len(xs) < 20:
        return None
    a, b, c = _lstsq3(xs, ys)

    if not 0.0 < a < 1.0:
        return {"error": f"a={a:.4f} fora de (0,1): a planta nao se parece com "
                         f"primeira ordem estavel nesses dados", "a": a}
    tau = -ts / math.log(a)
    k   = b / (1.0 - a)
    u0  = (-c / b) if abs(b) > 1e-15 else float("nan")

    # One-step residuals.
    pred = [a * x[0] + b * x[1] + c for x in xs]
    ybar = sum(ys) / len(ys)
    sse  = sum((p - y) ** 2 for p, y in zip(pred, ys))
    sst  = sum((y - ybar) ** 2 for y in ys)
    r2   = 1.0 - sse / sst if sst > 0 else float("nan")

    return {"a": a, "b": b, "c": c, "ts": ts, "tau": tau, "K": k, "u0": u0,
            "n": len(xs), "dropped": dropped, "r2": r2,
            "rmse": math.sqrt(sse / len(ys)),
            "runs": sorted({r.get("run_id", "") for r in use})}


def free_run_fit(rows, y_col, phase, model):
    """Simulate the model open-loop over each run and score it against the
    measurement. This is the honest test: one-step R^2 flatters any model with a
    slow pole, because y[k] alone already predicts y[k+1] well. A free run gets
    no measured feedback after the first sample.

    Returns the standard NRMSE fit percentage, 100 = perfect.
    """
    a, b, c = model["a"], model["b"], model["c"]
    per_run, meas_all, sim_all = {}, [], []
    for r in rows:
        if r.get("phase") != phase:
            continue
        per_run.setdefault(r.get("run_id"), []).append(r)

    for run, rs in per_run.items():
        y = _f(rs[0], y_col)
        if y is None:
            continue
        for cur, nxt in zip(rs, rs[1:]):
            u, ym = _f(nxt, "duty"), _f(nxt, y_col)   # same indexing as the fit
            if u is None or ym is None:
                continue
            y = a * y + b * u + c
            sim_all.append(y)
            meas_all.append(ym)

    if len(meas_all) < 10:
        return None
    ybar = sum(meas_all) / len(meas_all)
    num = math.sqrt(sum((m - s) ** 2 for m, s in zip(meas_all, sim_all)))
    den = math.sqrt(sum((m - ybar) ** 2 for m in meas_all))
    return 100.0 * (1.0 - num / den) if den > 0 else float("nan")


# ---------------------------------------------------------------------------
# Pole placement
# ---------------------------------------------------------------------------
def gains_for_separation(gamma, tau, zeta, wn, separation):
    """(k_p, k_i, k_d, p_3) for a third pole at separation*zeta*omega_n.

    Parameterising the search by SEPARATION rather than by k_d directly keeps it
    numerically sane: separation is dimensionless, so one grid works for any
    plant, and p_3 > 0 is guaranteed by construction instead of having to be
    policed (p_3 <= 0 flips the sign of k_i).
    """
    p3 = separation * zeta * wn
    kd = (tau * (2.0 * zeta * wn + p3) - 1.0) / gamma
    kp = (tau / gamma) * (wn ** 2 + 2.0 * zeta * wn * p3)
    ki = (tau / gamma) * (wn ** 2 * p3)
    return kp, ki, kd, p3


def simulate_step(gamma, tau, kp, ki, kd, p_on_measurement=True,
                  step_m=1.0, horizon=8.0, fast_pole=None):
    """Closed-loop unit step. Returns (overshoot %, settling s, rise s, |u| peak).

    `p_on_measurement` selects the loop structure: True feeds P from the
    measurement (setpoint weighting b = 0), which leaves only k_i in the closed
    loop numerator; False is textbook P-on-error, which puts a zero at -k_i/k_p
    in the closed loop. D always comes off the measurement -- that is what
    pid.py does in effect for a constant setpoint, since d(error)/dt =
    -d(measured)/dt and the first-sample guard kills the derivative kick.
    """
    fast = fast_pole or max(1.0 / tau, kp / max(kd, 1e-9))
    dt = min(1e-3, 1.0 / (50.0 * fast), tau / 50.0)
    n = int(horizon / dt)

    x = v = ei = 0.0
    peak, umax, ts, tr = 0.0, 0.0, 0.0, float("nan")
    band = 0.01 * abs(step_m)
    for i in range(n):
        e = step_m - x
        ei += e * dt
        u = (-kp * x if p_on_measurement else kp * e) + ki * ei - kd * v
        umax = max(umax, abs(u))
        v += (gamma * u - v) / tau * dt
        x += v * dt
        peak = max(peak, x)
        if tr != tr and x >= step_m:
            tr = i * dt
        if abs(x - step_m) > band:
            ts = (i + 1) * dt
    po = 100.0 * (peak - step_m) / abs(step_m)
    return po, ts, tr, umax


def choose_kd(gamma, tau, zeta, wn, spec, p_on_measurement=True,
              step_m=1.0, tol=0.05, max_duty=None, grid=48):
    """Pick k_d: the SMALLEST value whose simulated step meets the spec.

    "Best" has to be defined, because the spec (zeta, omega_n) only fixes the
    dominant pair -- p_3, and therefore k_d, is genuinely free. The criterion
    used here is that everything above the smallest satisfying k_d is paid for
    and not delivered: k_p, k_i, derivative noise gain (k_d * quantisation) and
    peak duty demand all grow monotonically with k_d, while the response stops
    improving once the third pole is out of the way. So: cheapest design that
    still honours OVERSHOOT / SETTLING / RISE.

    Optionally also requires the peak duty to stay under `max_duty` for a move
    of `step_m` -- a design that saturates is not the design that was computed.

    When nothing satisfies everything, returns the separation that minimises the
    worst relative violation, and names the binding constraint, rather than
    silently handing back a number that misses the spec.
    """
    po_t, ts_t, tr_t = spec["po"], spec["ts"], spec["tr"]
    horizon = max(8.0, 4.0 * ts_t)

    best_ok, best_any, best_score = None, None, float("inf")
    for i in range(grid):
        sep = 0.5 * (200.0 / 0.5) ** (i / (grid - 1.0))     # 0.5 .. 200, log
        kp, ki, kd, p3 = gains_for_separation(gamma, tau, zeta, wn, sep)
        po, ts, tr, umax = simulate_step(gamma, tau, kp, ki, kd,
                                         p_on_measurement, step_m, horizon, p3)
        viol = {"overshoot": po / po_t - 1.0,
                "settling":  ts / ts_t - 1.0,
                "rise":      (tr / tr_t - 1.0) if tr == tr else float("inf")}
        if max_duty:
            viol["duty"] = umax / max_duty - 1.0
        worst = max(viol.values())
        rec = {"sep": sep, "k_p": kp, "k_i": ki, "k_d": kd, "p_3": p3,
               "po": po, "ts": ts, "tr": tr, "umax": umax,
               "binding": max(viol, key=viol.get), "worst": worst}
        if worst <= tol and best_ok is None:
            best_ok = rec                       # grid ascends -> first is smallest
        if worst < best_score:
            best_score, best_any = worst, rec

    if best_ok is None:
        best_any["feasible"] = False
        return best_any

    # Refine: bisect between the last failing separation and the first passing
    # one, so the answer is the boundary rather than a grid point.
    lo, hi = 0.5, best_ok["sep"]
    for _ in range(30):
        mid = math.sqrt(lo * hi)
        kp, ki, kd, p3 = gains_for_separation(gamma, tau, zeta, wn, mid)
        po, ts, tr, umax = simulate_step(gamma, tau, kp, ki, kd,
                                         p_on_measurement, step_m, horizon, p3)
        viol = [po / po_t - 1.0, ts / ts_t - 1.0,
                (tr / tr_t - 1.0) if tr == tr else float("inf")]
        if max_duty:
            viol.append(umax / max_duty - 1.0)
        if max(viol) <= tol:
            hi, best_ok = mid, {"sep": mid, "k_p": kp, "k_i": ki, "k_d": kd,
                                "p_3": p3, "po": po, "ts": ts, "tr": tr,
                                "umax": umax, "binding": None, "worst": max(viol)}
        else:
            lo = mid
    best_ok["feasible"] = True
    return best_ok


def pid_from_plant(k, tau, zeta, wn, kd=None, wheel_scale=WHEEL_SCALE,
                   spec=None, p_on_measurement=True, step_m=1.0,
                   tol=0.05, max_duty=None):
    """PID gains for gamma/(s*(tau*s+1)) by placing a dominant pair + p_3.

    k_d given -> p_3 follows from it. k_d omitted -> chosen by `choose_kd`.
    """
    gamma = k * wheel_scale
    if gamma == 0.0:
        raise ValueError("gamma = 0")

    kd_min = (2.0 * zeta * wn * tau - 1.0) / gamma      # p_3 = 0
    chosen = None
    if kd is None:
        chosen = choose_kd(gamma, tau, zeta, wn, spec, p_on_measurement,
                           step_m, tol, max_duty)
        kd, p3 = chosen["k_d"], chosen["p_3"]
        kp, ki = chosen["k_p"], chosen["k_i"]
    else:
        p3 = (gamma * kd + 1.0) / tau - 2.0 * zeta * wn
        kp = (tau / gamma) * (wn ** 2 + 2.0 * zeta * wn * p3)
        ki = (tau / gamma) * (wn ** 2 * p3)

    return {"gamma": gamma, "k_p": kp, "k_i": ki, "k_d": kd,
            "p_3": p3, "kd_min": kd_min, "chosen": chosen,
            "separation": (p3 / (zeta * wn)) if zeta * wn else float("nan")}


def verify_closed_loop(gamma, tau, g, zeta, wn, step_m=1.0):
    """Simulate the designed loop and report what it ACTUALLY does.

    Pole placement fixes the DENOMINATOR, but a PID in the forward path also
    puts zeros in the closed loop:

        T(s) = gamma*(k_d*s^2 + k_p*s + k_i) / (tau*s^3 + ... )

    and zeta-from-overshoot assumes a second order system with NO zeros. So the
    placement alone does not deliver the overshoot spec. The PI zero sits at
    -k_i/k_p; when that lands inside the dominant pole band it dominates the
    transient. On the reference design here it turned a 5% spec into 29%.

    Two ways out, both simulated below:
      * feed P (and D) from the MEASUREMENT rather than the error -- setpoint
        weighting b = 0 -- which leaves only k_i in the numerator;
      * or pre-filter the reference with (k_i/k_p)/(s + k_i/k_p), cancelling
        the zero before it reaches the loop.

    pid.py already takes D off the measurement in effect: for the constant
    setpoint of a drive_distance move, d(error)/dt = -d(measured)/dt, and the
    first-sample guard suppresses the derivative kick.
    """
    kp, ki, kd = g["k_p"], g["k_i"], g["k_d"]
    T = max(8.0, 5.0 * 4.6 / (zeta * wn))
    a = simulate_step(gamma, tau, kp, ki, kd, False, step_m, T, g["p_3"])
    b = simulate_step(gamma, tau, kp, ki, kd, True,  step_m, T, g["p_3"])
    return {"zero": (-ki / kp) if kp else float("nan"),
            "as_is": a, "b0": b}


# ---------------------------------------------------------------------------
# Report
# ---------------------------------------------------------------------------
def _report_plant(name, model, free_fit):
    print(f"\n=== {name} ===")
    if model is None:
        print("  sem dados suficientes -- rode gray_regression.py primeiro")
        return False
    if "error" in model:
        print(f"  {model['error']}")
        return False
    print(f"  corridas    : {', '.join(model['runs'])}")
    print(f"  amostras    : {model['n']}  (descartadas por jitter de dt: "
          f"{model['dropped']})")
    print(f"  Ts          : {model['ts'] * 1000:.1f} ms")
    print(f"  K           : {model['K']:.6g}")
    print(f"  tau         : {model['tau']:.4f} s")
    print(f"  u0 (zona morta): {model['u0']:.0f} duty")
    print(f"  R^2 1-passo : {model['r2']:.4f}   RMSE {model['rmse']:.5g}")
    if free_fit is not None:
        print(f"  fit livre   : {free_fit:.1f}%   <- o numero que importa")
        if free_fit < 70.0:
            print("  AVISO: abaixo de ~70% o modelo nao explica os dados; "
                  "os ganhos abaixo nao valem.")
    return True


def _report_gains(label, g, zeta, wn, kd_arg):
    print(f"  --- ganhos {label} ---")
    print(f"  gamma  = {g['gamma']:.6g}")
    c = g.get("chosen")
    if kd_arg is not None:
        print(f"  k_d    = {g['k_d']:.6g}   (fornecido)")
    elif c is None:
        print(f"  k_d    = {g['k_d']:.6g}")
    elif c["feasible"]:
        print(f"  k_d    = {g['k_d']:.6g}   (MENOR que cumpre a especificacao)")
        print(f"           simulado: PO {c['po']:.2f}%  ts {c['ts']:.2f}s  "
              f"tr {c['tr']:.2f}s  |u|pico {c['umax']:.0f}")
    else:
        print(f"  k_d    = {g['k_d']:.6g}   (ESPECIFICACAO INATINGIVEL -- "
              f"melhor esforco)")
        print(f"           simulado: PO {c['po']:.2f}%  ts {c['ts']:.2f}s  "
              f"tr {c['tr']:.2f}s  |u|pico {c['umax']:.0f}")
        print(f"           restricao que impede: {c['binding']} "
              f"(excedida em {100 * c['worst']:.0f}%)")
    print(f"  k_p    = {g['k_p']:.6g}")
    print(f"  k_i    = {g['k_i']:.6g}")
    print(f"  p_3    = {g['p_3']:.4g} rad/s  (separacao "
          f"{g['separation']:.1f}x o par dominante)")
    if g["p_3"] <= 0.0:
        print(f"  ERRO: p_3 <= 0 -> k_i negativo, projeto instavel. "
              f"k_d precisa ser > {g['kd_min']:.6g}")
    elif g["separation"] < 3.0:
        print(f"  AVISO: separacao < 3x -- o terceiro polo interfere e a "
              f"especificacao de sobressinal/acomodacao nao vai valer.")


def main():
    ap = argparse.ArgumentParser(
        description="Ajusta os modelos e deriva ganhos PID dos CSVs do "
                    "gray_regression.py")
    ap.add_argument("csv", nargs="*",
                    help="CSVs (padrao: Scripts/data/gray_*.csv)")
    ap.add_argument("--overshoot", type=float, default=OVERSHOOT_PCT,
                    help=f"PO em %% (padrao {OVERSHOOT_PCT})")
    ap.add_argument("--settling",  type=float, default=SETTLING_TIME_S,
                    help=f"tempo de acomodacao em s (padrao {SETTLING_TIME_S})")
    ap.add_argument("--rise",      type=float, default=RISE_TIME_S,
                    help=f"tempo de subida em s (padrao {RISE_TIME_S})")
    ap.add_argument("--kd",        type=float, default=None,
                    help="k_d da malha de distancia (padrao: menor que cumpre "
                         "a especificacao)")
    ap.add_argument("--kd-heading", type=float, default=None,
                    help="k_d da malha de heading (idem)")
    ap.add_argument("--move",      type=float, default=1.0,
                    help="tamanho do degrau simulado, m ou rad (padrao 1.0). "
                         "A demanda de duty escala com ele.")
    ap.add_argument("--max-duty",  type=float, default=None,
                    help="exige |u| de pico abaixo disto na escolha do k_d "
                         f"(ex.: {CONFIG.position.output_limit:g})")
    ap.add_argument("--spec-tol",  type=float, default=0.05,
                    help="folga relativa aceita na especificacao (padrao 0.05)")
    ap.add_argument("--p-on-error", action="store_true",
                    help="dimensiona para P no erro (como o pid.py esta hoje) "
                         "em vez de P na medicao")
    ap.add_argument("--wheel-scale", type=float, default=WHEEL_SCALE,
                    help="gamma = K * este fator. 1.0 quando K ja e linear "
                         f"(padrao); use {CONFIG.wheel.diameter} para seguir "
                         "a especificacao gamma = K*diametro ao pe da letra")
    args = ap.parse_args()

    paths = args.csv or sorted(glob.glob(os.path.join(DATA_DIR, "gray_*.csv")))
    if not paths:
        print(f"nenhum CSV em {DATA_DIR}")
        return 1
    rows = load_rows(paths)
    print(f"[csv] {len(paths)} arquivo(s), {len(rows)} linhas")

    zeta, wn, wn_ts, wn_tr = spec_to_poles(args.overshoot, args.settling,
                                           args.rise)
    ts_hit, tr_hit = achieved(zeta, wn)
    print(f"\n=== especificacao ===")
    print(f"  PO {args.overshoot:g}%  ->  zeta = {zeta:.4f}")
    print(f"  omega_n por acomodacao ({args.settling:g} s): {wn_ts:.3f} rad/s")
    print(f"  omega_n por subida     ({args.rise:g} s): {wn_tr:.3f} rad/s")
    print(f"  omega_n adotado        : {wn:.3f} rad/s  "
          f"({'acomodacao' if wn == wn_ts else 'subida'} e a restricao ativa)")
    print(f"  omega_d                : {damped(wn, zeta):.3f} rad/s")
    print(f"  atingido               : acomodacao {ts_hit:.2f} s, "
          f"subida {tr_hit:.2f} s")
    if abs(wn_ts - wn_tr) / max(wn_ts, wn_tr) > 0.25:
        print("  NOTA: as duas especificacoes divergem >25%. A mais folgada "
              "sera superada com folga.")

    snippet = []
    for label, y_col, phase, kd_arg in (
            ("DISTANCIA  (v_center, phase=prbs)",      "v_center", "prbs",
             args.kd),
            ("HEADING    (w_gyro, phase=prbs_head)",   "w_gyro",   "prbs_head",
             args.kd_heading)):
        model = fit_first_order(rows, y_col, phase)
        ff = free_run_fit(rows, y_col, phase, model) if model and "error" not in model else None
        if not _report_plant(label, model, ff):
            continue
        try:
            g = pid_from_plant(model["K"], model["tau"], zeta, wn,
                               kd=kd_arg, wheel_scale=args.wheel_scale,
                               spec={"po": args.overshoot, "ts": ts_hit,
                                     "tr": tr_hit},
                               p_on_measurement=not args.p_on_error,
                               step_m=args.move, tol=args.spec_tol,
                               max_duty=args.max_duty)
        except ValueError as e:
            print(f"  {e}")
            continue
        _report_gains(label.split()[0].lower(), g, zeta, wn, kd_arg)

        v = verify_closed_loop(g["gamma"], model["tau"], g, zeta, wn, args.move)
        po_a, ts_a, tr_a, um_a = v["as_is"]
        po_b, ts_b, tr_b, um_b = v["b0"]
        print(f"  --- degrau simulado (alvo PO {args.overshoot:g}%, "
              f"ts {ts_hit:.2f}s, tr {tr_hit:.2f}s) ---")
        print(f"  zero do PI em {v['zero']:.3f} rad/s "
              f"(polos dominantes |s| = {wn:.3f})")
        print(f"  P no erro     : PO {po_a:6.2f}%  ts {ts_a:.2f}s  tr {tr_a:.2f}s  |u| {um_a:.0f}")
        print(f"  P na medicao  : PO {po_b:6.2f}%  ts {ts_b:.2f}s  tr {tr_b:.2f}s  |u| {um_b:.0f}")
        if po_a > 1.5 * args.overshoot:
            print(f"  ATENCAO: com P no erro o sobressinal e {po_a / args.overshoot:.1f}x "
                  f"a especificacao. O zero de malha fechada em {v['zero']:.2f} rad/s "
                  f"cai dentro da banda dos polos -- a formula zeta<-PO supoe zero "
                  f"nenhum. Use P na medicao (setpoint weighting b=0) ou pre-filtre "
                  f"a referencia com (ki/kp)/(s + ki/kp).")
        snippet.append((label.split()[0], g, model))

    if snippet:
        print("\n=== para o config.py ===")
        for name, g, m in snippet:
            cls = "PositionGains" if name == "DISTANCIA" else "HeadingGains"
            floor = "min_move_duty" if name == "DISTANCIA" else "min_turn_duty"
            print(f"\n# {cls}  (tau={m['tau']:.4f}s  K={m['K']:.6g})")
            print(f"kp: float = {g['k_p']:.1f}")
            print(f"ki: float = {g['k_i']:.1f}")
            print(f"kd: float = {g['k_d']:.1f}")
            print(f"{floor}: float = {m['u0']:.0f}   # zona morta medida")
        print("\nNao cole direto: o teto de duty (PositionGains.output_limit = "
              f"{CONFIG.position.output_limit:g}) existe porque o encoder perde "
              "contagem acima de ~3000, e limita a banda que estes ganhos "
              "conseguem entregar. Valide em hardware.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
