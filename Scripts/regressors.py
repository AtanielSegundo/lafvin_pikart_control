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

IDENTIFICATION (see `fit_first_order`)
--------------------------------------
The sampling is NOT uniform: the acquisition loop shares a Pi with the pigpio
encoder callbacks and the I2C bus, and its dt wanders (20 -> 80 ms in the
reference data). So the model is discretised PER SAMPLE with that row's own dt,

    y[k] = a_k*y[k-1] + (1 - a_k)*(K*u[k] + d),   a_k = exp(-dt_k / tau)

which is the EXACT zero-order-hold solution over an interval of any length.
Nothing has to be resampled and no row has to be thrown away for being late:
a stretched interval simply carries less information about tau (a_k -> 0) and
the fit weighs it accordingly. `--allow-degenerated` turns off the remaining
sanity filter entirely.

Given tau the model is LINEAR in (K, d = -K*u0), so tau is found by a 1-D search
with a 2-parameter least squares inside it (separable least squares). Two
objectives are available:

  * `--method oe` (default) -- OUTPUT ERROR: minimise the error of a FREE RUN,
    the model driven by u alone from one initial condition. This is the honest
    objective and, more importantly, it is unbiased when the OUTPUT is noisy.
  * `--method arx` -- one-step-ahead prediction error. Cheap and classic, but
    measurement noise on y[k-1] sits on the RIGHT-hand side and biases `a`
    DOWNWARD, i.e. tau collapses toward zero. On the reference data (v_center
    quantised at 4.4 mm/s and corrupted by encoder bursts) ARX returns
    tau = 0.03 s and a NEGATIVE K; OE on the same rows does not.

`--robust` adds Huber IRLS on top, so a handful of encoder-burst outliers stop
dominating the sum of squares.

DEAD ZONE
---------
A binary PRBS visits exactly TWO duty levels. Two points fix a line, so u0 is a
long EXTRAPOLATION from the operating point down to the axis crossing, and any
small slope error explodes it (the reference run extrapolates from [1900, 3100]
and lands on u0 = 13383). Pin the measured value with `--u0` / `--u0-heading`
instead -- that is what the `static` / `kinetic` experiments are for -- and K is
then fit with a single free parameter.

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

No third-party dependencies: everything is plain-Python least squares, so this
runs on the Pi or on a laptop.

    python3 Scripts/regressors.py
    python3 Scripts/regressors.py op_1_prbs.csv op_1_prbs_heading.csv
    python3 Scripts/regressors.py --allow-degenerated --robust --u0 1200
    python3 Scripts/regressors.py --diagnose-only        # just audit the data
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

# Drop a sample whose dt exceeds this many times the run's median dt. With the
# per-sample discretisation a late row is no longer WRONG, only uninformative,
# so this is a sanity net (a 20x interval is a crash, not a schedule hiccup),
# not the tight 20% guard a uniform-Ts fit needed. --allow-degenerated -> inf.
DT_MAX_FACTOR = 6.0

# tau search bracket, seconds. Wide enough for anything this drivetrain can be.
TAU_LO, TAU_HI = 0.005, 5.0


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
# Linear algebra: weighted least squares of any (small) order
# ---------------------------------------------------------------------------
def _solve(A, b):
    """Gaussian elimination with partial pivoting on an n x n system."""
    n = len(b)
    M = [list(A[i]) + [b[i]] for i in range(n)]
    for col in range(n):
        piv = max(range(col, n), key=lambda r: abs(M[r][col]))
        if abs(M[piv][col]) < 1e-18:
            raise ValueError("sistema singular -- dados insuficientes ou "
                             "sem excitacao (duty constante?)")
        M[col], M[piv] = M[piv], M[col]
        for r in range(col + 1, n):
            f = M[r][col] / M[col][col]
            for c in range(col, n + 1):
                M[r][c] -= f * M[col][c]
    x = [0.0] * n
    for r in range(n - 1, -1, -1):
        x[r] = (M[r][n] - sum(M[r][c] * x[c] for c in range(r + 1, n))) / M[r][r]
    return x


def _wlstsq(rows_x, rows_y, weights=None):
    """Weighted least squares for y = x . theta. Returns theta."""
    n = len(rows_x[0])
    A = [[0.0] * n for _ in range(n)]
    b = [0.0] * n
    for i, (x, y) in enumerate(zip(rows_x, rows_y)):
        w = 1.0 if weights is None else weights[i]
        if w == 0.0:
            continue
        for p in range(n):
            b[p] += w * x[p] * y
            for q in range(n):
                A[p][q] += w * x[p] * x[q]
    return _solve(A, b)


def _median(vals):
    s = sorted(vals)
    n = len(s)
    if not n:
        return float("nan")
    return s[n // 2] if n % 2 else 0.5 * (s[n // 2 - 1] + s[n // 2])


def _pct(vals, p):
    s = sorted(vals)
    if not s:
        return float("nan")
    return s[min(len(s) - 1, int(p * len(s)))]


# ---------------------------------------------------------------------------
# CSV loading
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


def add_w_from_yaw(rows):
    """Rebuild `w_gyro` by differencing the logged `yaw` over the row's own dt.

    gray_regression differences yaw between two DISTINCT gyro readings and
    divides by the span measured on the CONTROL thread's clock. GyroMPU
    integrates on its own thread, so that span is the observation interval, not
    the integration interval -- a +-10 ms uncertainty on a 20 ms window, i.e.
    +-50% noise on every w, plus blank rows whenever the gyro thread had not
    ticked yet.

    Differencing over the logged interval instead gives the AVERAGE rate across
    exactly the interval the ZOH model integrates over -- which is what the fit
    wants -- on a longer, known baseline, and it fills every row.

    Overwrites w_gyro in place; rows that cannot be differenced are blanked.
    """
    filled = 0
    prev = None                          # (key, t, yaw) of the previous row
    for r in rows:
        key = (r.get("_src"), r.get("run_id"))
        yaw, t = _f(r, "yaw"), _f(r, "t")
        if yaw is None or t is None:
            r["w_gyro"] = ""
            prev = None                  # break the chain across the hole
            continue
        span = (t - prev[1]) if (prev is not None and prev[0] == key) else 0.0
        if span > 0.0:
            r["w_gyro"] = f"{(yaw - prev[2]) / span:.6f}"
            filled += 1
        else:
            r["w_gyro"] = ""             # first row of a run: no baseline yet
        prev = (key, t, yaw)
    return filled


# ---------------------------------------------------------------------------
# Segments: contiguous stretches of one run/phase, with their own dt per sample
# ---------------------------------------------------------------------------
class Segment:
    """One contiguous stretch of samples from a single run and phase.

    `dt[k]` is the length of the interval that ENDED at sample k, `u[k]` the
    duty in force during it and `y[k]` the output at the end of it -- exactly
    how gray_regression writes a row ("the row describes the interval that just
    ended"). So the recursion driving y[k] uses dt[k] and u[k], never u[k-1];
    pairing y[k] with u[k-1] shifts the input one sample late and the fit
    absorbs it as a slower pole.
    """

    __slots__ = ("run", "src", "t", "dt", "u", "y")

    def __init__(self, run, src):
        self.run, self.src = run, src
        self.t, self.dt, self.u, self.y = [], [], [], []

    def __len__(self):
        return len(self.y)

    def add(self, t, dt, u, y):
        self.t.append(t)
        self.dt.append(dt)
        self.u.append(u)
        self.y.append(y)


def build_segments(rows, y_col, phase, dt_max_factor=DT_MAX_FACTOR,
                   min_len=8, clip_abs=None):
    """Split the matching rows into fittable segments.

    A segment BREAKS on a change of run/file/phase, or on a missing y or u.

    A rejected sample (bad |y|, or an absurd dt) does NOT have to break it. The
    discretisation is per-sample, so the interval a dropped sample occupied can
    simply be ADDED to the next accepted one -- provided the duty did not change
    across the gap, which is exactly the zero-order-hold assumption the model
    already makes. That is what `bridged` counts. Without this, rejecting half
    the samples of a corrupted run leaves only fragments too short to identify
    anything, and a legitimate cleaning step destroys the record it was meant to
    rescue.

    When the duty DID change across the gap the input is no longer piecewise
    constant over the merged interval, so the segment breaks after all.

    `clip_abs` drops samples whose |y| exceeds it -- for rejecting physically
    impossible readings (a 2 m/s v_center on a 0.6 m/s kart) rather than jitter.
    """
    use = [r for r in rows if r.get("phase") == phase]
    stats = {"n_rows": len(use), "dropped_dt": 0, "dropped_clip": 0,
             "dropped_nan": 0, "bridged": 0, "ts": float("nan")}
    if not use:
        return [], stats

    dts = [d for d in (_f(r, "dt") for r in use) if d and d > 0]
    if not dts:
        return [], stats
    ts = _median(dts)
    stats["ts"] = ts
    dt_cap = ts * dt_max_factor if dt_max_factor and dt_max_factor > 0 else float("inf")

    segs, cur, key_prev = [], None, None
    pending_dt, pending_u = 0.0, None      # interval carried over a bad sample

    def flush():
        nonlocal cur, pending_dt, pending_u
        if cur is not None and len(cur) >= min_len:
            segs.append(cur)
        cur = None
        pending_dt, pending_u = 0.0, None

    def skip(dt, u):
        """Hold a rejected sample's interval so the next good one absorbs it."""
        nonlocal pending_dt, pending_u
        if pending_u is not None and u != pending_u:
            flush()                        # duty changed inside the gap
            return
        pending_dt += dt
        pending_u = u

    for r in use:
        key = (r.get("_src"), r.get("run_id"))
        y, u, dt = _f(r, y_col), _f(r, "duty"), _f(r, "dt")
        t = _f(r, "t")
        if key != key_prev:
            flush()
            key_prev = key
        if y is None or u is None or dt is None or dt <= 0 or t is None:
            stats["dropped_nan"] += 1
            flush()                        # no duty known: nothing to bridge
            continue
        if dt > dt_cap:
            stats["dropped_dt"] += 1
            skip(dt, u)
            continue
        if clip_abs is not None and abs(y) > clip_abs:
            stats["dropped_clip"] += 1
            skip(dt, u)
            continue
        if pending_dt > 0.0:
            if pending_u == u:
                dt += pending_dt           # merge: one ZOH step, same input
                stats["bridged"] += 1
            else:
                flush()
            pending_dt, pending_u = 0.0, None
        if cur is None:
            cur = Segment(r.get("run_id", ""), r.get("_src", ""))
        cur.add(t, dt, u, y)
    flush()
    return segs, stats


# ---------------------------------------------------------------------------
# Separable least squares: linear in (K, d) for any fixed tau
# ---------------------------------------------------------------------------
def _basis_oe(seg, tau):
    """Free-run basis for one segment.

    y_sim[k] = K*phi1[k] + d*phi2[k] + phi0[k], where phi0 carries the initial
    condition. Because the recursion is linear, simulating the two unit inputs
    once lets a plain 2-parameter least squares minimise the SIMULATION error
    exactly -- no iteration over (K, d) needed.
    """
    phi1 = phi2 = 0.0
    phi0 = seg.y[0]
    X, Y = [], []
    for k in range(1, len(seg)):
        a = math.exp(-seg.dt[k] / tau)
        om = 1.0 - a
        phi1 = a * phi1 + om * seg.u[k]
        phi2 = a * phi2 + om
        phi0 = a * phi0
        X.append((phi1, phi2))
        Y.append(seg.y[k] - phi0)
    return X, Y


def _basis_arx(seg, tau):
    """One-step-ahead basis: y[k] - a*y[k-1] = (1-a)*(K*u[k] + d)."""
    X, Y = [], []
    for k in range(1, len(seg)):
        a = math.exp(-seg.dt[k] / tau)
        om = 1.0 - a
        X.append((om * seg.u[k], om))
        Y.append(seg.y[k] - a * seg.y[k - 1])
    return X, Y


def _fit_at_tau(segs, tau, method, weights=None, u0_fixed=None):
    """Least squares for (K, d) at a fixed tau. Returns (sse, K, d, n, resid)."""
    basis = _basis_oe if method == "oe" else _basis_arx
    X, Y = [], []
    for seg in segs:
        x, y = basis(seg, tau)
        X.extend(x)
        Y.extend(y)
    if len(X) < 4:
        return float("inf"), 0.0, 0.0, 0, []

    if u0_fixed is None:
        theta = _wlstsq(X, Y, weights)
        k, d = theta[0], theta[1]
    else:
        # d = -K*u0 with u0 pinned: one free parameter, hugely better
        # conditioned than extrapolating the axis crossing from two duty levels.
        Xr = [(x[0] - u0_fixed * x[1],) for x in X]
        k = _wlstsq(Xr, Y, weights)[0]
        d = -k * u0_fixed

    resid = [y - (k * x[0] + d * x[1]) for x, y in zip(X, Y)]
    if weights is None:
        sse = sum(r * r for r in resid)
    else:
        sse = sum(w * r * r for w, r in zip(weights, resid))
    return sse, k, d, len(X), resid


def _search_tau(segs, method, weights=None, u0_fixed=None,
                lo=TAU_LO, hi=TAU_HI, grid=72):
    """1-D search for tau: log grid, then golden-section refine on the best cell.

    Grid first because the SSE(tau) curve is not always unimodal on real data --
    a pure descent from one seed can settle in the wrong basin.
    """
    best = None
    taus = [lo * (hi / lo) ** (i / (grid - 1.0)) for i in range(grid)]
    scores = []
    for tau in taus:
        sse = _fit_at_tau(segs, tau, method, weights, u0_fixed)[0]
        scores.append(sse)
        if best is None or sse < best[0]:
            best = (sse, tau)
    i = scores.index(best[0])
    a = taus[max(0, i - 1)]
    b = taus[min(len(taus) - 1, i + 1)]

    # Golden-section on the bracketing cell (in log tau, so the step is
    # proportional -- the same absolute step means very different things at
    # tau = 0.01 and tau = 1).
    inv = (math.sqrt(5.0) - 1.0) / 2.0
    la, lb = math.log(a), math.log(b)
    lc, ld = lb - inv * (lb - la), la + inv * (lb - la)
    fc = _fit_at_tau(segs, math.exp(lc), method, weights, u0_fixed)[0]
    fd = _fit_at_tau(segs, math.exp(ld), method, weights, u0_fixed)[0]
    for _ in range(48):
        if fc < fd:
            lb, ld, fd = ld, lc, fc
            lc = lb - inv * (lb - la)
            fc = _fit_at_tau(segs, math.exp(lc), method, weights, u0_fixed)[0]
        else:
            la, lc, fc = lc, ld, fd
            ld = la + inv * (lb - la)
            fd = _fit_at_tau(segs, math.exp(ld), method, weights, u0_fixed)[0]
        if abs(lb - la) < 1e-6:
            break
    tau = math.exp(0.5 * (la + lb))
    sse = _fit_at_tau(segs, tau, method, weights, u0_fixed)[0]
    if sse > best[0]:                     # refinement never wins -> keep the grid
        tau, sse = best[1], best[0]
    return tau, sse


def _huber_weights(resid, c=1.5):
    """Huber IRLS weights from the MAD scale. 1 inside the bulk, 1/|z| outside."""
    med = _median(resid)
    mad = _median([abs(r - med) for r in resid]) * 1.4826
    if mad <= 0.0:
        return [1.0] * len(resid)
    out = []
    for r in resid:
        z = abs(r - med) / mad
        out.append(1.0 if z <= c else c / z)
    return out


def fit_first_order(rows, y_col, phase, method="oe", robust=False,
                    dt_max_factor=DT_MAX_FACTOR, u0_fixed=None, clip_abs=None):
    """Fit tau*y_dot + y = K*(u - u0) with a per-sample ZOH discretisation.

    Returns a dict, or None when there is not enough usable data.
    """
    segs, stats = build_segments(rows, y_col, phase,
                                 dt_max_factor=dt_max_factor, clip_abs=clip_abs)
    n_pts = sum(len(s) - 1 for s in segs)
    if n_pts < 20:
        return None

    weights = None
    for _ in range(4 if robust else 1):
        tau = _search_tau(segs, method, weights, u0_fixed)[0]
        sse, k, d, n, resid = _fit_at_tau(segs, tau, method, weights, u0_fixed)
        if not robust:
            break
        weights = _huber_weights(resid)

    if not (TAU_LO * 1.01 < tau < TAU_HI * 0.99):
        return {"error": f"tau={tau:.4g} s bateu no limite da busca "
                         f"[{TAU_LO}, {TAU_HI}] -- a planta nao se parece com "
                         f"primeira ordem estavel nesses dados "
                         f"({n_pts} amostras uteis em {len(segs)} trecho(s), "
                         f"de {stats['n_rows']} linhas)",
                "tau": tau, "stats": stats}
    if abs(k) < 1e-12:
        return {"error": "K = 0 -- sem excitacao util", "stats": stats}

    u0 = -d / k
    a_med = math.exp(-stats["ts"] / tau)

    # Both scores, always, whichever objective was optimised -- they measure
    # different things and the gap between them is itself diagnostic.
    r2 = _score_onestep(segs, tau, k, d)
    ff = _score_freerun(segs, tau, k, d)
    snr = estimate_snr(segs)

    duties = sorted({u for s in segs for u in s.u})
    u_lo, u_hi = (duties[0], duties[-1]) if duties else (0.0, 0.0)
    rmse = math.sqrt(sse / max(1, n))
    n_out = sum(1 for w in (weights or []) if w < 0.999)

    return {"tau": tau, "K": k, "d": d, "u0": u0, "a_med": a_med,
            "ts": stats["ts"], "n": n, "n_seg": len(segs),
            "method": method, "robust": robust, "n_outliers": n_out,
            "r2": r2, "free_fit": ff, "snr_db": snr, "rmse": rmse, "sse": sse,
            "u_lo": u_lo, "u_hi": u_hi, "duty_levels": duties,
            "u0_fixed": u0_fixed,
            "u0_sane": (u0_fixed is not None) or (0.0 <= u0 <= u_lo),
            "stats": stats,
            "runs": sorted({s.run for s in segs})}


def estimate_snr(segs):
    """Rough output SNR of the measurement, in dB, from its lag-1 difference.

    A first-order plant with tau >> Ts is SMOOTH over one sample, so almost all
    of y[k] - y[k-1] is measurement noise, and var(diff)/2 estimates the noise
    power. Reported because the free-run fit percentage is as much an SNR
    measure as a model-quality one: on synthetic data with the plant known
    EXACTLY, sigma = 0.05 on a 0.096 signal swing still scores only 8%. Without
    this number a correct model on noisy data is indistinguishable from a wrong
    one.
    """
    diffs, ys = [], []
    for seg in segs:
        for i in range(1, len(seg)):
            diffs.append(seg.y[i] - seg.y[i - 1])
            ys.append(seg.y[i])
    if len(ys) < 8:
        return None
    ybar = sum(ys) / len(ys)
    var_y = sum((y - ybar) ** 2 for y in ys) / len(ys)
    var_n = sum(d * d for d in diffs) / (2.0 * len(diffs))
    if var_n <= 0 or var_y <= var_n:
        return None
    return 10.0 * math.log10((var_y - var_n) / var_n)


def _score_onestep(segs, tau, k, d):
    """R^2 of the one-step-ahead prediction (flatters slow poles; reported for
    continuity with the old output, not as the criterion)."""
    pred, meas = [], []
    for seg in segs:
        for i in range(1, len(seg)):
            a = math.exp(-seg.dt[i] / tau)
            pred.append(a * seg.y[i - 1] + (1.0 - a) * (k * seg.u[i] + d))
            meas.append(seg.y[i])
    if len(meas) < 4:
        return float("nan")
    ybar = sum(meas) / len(meas)
    sse = sum((p - m) ** 2 for p, m in zip(pred, meas))
    sst = sum((m - ybar) ** 2 for m in meas)
    return 1.0 - sse / sst if sst > 0 else float("nan")


def _score_freerun(segs, tau, k, d):
    """NRMSE fit percentage of a FREE RUN -- the number that matters.

    The model gets one initial condition per segment and then only u. 100 =
    perfect, 0 = no better than predicting the mean, negative = worse than that.
    """
    sim, meas = [], []
    for seg in segs:
        y = seg.y[0]
        for i in range(1, len(seg)):
            a = math.exp(-seg.dt[i] / tau)
            y = a * y + (1.0 - a) * (k * seg.u[i] + d)
            sim.append(y)
            meas.append(seg.y[i])
    if len(meas) < 4:
        return None
    ybar = sum(meas) / len(meas)
    num = math.sqrt(sum((m - s) ** 2 for m, s in zip(meas, sim)))
    den = math.sqrt(sum((m - ybar) ** 2 for m in meas))
    return 100.0 * (1.0 - num / den) if den > 0 else float("nan")


# ---------------------------------------------------------------------------
# Data audit -- what actually went wrong with a run
# ---------------------------------------------------------------------------
def diagnose(rows, phase, y_col, label, max_abs=None, deadband=None,
             gyro_fs_rad=None):
    """Audit one phase of the data and print what is wrong with it.

    Every check here exists because it fired on real data from this rig. The
    fit can only ever report that it failed; these say WHY.
    """
    use = [r for r in rows if r.get("phase") == phase]
    print(f"\n--- diagnostico: {label} (phase={phase}, {len(use)} linhas) ---")
    if not use:
        print("  nenhuma linha com essa fase")
        return
    problems = []

    dts = [d for d in (_f(r, "dt") for r in use) if d and d > 0]
    ts = _median(dts)
    late = sum(1 for d in dts if d > 1.5 * ts)
    print(f"  dt: mediana {ts * 1000:.1f} ms   p90 {_pct(dts, 0.9) * 1000:.1f}   "
          f"max {max(dts) * 1000:.1f}   atrasados >1.5x: {late} "
          f"({100.0 * late / len(dts):.0f}%)")
    if max(dts) > 3.0 * ts:
        problems.append(
            f"o laco estourou o periodo ate {max(dts) / ts:.1f}x a mediana. "
            f"Isso nao invalida o ajuste (cada amostra usa o proprio dt), mas "
            f"diz que a aquisicao estava disputando CPU com alguma coisa.")

    # Encoder count rate vs loop period: if the loop stretches exactly when the
    # encoders are busiest, the pigpio callbacks are the thing stretching it.
    rate, dtv = [], []
    for r in use:
        dl, dr, d = _f(r, "dc_left"), _f(r, "dc_right"), _f(r, "dt")
        if None in (dl, dr, d) or d <= 0:
            continue
        rate.append(2.0 * (abs(dl) + abs(dr)) / d)     # ~edges/s over 4 encoders
        dtv.append(d)
    if len(rate) > 20:
        cr = _corr(rate, dtv)
        print(f"  encoder: {sum(rate) / len(rate):.0f} bordas/s medias, "
              f"pico {max(rate):.0f}   corr(bordas/s, dt) = {cr:+.2f}")
        if cr > 0.3:
            problems.append(
                f"corr(taxa de bordas, dt) = {cr:+.2f}: o laco fica MAIS LENTO "
                f"justamente quando os encoders estao mais rapidos. Assinatura "
                f"de callback pigpio saturando (cada borda atravessa o socket "
                f"de notificacao e pega a GIL). Acima disso o daemon comeca a "
                f"perder bordas e a contagem vira lixo.")

    # Physically impossible outputs.
    ys = [v for v in (_f(r, y_col) for r in use) if v is not None]
    if ys:
        print(f"  {y_col}: media {sum(ys) / len(ys):+.4f}  "
              f"min {min(ys):+.4f}  max {max(ys):+.4f}")
    if max_abs and ys:
        bad = sum(1 for v in ys if abs(v) > max_abs)
        if bad:
            problems.append(
                f"{bad}/{len(ys)} amostras ({100.0 * bad / len(ys):.0f}%) com "
                f"|{y_col}| acima do maximo fisico {max_abs:g}; pico "
                f"{max(abs(v) for v in ys):.2f}. Nao e o kart -- e a contagem.")

    # Encoder pathologies (translational runs only; a spin legitimately has one
    # side negative).
    if phase != "prbs_head":
        stall = rev = asym = 0
        for r in use:
            dl, dr, u = _f(r, "dc_left"), _f(r, "dc_right"), _f(r, "duty")
            if None in (dl, dr, u):
                continue
            if u > 1500:
                if abs(dl) < 2 and abs(dr) < 2:
                    stall += 1
                if dl < -5 or dr < -5:
                    rev += 1
            if max(abs(dl), abs(dr)) > 80 and min(abs(dl), abs(dr)) < 10:
                asym += 1
        if stall:
            problems.append(
                f"{stall} amostras com ZERO contagem nos DOIS lados a duty>1500 "
                f"-- com o kart a toda. O fluxo de callbacks parou.")
        if rev:
            problems.append(
                f"{rev} amostras contando PARA TRAS com duty de avanco -- "
                f"bordas perdidas corrompem a maquina de estados da quadratura.")
        if asym:
            problems.append(
                f"{asym} amostras com um lado >80 contagens e o outro <10 "
                f"(queda de um lado so).")

    # Excitation. What matters is not the duty levels themselves but the
    # OUTPUT each level produced: that is where a dead zone shows up, and it is
    # measured rather than assumed from a config floor.
    levels = {}
    for r in use:
        u, y = _f(r, "duty"), _f(r, y_col)
        if u is None or y is None:
            continue
        levels.setdefault(u, []).append(abs(y))
    duties = sorted(levels)
    print(f"  duty: {len(duties)} nivel(is)")
    for u in duties:
        vals = levels[u]
        print(f"      u={u:6.0f}  n={len(vals):4d}  |{y_col}| medio = "
              f"{sum(vals) / len(vals):.4f}")
    if deadband:
        print(f"      (piso configurado para esta malha: {deadband:.0f} duty)")

    if len(duties) <= 2:
        problems.append(
            f"so {len(duties)} niveis de duty: K fica bem determinado, u0 NAO. "
            f"A zona morta vira uma extrapolacao longa a partir de "
            f"{min(duties):.0f}, e um erro pequeno na inclinacao a explode. "
            f"Meca o limiar no experimento static/kinetic e passe --u0.")

    # Dead-zone contamination, judged on the DATA: if the lowest level barely
    # moves the kart while the highest does, half the record is stiction, which
    # no first-order tau can explain.
    if len(duties) >= 2:
        lo_m = sum(levels[duties[0]]) / len(levels[duties[0]])
        hi_m = sum(levels[duties[-1]]) / len(levels[duties[-1]])
        span = duties[-1] / duties[0] if duties[0] else float("inf")
        if hi_m > 0 and lo_m / hi_m < 0.25 and span < 10.0:
            n_lo = len(levels[duties[0]])
            problems.append(
                f"o nivel BAIXO do PRBS (u={duties[0]:.0f}) produz apenas "
                f"{100.0 * lo_m / hi_m:.0f}% da saida do nivel alto "
                f"(u={duties[-1]:.0f}) -- muito menos que a razao de duty "
                f"({100.0 / span:.0f}%). O kart esta caindo na zona morta em "
                f"{n_lo}/{len(use)} amostras. Isso e atrito estatico, nao a "
                f"planta de primeira ordem: nenhum tau explica esses dados. "
                f"Suba o centro do PRBS ou reduza a amplitude.")

    # Gyro health.
    ws = [_f(r, "w_gyro") for r in use]
    have = sum(1 for w in ws if w is not None)
    if have < len(use):
        print(f"  w_gyro: {have}/{len(use)} preenchidos "
              f"({100.0 * (len(use) - have) / len(use):.0f}% em branco)")
        if have < 0.8 * len(use) and phase == "prbs_head":
            problems.append(
                f"{len(use) - have} linhas sem w_gyro. Use --w-from-yaw para "
                f"reconstruir a taxa a partir da coluna yaw sobre o dt da "
                f"propria linha.")
    if gyro_fs_rad:
        peak = max((abs(w) for w in ws if w is not None), default=0.0)
        print(f"  |w|max {peak:.2f} rad/s = {100 * peak / gyro_fs_rad:.0f}% do "
              f"fundo de escala")
        if peak > 0.8 * gyro_fs_rad:
            problems.append(
                f"|w| chegou a {100 * peak / gyro_fs_rad:.0f}% do fundo de "
                f"escala: acima disso a leitura satura e volta baixa e erratica.")

    if problems:
        for i, p in enumerate(problems, 1):
            print(f"  [{i}] {p}")
    else:
        print("  nada anomalo encontrado.")


def _corr(a, b):
    n = len(a)
    ma, mb = sum(a) / n, sum(b) / n
    cov = sum((x - ma) * (y - mb) for x, y in zip(a, b))
    va = sum((x - ma) ** 2 for x in a)
    vb = sum((y - mb) ** 2 for y in b)
    return cov / math.sqrt(va * vb) if va > 0 and vb > 0 else 0.0


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
    if gamma < 0.0:
        raise ValueError(
            f"gamma = {gamma:.4g} < 0: mais duty produziria MENOS velocidade. "
            f"Isso e um ajuste invalido, nao uma planta -- nao ha ganhos a "
            f"derivar dele.")

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
def _report_plant(name, model, min_free_fit=70.0):
    print(f"\n=== {name} ===")
    if model is None:
        print("  sem dados suficientes -- rode gray_regression.py primeiro")
        return False
    if "error" in model:
        print(f"  {model['error']}")
        return False

    st = model["stats"]
    drops = st["dropped_dt"] + st["dropped_clip"] + st["dropped_nan"]
    print(f"  corridas    : {', '.join(model['runs'])}")
    print(f"  metodo      : {model['method'].upper()}"
          f"{' + Huber IRLS' if model['robust'] else ''}"
          f"   (dt por amostra, ZOH exato)")
    print(f"  amostras    : {model['n']} em {model['n_seg']} trecho(s)"
          f"   descartadas: {drops}"
          f" (dt {st['dropped_dt']}, clip {st['dropped_clip']}, "
          f"vazias {st['dropped_nan']})")
    if st.get("bridged"):
        print(f"  emendas     : {st['bridged']} lacunas absorvidas no dt da "
              f"amostra seguinte (duty inalterado atraves delas)")
    if model["robust"] and model["n_outliers"]:
        print(f"  outliers    : {model['n_outliers']} amostras com peso "
              f"reduzido pelo Huber")
    print(f"  Ts mediano  : {st['ts'] * 1000:.1f} ms   "
          f"(a = exp(-Ts/tau) = {model['a_med']:.4f} nesse Ts)")
    print(f"  K           : {model['K']:.6g}")
    print(f"  tau         : {model['tau']:.4f} s")
    if model["u0_fixed"] is not None:
        print(f"  u0 (zona morta): {model['u0']:.0f} duty   (FIXADO pelo "
              f"operador; duty usado: {model['u_lo']:.0f}..{model['u_hi']:.0f})")
    else:
        print(f"  u0 (zona morta): {model['u0']:.0f} duty   "
              f"(duty usado: {model['u_lo']:.0f}..{model['u_hi']:.0f}, "
              f"{len(model['duty_levels'])} niveis)")
    print(f"  R^2 1-passo : {model['r2']:.4f}   RMSE {model['rmse']:.5g}")

    bad = False
    snr = model["snr_db"]
    if snr is not None:
        print(f"  SNR da saida: {snr:.1f} dB   (sinal vs ruido amostra-a-amostra)")
    ff = model["free_fit"]
    if ff is not None:
        print(f"  fit livre   : {ff:.1f}%   <- o numero que importa")
        if ff < min_free_fit:
            bad = True
            print(f"  AVISO: abaixo de ~{min_free_fit:g}% o modelo nao explica "
                  f"os dados.")
            if snr is not None and snr < 6.0:
                # A CORRECT model scores low on noisy data: verified on
                # synthetic runs where tau and K were recovered to within 10%
                # and the free fit still read 8%. Do not read a low percentage
                # here as "wrong model" until the SNR is fixed.
                print(f"         ...mas com SNR de {snr:.1f} dB o percentual "
                      f"seria baixo mesmo com o modelo CERTO. Trate a "
                      f"aquisicao (veja a auditoria) antes de culpar o modelo.")
    if not model["u0_sane"]:
        bad = True
        print(f"  AVISO: u0 = {model['u0']:.0f} esta fora de [0, "
              f"{model['u_lo']:.0f}]. Zona morta negativa ou acima do menor duty "
              f"que moveu o kart e impossivel. Com PRBS binario (2 niveis) u0 e "
              f"uma extrapolacao longa: meca-o no experimento static/kinetic e "
              f"passe --u0.")
    if model["K"] < 0:
        bad = True
        print(f"  AVISO: K < 0 -- mais duty daria MENOS velocidade. Impossivel; "
              f"os dados estao dominados por ruido de medicao.")
    if bad:
        print("  >>> AJUSTE INVALIDO: ignore os ganhos abaixo. <<<")
    model["valid"] = not bad
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


def _parse_max_duty(value, default):
    """--max-duty accepts 'auto' (the config's own limit), 'off', or a number."""
    if value is None or value == "auto":
        return default
    if value in ("off", "none", "0"):
        return None
    return float(value)


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

    # -- identification -----------------------------------------------------
    ap.add_argument("--method", choices=("oe", "arx"), default="oe",
                    help="oe = erro de simulacao (padrao, imune a ruido de "
                         "medicao); arx = erro de predicao de 1 passo")
    ap.add_argument("--robust", action="store_true",
                    help="IRLS de Huber: reduz o peso das amostras "
                         "esporadicas de encoder estourado")
    ap.add_argument("--allow-degenerated", action="store_true",
                    help="NAO descarta amostras com dt muito acima do "
                         "esperado. O ajuste discretiza cada amostra com o "
                         "PROPRIO dt (ZOH exato), entao um intervalo esticado "
                         "e apenas pouco informativo, nao invalido.")
    ap.add_argument("--dt-max-factor", type=float, default=DT_MAX_FACTOR,
                    help=f"descarta amostras com dt acima deste fator vezes a "
                         f"mediana (padrao {DT_MAX_FACTOR:g}; "
                         f"--allow-degenerated desliga)")
    ap.add_argument("--clip-speed", type=float, default=None,
                    help=f"descarta amostras com |v_center| acima disto, "
                         f"em m/s (sugestao {CONFIG.control.max_linear:g} = o "
                         f"maximo fisico do kart)")
    ap.add_argument("--clip-rate", type=float, default=None,
                    help="idem para |w_gyro|, em rad/s")
    ap.add_argument("--u0", type=float, default=None,
                    help="fixa a zona morta da malha de DISTANCIA (duty) em vez "
                         "de extrapola-la; use o valor do experimento "
                         "static/kinetic")
    ap.add_argument("--u0-heading", type=float, default=None,
                    help="idem para a malha de HEADING")
    ap.add_argument("--w-from-yaw", action="store_true",
                    help="reconstroi w_gyro diferenciando a coluna yaw sobre o "
                         "dt da propria linha (mais limpo e preenche todas as "
                         "linhas)")
    ap.add_argument("--min-fit", type=float, default=70.0,
                    help="fit livre minimo para considerar o ajuste valido "
                         "(padrao 70%%)")
    ap.add_argument("--diagnose-only", action="store_true",
                    help="so audita os dados, nao ajusta nada")
    ap.add_argument("--force-snippet", action="store_true",
                    help="imprime o bloco para o config.py mesmo com o ajuste "
                         "marcado invalido")

    # -- design -------------------------------------------------------------
    ap.add_argument("--kd",        type=float, default=None,
                    help="k_d da malha de distancia (padrao: menor que cumpre "
                         "a especificacao)")
    ap.add_argument("--kd-heading", type=float, default=None,
                    help="k_d da malha de heading (idem)")
    ap.add_argument("--move",      type=float, default=1.0,
                    help="degrau simulado da malha de DISTANCIA, em metros "
                         "(padrao 1.0). A demanda de duty escala com ele.")
    ap.add_argument("--move-heading", type=float, default=math.pi / 2,
                    help="degrau simulado da malha de HEADING, em radianos "
                         "(padrao pi/2 = 90 graus)")
    ap.add_argument("--max-duty",  default="auto",
                    help="exige |u| de pico abaixo disto na escolha do k_d. "
                         "'auto' (padrao) usa o output_limit de cada malha "
                         f"({CONFIG.position.output_limit:g} / "
                         f"{CONFIG.heading.output_limit:g}); 'off' desliga")
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

    if args.w_from_yaw:
        n = add_w_from_yaw(rows)
        print(f"[w] w_gyro reconstruido de yaw/dt em {n} linhas")

    dt_factor = float("inf") if args.allow_degenerated else args.dt_max_factor
    if args.allow_degenerated:
        print("[dt] --allow-degenerated: nenhuma amostra e descartada por dt. "
              "Cada uma e discretizada com o proprio dt (a_k = exp(-dt_k/tau)).")

    # -- data audit ---------------------------------------------------------
    print("\n=== auditoria dos dados ===")
    diagnose(rows, "prbs", "v_center", "DISTANCIA",
             max_abs=CONFIG.control.max_linear,
             deadband=CONFIG.position.min_move_duty)
    # No max_abs for heading: CONFIG.control.max_angular is where DRIVE commands
    # are clamped, not a speed the kart cannot reach -- an open-loop spin at
    # PRBS_heading duties runs well past it, and flagging that as a fault would
    # be crying wolf. The gyro full scale is the real ceiling on this signal.
    diagnose(rows, "prbs_head", "w_gyro", "HEADING",
             deadband=CONFIG.heading.min_turn_duty,
             gyro_fs_rad=math.radians(1000))
    if args.diagnose_only:
        return 0

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
    # The step size is per loop and in DIFFERENT units: metres for the distance
    # loop, radians for the heading one. Sharing one --move silently sized a
    # 1 rad turn as if it were a 1 m move.
    loops = (
        ("DISTANCIA  (v_center, phase=prbs)", "v_center", "prbs",
         args.kd, args.move, args.u0, args.clip_speed,
         CONFIG.position.output_limit),
        ("HEADING    (w_gyro, phase=prbs_head)", "w_gyro", "prbs_head",
         args.kd_heading, args.move_heading, args.u0_heading, args.clip_rate,
         CONFIG.heading.output_limit),
    )
    for label, y_col, phase, kd_arg, step, u0_fix, clip, dlimit in loops:
        model = fit_first_order(rows, y_col, phase, method=args.method,
                                robust=args.robust, dt_max_factor=dt_factor,
                                u0_fixed=u0_fix, clip_abs=clip)
        if not _report_plant(label, model, args.min_fit):
            continue
        try:
            g = pid_from_plant(model["K"], model["tau"], zeta, wn,
                               kd=kd_arg, wheel_scale=args.wheel_scale,
                               spec={"po": args.overshoot, "ts": ts_hit,
                                     "tr": tr_hit},
                               p_on_measurement=not args.p_on_error,
                               step_m=step, tol=args.spec_tol,
                               max_duty=_parse_max_duty(args.max_duty, dlimit))
        except ValueError as e:
            print(f"  {e}")
            continue
        _report_gains(label.split()[0].lower(), g, zeta, wn, kd_arg)

        v = verify_closed_loop(g["gamma"], model["tau"], g, zeta, wn, step)
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

    usable = [s for s in snippet if s[2].get("valid") or args.force_snippet]
    if usable:
        print("\n=== para o config.py ===")
        for name, g, m in usable:
            cls = "PositionGains" if name == "DISTANCIA" else "HeadingGains"
            floor = "min_move_duty" if name == "DISTANCIA" else "min_turn_duty"
            if not m.get("valid"):
                print(f"\n# {cls}  --- AJUSTE INVALIDO, impresso so por "
                      f"--force-snippet ---")
            print(f"\n# {cls}  (tau={m['tau']:.4f}s  K={m['K']:.6g}  "
                  f"fit livre {m['free_fit']:.0f}%)")
            print(f"kp: float = {g['k_p']:.1f}")
            print(f"ki: float = {g['k_i']:.1f}")
            print(f"kd: float = {g['k_d']:.1f}")
            print(f"{floor}: float = {m['u0']:.0f}   # zona morta"
                  f"{' medida' if m['u0_fixed'] is not None else ' ESTIMADA'}")
        print("\nNao cole direto: o teto de duty (PositionGains.output_limit = "
              f"{CONFIG.position.output_limit:g}) existe porque o encoder perde "
              "contagem acima de ~3000, e limita a banda que estes ganhos "
              "conseguem entregar. Valide em hardware.")
    elif snippet:
        print("\n=== para o config.py ===")
        print("  nenhum ajuste valido -- nada a colar. Resolva o que a "
              "auditoria acima apontou e recolete. (--force-snippet imprime "
              "assim mesmo.)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
