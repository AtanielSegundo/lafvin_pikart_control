# Heading drift fix and plotting tools

Changes on branch `simple-features`, November 2026.

## Summary

The gyro heading drifted while the kart was parked: −2.85° in 5 minutes,
about −0.6°/min, and faster in the first minutes after boot. This change
adds a **zero-rate filter** that runs only while the encoders are still.
While the wheels are still it holds yaw and re-learns the gyro bias. While
they turn, yaw is plain gyro integration, as before.

Replaying the recorded parked run through the filter:

| Scenario | Before | After |
|---|---|---|
| Kart parked 300 s | −2.85° | **+0.01°** |
| Same run with +90°, −45° and +180° turns added | −2.85° error | **+0.07°** error |

The change also adds three scripts: one to plot the motor calibration, one
to plot the heading live, and one to analyse drift.

---

## 1. The problem

### Measurement

The kart was parked at 0° with the motors off. The heading was recorded for
300 s from the WebSocket telemetry at about 14 Hz:

```
python Scripts/plot_heading.py --ip 192.168.0.167 --duration 300 --csv Scripts/data/heading_still.csv
python Scripts/analyze_heading_drift.py Scripts/data/heading_still.csv
```

| Quantity | Value |
|---|---|
| Samples | 4308 over 299.9 s, 0 encoder counts |
| Drift | −2.85° total, **−0.613°/min** |
| Leftover bias (least-squares slope) | −0.0102°/s |
| Leftover bias over 30 s windows | −0.0174 to −0.0042°/s |
| Allan deviation at 0.08 s | 0.063°/s (white rate noise) |
| Allan deviation minimum | **0.0016°/s at τ = 100 s** |

At the start of the recording the yaw was already −6.3°. It had drifted
that far in the first ~6 minutes after the server started, so the drift is
fastest while the chip warms up.

![Drift analysis](Scripts/data/heading_drift_before.png)

*Top left: parked yaw, with the filter replay in green. Top right: Allan
deviation. Bottom left: leftover bias over time. Bottom right: heading error
with synthetic turns added; the shaded area is the stillness weight.*

(`Scripts/data/` is not tracked by git, so the image is only available
locally.)

### Root cause

1. **The bias is measured once.** `GyroMPU.calibrate()` averages 200
   samples (about 1 s) at start-up. The MPU6050 bias then changes as the
   chip warms up, and nothing updates it. Yaw has no gravity reference the
   way roll and pitch do, so any bias error adds up without limit.
2. **The standstill gate was off.** `StandstillConfig.enabled` was set from
   `True` to `False` in commit `8cc59f1` ("PID TUNIN"). The telemetry
   confirmed it: `standstill.frozen_events = 0`.

### What the Allan deviation shows

The curve falls with a slope of about −½ all the way to τ = 100 s. Over that
range white noise dominates, and the bias is stable enough to estimate by
averaging. At τ = 20 s the deviation is about 0.003°/s, so a 20 s average
estimates the bias to roughly a third of the error it is correcting
(0.01°/s).

---

## 2. The filter

### Rule

> The filter is active **only while the encoders are still**. While the
> wheels are turning, yaw is plain gyro integration.

### Stillness weight (`StandstillDetector.stillness()`)

A weight `w` from 0 to 1, computed from the encoders:

```
encoders count (above count_tolerance)   ->  w = 0 at once, on the first count
gyro rate above gyro_rate_floor_dps      ->  w = 0 (in-place turn safety)
quiet for enter_quiet_s (1 s)            ->  w ramps 0 -> 1 over still_ramp_s (1 s)
standstill disabled                      ->  w = 0 always
```

The 3°/s gyro-rate override is kept from the original detector. In an
in-place turn the wheels scrub sideways and can register few counts while
the chassis is really rotating.

### Zero-rate update (`ZeroRateFilter.step(rate, dt, w)`)

Each gyro step (50 Hz), with `rate` being the calibrated yaw rate in °/s:

```
w == 0 (moving):   integrated_rate = rate - bias          # bias only re-applied, no filtering
w  > 0 (still):    innov = clip(rate - bias, ±max_innovation_dps)
                   bias += w * dt/bias_tau_s * innov      # learn: true rate is 0, so the reading is bias
                   integrated_rate = (1 - w) * (rate - bias)   # hold
```

- **Stronger as stillness grows:** the longer the wheels have been quiet,
  the more the reading is trusted as bias and the more the integration is
  held.
- **Off when moving:** at the first encoder count, `w` drops to 0 and the
  rate passes straight through.
- **Learned bias while moving:** subtracting it (`apply_learned_bias`) is
  just the start-up calibration kept current. It is not filtering, and it
  can be switched off.
- **Knock protection:** the update is clipped at `max_innovation_dps`, so a
  knock on the chassis while parked cannot distort the bias.
- **Recalibration:** `calibrate()` resets the learned bias, because the
  fresh calibration replaces it.

### Parameters (`StandstillConfig` in `Server/config.py`)

| Parameter | Value | Why |
|---|---|---|
| `enabled` | `True` (was `False`) | The filter depends on the detector |
| `count_tolerance` | 4 (unchanged) | Encoder counts allowed while still, so a wheel resting on a quadrature edge doesn't count as motion |
| `enter_quiet_s` | 1.0 s (unchanged) | Quiet time before the kart counts as still |
| `gyro_rate_floor_dps` | 3.0 (unchanged) | Gyro override for in-place turns |
| `still_ramp_s` | **1.0 s** (new) | Fades the filter in after the kart stops |
| `bias_tau_s` | **20 s** (new) | From the Allan deviation: about 0.003°/s noise on the estimate, and still short compared with how fast the bias wanders with temperature |
| `max_innovation_dps` | **0.5°/s** (new) | Several times the per-sample noise, well below the 3°/s gyro floor |
| `apply_learned_bias` | **True** (new) | Subtract the learned bias while moving |

---

## 3. Files changed

### Server

| File | Change |
|---|---|
| `Server/config.py` | `StandstillConfig.enabled = True`; new fields `still_ramp_s`, `bias_tau_s`, `max_innovation_dps`, `apply_learned_bias` |
| `Server/standstill.py` | `StandstillDetector.stillness()`; new class `ZeroRateFilter`; `stillness` added to `telemetry()` and `blank_telemetry()` |
| `Server/heading.py` | `GyroMPU` takes optional `zero_rate` and `fn_stillness`. When given, the yaw axis goes through the filter instead of the on/off gate; roll and pitch are unchanged. `calibrate()` resets the learned bias |
| `Server/proc_sensors.py` | Creates the `ZeroRateFilter`, passes it to `GyroMPU` with `standstill.stillness`, and publishes `learned_bias_dps` |
| `Server/ipc.py` | Standstill shared block grows from 4 to 6 doubles (`STILL_WEIGHT`, `STILL_BIAS`); `read_standstill()` returns `stillness` and `learned_bias_dps` |
| `Server/tests/test_standstill.py` | 10 new tests (`TestStillness`, `TestZeroRateFilter`) |

### New telemetry

Under the top-level `standstill` key of the WebSocket telemetry:

```json
"standstill": {
  "moving": false,
  "frozen_events": 3,
  "frozen_s": 41.2,
  "age": 0.01,
  "stillness": 1.0,
  "learned_bias_dps": -0.00791
}
```

### Scripts

| File | Purpose |
|---|---|
| `Scripts/plot_motor_calibration.py` | **New.** Plots `Server/motor_calibration.json` (written by `motor_calibration.py`): speed lines, correction map, raw mismatch, dead band, verification and a summary table |
| `Scripts/plot_heading.py` | **New.** Live heading plot from the WebSocket (read-only): yaw, odometry theta, target, yaw rate, turn error and duty, and a compass. Records encoder motion (`enc_moved`) and the standstill flag (`still`). `--csv` saves the samples; `--duration S` records without a window |
| `Scripts/analyze_heading_drift.py` | **New.** Drift statistics, Allan deviation, leftover bias over time, and a replay of a recording through `StandstillDetector` + `ZeroRateFilter`, with and without synthetic turns |

Usage:

```
python Scripts/plot_motor_calibration.py [path.json] [--save out.png]
python Scripts/plot_heading.py --ip 192.168.0.167 [--window 60] [--csv out.csv] [--duration 300]
python Scripts/analyze_heading_drift.py capture.csv [--save out.png]
```

The default IP in these scripts (`192.168.0.237`, from `motor_calibration.py`)
is not the kart's current address, so pass `--ip 192.168.0.167`.

---

## 4. Tests

```
cd Server && python -m pytest tests -q
```

- All 10 new tests pass.
- 201 tests pass in total.
- **5 tests fail that were already failing.** They are in
  `tests/test_core.py::TestGyroHeadingTurn`
  (`test_turn_settles_without_overshooting` at 45, 90, −90 and 180°, and
  `test_turn_survives_a_stickier_plant`). They fail the same way on the code
  without this change, most likely since the recent heading-gain update
  (`46d6922 GAINS UPDATED`), and are unrelated to the filter.

The new tests cover:

- **Stillness weight:** 0 while moving, ramps in after the quiet time, drops
  to 0 on the first encoder count, and is 0 when the detector is disabled.
- **Filter:** plain integration while moving, hold and learn while still,
  learned bias removed on a later drive, `apply_learned_bias=False`
  respected, knock clipping, and reset.

`heading.py` imports `mpu6050`, which is only installed on the Pi. On the
laptop it was only compile-checked, so the first real check of the gyro
thread is on the kart.

---

## 5. Deploying and checking

1. Copy `Server/` to the Pi and restart the server. The gyro calibrates at
   start-up, so keep the kart still for the first couple of seconds.
2. **Parked check:** leave the kart still for 5 minutes. Yaw should stay
   flat and `learned_bias_dps` should settle on a small steady value. The
   start-up calibration is redone on restart, so it won't necessarily be
   the −0.01°/s from the recorded run.
   ```
   python Scripts/plot_heading.py --ip 192.168.0.167 --duration 300 --csv Scripts/data/heading_still_after.csv
   python Scripts/analyze_heading_drift.py Scripts/data/heading_still_after.csv
   ```
3. **Drive check:** drive, park for a while, turn to a known angle (for
   example 90° against a reference line), and park again. The turn should
   land as before, and the heading should not creep during the stops.
4. **PID tuning:** the gate was switched off in the "PID TUNIN" commit.
   If tuning runs behave differently now, check `stillness` in telemetry
   during the end of a turn. It should be 0 while the wheels are still
   creeping.

### Rolling back

| To | Set |
|---|---|
| Turn everything off (behaviour before this change) | `StandstillConfig.enabled = False` |
| Keep freezing while parked but not subtract the learned bias while moving | `apply_learned_bias = False` |
| Make the filter learn faster or slower | `bias_tau_s` (smaller is faster but noisier) |

### Known limits

- **Bias while driving:** the bias is only learned while the kart is
  parked. During a long drive with no stops it can still wander with
  temperature, starting from the last learned value.
- **Kart rotated by hand:** if the kart is turned slowly (below 3°/s) with
  the wheels locked and not counting, that rotation is lost. The encoders
  read "still" and the gyro rate is below the floor.
- **Replay accuracy:** the replay used the 14 Hz telemetry, not the raw
  50 Hz gyro, so the "after" numbers are estimates until the parked check
  on the kart confirms them.
