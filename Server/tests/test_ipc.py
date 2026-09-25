#!/usr/bin/python3
"""
Unit tests for the multiprocess plumbing (Server/ipc.py) and the sensor-side
logic that moved out of DriveController.

These cover the code the process split ADDED, which is where the new bugs would
live:

  * differencing monotonic encoder totals instead of consuming read-and-reset
    deltas -- including the single-phase M3 rule, which is now applied on the
    consumer side and would otherwise be free to drift from the producer's;
  * the staleness checks, which are the whole reason a frozen sensor value can't
    silently drive the heading PID;
  * the front guard's hysteresis, lifted out of DriveController._dist_guard;
  * the camera frame ring and the never-block queue policy.

The closed-loop equivalence test at the end is the important one: it drives a
DriveController entirely through the shared-memory adapters and asserts it still
converges, i.e. that the feedback path across a process boundary is faithful.

Runs anywhere -- no pigpio, no fork, no Pi.
"""
import math
import os
import sys
import time
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import ipc as ipc_mod                                            # noqa: E402
from config import CONFIG, ENCODER_TAGS, SideMapping             # noqa: E402
from drive_controller import DriveController, SimulatedDrivePlant  # noqa: E402
from encoders import SimulatedEncoder, WheelEncoders, side_means  # noqa: E402


def _ipc():
    """An IPC bundle without forking anything."""
    return ipc_mod.IPC(CONFIG)


class TestQueuePolicy(unittest.TestCase):
    """Two separate properties, asserted separately, because they have different
    failure modes: the bound on blocking is what protects the control loop, and
    preferring the newest item is what makes dropping acceptable."""

    def test_put_drop_oldest_keeps_the_newest_item(self):
        import multiprocessing as mp
        q = mp.Queue(maxsize=2)
        for i in range(6):
            self.assertTrue(ipc_mod.put_drop_oldest(q, i))
            # Let the feeder thread flush, so the queue's state is well defined
            # at each step and this asserts the drop-oldest logic rather than the
            # machine's current load.
            time.sleep(0.01)
        drained = ipc_mod.drain(q)
        self.assertLessEqual(len(drained), 3)
        self.assertIn(5, drained)             # the newest command survived

    def test_put_never_blocks_for_long_on_a_wedged_consumer(self):
        """The property the control loop depends on: enqueueing to a queue nobody
        is draining must return promptly instead of propagating the stall."""
        import multiprocessing as mp
        q = mp.Queue(maxsize=1)
        for i in range(4):
            q.put(i) if i == 0 else None
        time.sleep(0.05)
        start = time.monotonic()
        for i in range(5):
            ipc_mod.put_drop_oldest(q, 100 + i)
        elapsed = time.monotonic() - start
        # 5 puts, each bounded by 3 retries x drop_timeout(20 ms) = 60 ms.
        self.assertLess(elapsed, 5 * 0.06 + 0.5,
                        f"put_drop_oldest blocked for {elapsed:.3f}s")

    def test_drain_latest(self):
        import multiprocessing as mp
        q = mp.Queue(maxsize=8)
        for i in range(5):
            q.put(i)
        time.sleep(0.05)                      # feeder thread
        self.assertEqual(ipc_mod.drain_latest(q), 4)

    def test_drain_of_empty_queue_is_empty(self):
        import multiprocessing as mp
        self.assertEqual(ipc_mod.drain(mp.Queue()), [])


class TestFrameRing(unittest.TestCase):
    def test_publish_and_read_roundtrip(self):
        ring = ipc_mod.FrameRing(slots=3, slot_bytes=1024)
        ring.publish(b"\xff\xd8frame-one")
        data, seq = ring.read(0, timeout=0.5)
        self.assertEqual(data, b"\xff\xd8frame-one")
        self.assertEqual(seq, 1)

    def test_reader_gets_the_newest_frame_not_a_backlog(self):
        """A viewer that falls behind must skip, not replay. With one shared
        `frame` attribute and one Condition, a slow viewer used to hold up the
        others."""
        ring = ipc_mod.FrameRing(slots=3, slot_bytes=1024)
        for i in range(5):
            ring.publish(f"frame-{i}".encode())
        data, seq = ring.read(0, timeout=0.5)
        self.assertEqual(data, b"frame-4")
        self.assertEqual(seq, 5)

    def test_independent_cursors(self):
        ring = ipc_mod.FrameRing(slots=3, slot_bytes=1024)
        ring.publish(b"a")
        _d1, s1 = ring.read(0, timeout=0.5)
        ring.publish(b"b")
        # Second viewer, its own cursor, sees the newest.
        d2, _s2 = ring.read(0, timeout=0.5)
        self.assertEqual(d2, b"b")
        # First viewer resumes from where it was.
        d1b, _ = ring.read(s1, timeout=0.5)
        self.assertEqual(d1b, b"b")

    def test_timeout_returns_none(self):
        ring = ipc_mod.FrameRing(slots=2, slot_bytes=64)
        data, seq = ring.read(0, timeout=0.05)
        self.assertIsNone(data)
        self.assertEqual(seq, 0)

    def test_oversize_frame_is_dropped_and_counted(self):
        ring = ipc_mod.FrameRing(slots=2, slot_bytes=16)
        self.assertFalse(ring.publish(b"x" * 64))
        self.assertEqual(ring.dropped, 1)
        self.assertEqual(ring.seq, 0)         # nothing was published


class TestSharedEncoderReader(unittest.TestCase):
    def setUp(self):
        self.ipc = _ipc()
        # No single-phase rule, to isolate the differencing itself.
        self.sides = SideMapping()
        self.reader = ipc_mod.SharedEncoderReader(self.ipc, self.sides, modes={})

    def test_first_read_adopts_a_baseline(self):
        """P_sensors may have been counting for minutes before P_control attaches.
        The first tick must not integrate all of that as one step."""
        self.ipc.publish_encoders({"M1": 5000, "M2": 5000, "M3": -5000,
                                   "M4": -5000})
        self.assertEqual(self.reader.read_reset_sides(), (0.0, 0.0))

    def test_deltas_match_the_in_process_aggregation(self):
        signs = self.sides.signs
        self.ipc.publish_encoders({t: 0 for t in ENCODER_TAGS})
        self.reader.read_reset_sides()                 # baseline

        step = {"M1": 100, "M2": 100, "M3": -100, "M4": -100}
        self.ipc.publish_encoders(step)
        left, right = self.reader.read_reset_sides()

        expected = side_means(step, self.sides, {})
        self.assertAlmostEqual(left, expected[0], places=6)
        self.assertAlmostEqual(right, expected[1], places=6)
        # Signs are configured so forward counts positive on both sides.
        self.assertGreater(left, 0)
        self.assertGreater(right, 0)
        self.assertEqual(signs["M3"], -1)              # sanity: unchanged

    def test_successive_deltas_are_relative_not_absolute(self):
        self.ipc.publish_encoders({t: 0 for t in ENCODER_TAGS})
        self.reader.read_reset_sides()
        for expected_counts in (100, 200, 300):
            self.ipc.publish_encoders({"M1": expected_counts,
                                       "M2": expected_counts,
                                       "M3": -expected_counts,
                                       "M4": -expected_counts})
            left, right = self.reader.read_reset_sides()
            self.assertAlmostEqual(left, 100.0, places=6)
            self.assertAlmostEqual(right, 100.0, places=6)

    def test_a_skipped_publication_self_corrects(self):
        """The reason totals beat a cross-process read-and-reset handshake: a
        publication that never lands must not lose counts, only delay them."""
        self.ipc.publish_encoders({t: 0 for t in ENCODER_TAGS})
        self.reader.read_reset_sides()
        # P_sensors advances twice but we only read once -- e.g. the control loop
        # was late. The delta must cover BOTH steps.
        self.ipc.publish_encoders({"M1": 50, "M2": 50, "M3": -50, "M4": -50})
        self.ipc.publish_encoders({"M1": 120, "M2": 120, "M3": -120, "M4": -120})
        left, right = self.reader.read_reset_sides()
        self.assertAlmostEqual(left, 120.0, places=6)
        self.assertAlmostEqual(right, 120.0, places=6)

    def test_single_phase_rule_is_applied_consumer_side(self):
        """M3 counts unsigned magnitude at half resolution; direction is borrowed
        from M4. That rule now runs where the signs live (P_control), so it has
        to survive the shared-memory hop."""
        reader = ipc_mod.SharedEncoderReader(
            self.ipc, self.sides, modes={"M3": {"direction_from": "M4",
                                                "scale": 2.0}})
        self.ipc.publish_encoders({t: 0 for t in ENCODER_TAGS})
        reader.read_reset_sides()

        # Driving forward: M4 raw is negative (sign -1 -> positive travel), M3
        # reports +60 of magnitude, scaled x2 and given M4's direction.
        self.ipc.publish_encoders({"M1": 100, "M2": 100, "M3": 60, "M4": -100})
        left, right = reader.read_reset_sides()
        self.assertAlmostEqual(left, 100.0, places=6)
        # right = mean(M3, M4) = mean(60*2*(+1), -100*-1) = mean(120, 100) = 110
        self.assertAlmostEqual(right, 110.0, places=6)

    def test_raw_totals_passthrough(self):
        self.ipc.publish_encoders({"M1": 1, "M2": 2, "M3": 3, "M4": 4})
        self.assertEqual(self.reader.raw_totals(),
                         {"M1": 1, "M2": 2, "M3": 3, "M4": 4})


class TestSharedGyroReader(unittest.TestCase):
    """The staleness contract. Inside one process, `connected` was enough. Across
    processes a killed producer leaves its last yaw in shared memory forever, and
    a heading PID closing on a frozen number spins until its safety timeout."""

    def setUp(self):
        self.ipc = _ipc()
        self.gyro = ipc_mod.SharedGyroReader(self.ipc, CONFIG)

    def test_fresh_and_connected(self):
        self.ipc.publish_gyro(42.0, True, time.monotonic())
        self.assertTrue(self.gyro.is_connected())
        self.assertAlmostEqual(self.gyro.get_yaw(), 42.0)

    def test_disconnected_flag_is_honoured(self):
        self.ipc.publish_gyro(42.0, False, time.monotonic())
        self.assertFalse(self.gyro.is_connected())

    def test_stale_sample_reads_as_disconnected(self):
        """Producer alive and publishing, but its sensor thread is wedged in an
        I2C read: the sample timestamp stops advancing."""
        old = time.monotonic() - 10 * CONFIG.process.gyro_stale_s
        self.ipc.publish_gyro(42.0, True, old)
        self.assertFalse(self.gyro.is_connected())

    def test_stale_publication_reads_as_disconnected(self):
        """P_sensors died: nothing updates the block at all."""
        self.ipc.publish_gyro(42.0, True, time.monotonic())
        with self.ipc.gyro:
            self.ipc.gyro[ipc_mod.GYRO_TS] = \
                time.monotonic() - 10 * CONFIG.process.gyro_stale_s
        self.assertFalse(self.gyro.is_connected())

    def test_recovery(self):
        self.ipc.publish_gyro(1.0, True, time.monotonic() - 10.0)
        self.assertFalse(self.gyro.is_connected())
        self.ipc.publish_gyro(2.0, True, time.monotonic())
        self.assertTrue(self.gyro.is_connected())


class TestSharedFrontGuard(unittest.TestCase):
    def setUp(self):
        self.ipc = _ipc()
        self.guard = ipc_mod.SharedFrontGuard(self.ipc, CONFIG)

    def test_engaged_when_published_fresh(self):
        self.ipc.publish_distance(5.0, True, True)
        self.assertTrue(self.guard.engaged())
        self.assertEqual(self.guard.distance_cm, 5.0)

    def test_clear_when_published_clear(self):
        self.ipc.publish_distance(150.0, False, True)
        self.assertFalse(self.guard.engaged())

    def test_stale_guard_fails_open(self):
        """The guard only ever vetoes motion, so a dead P_sensors must not brick
        the kart -- it fails open, and the stale distance stops being reported."""
        self.ipc.publish_distance(5.0, True, True)
        with self.ipc.dist:
            self.ipc.dist[ipc_mod.DIST_TS] = \
                time.monotonic() - 10 * CONFIG.process.guard_stale_s
        self.assertFalse(self.guard.engaged())
        self.assertIsNone(self.guard.distance_cm)

    def test_unknown_distance_is_none_not_nan(self):
        self.ipc.publish_distance(None, False, True)
        self.assertIsNone(self.guard.distance_cm)


class _FakeSensor:
    def __init__(self, readings):
        self.readings = list(readings)
        self.healthy = True

    def get_distance(self):
        return self.readings.pop(0) if self.readings else 300


class TestFrontGuardMonitor(unittest.TestCase):
    """The hysteresis and TTL lifted out of DriveController._dist_guard."""

    def _monitor(self, readings):
        import proc_sensors
        return proc_sensors.FrontGuardMonitor(_FakeSensor(readings), _ipc(),
                                             CONFIG)

    def test_trips_below_the_limit(self):
        limit = CONFIG.control.minimum_front_distance_cm
        mon = self._monitor([limit - 1])
        mon.poll_once()
        self.assertTrue(mon.engaged)

    def test_hysteresis_holds_between_limit_and_release(self):
        limit = CONFIG.control.minimum_front_distance_cm
        mon = self._monitor([limit - 1, limit + 2])
        mon.poll_once()
        self.assertTrue(mon.engaged)
        mon.poll_once()          # inside the hysteresis band: still engaged
        self.assertTrue(mon.engaged)

    def test_releases_above_limit_plus_hysteresis(self):
        limit = CONFIG.control.minimum_front_distance_cm
        mon = self._monitor([limit - 1])
        mon.poll_once()
        self.assertTrue(mon.engaged)
        # Three clear readings, to flush the 3-deep "closest recent" window.
        mon.sensor.readings = [200, 200, 200]
        for _ in range(3):
            mon.poll_once()
        self.assertFalse(mon.engaged)

    def test_failure_sentinel_is_not_treated_as_a_reading(self):
        """The RPi.GPIO sensor answers 'no echo' with 255; taking that as 255 cm
        would read as a clear road."""
        mon = self._monitor([255, 0, None])
        for _ in range(3):
            mon.poll_once()
        self.assertEqual(mon.recent, [])
        self.assertIsNone(mon.reported)

    def test_reported_distance_expires_but_the_guard_does_not(self):
        limit = CONFIG.control.minimum_front_distance_cm
        mon = self._monitor([limit - 1])
        mon.poll_once()
        self.assertTrue(mon.engaged)
        # Age the reading past the TTL, then feed only failed reads.
        mon.reported_ts -= (CONFIG.control.front_distance_ttl_s + 1.0)
        mon.sensor.readings = [255]
        mon.poll_once()
        self.assertIsNone(mon.reported)      # stops REPORTING a stale number
        self.assertTrue(mon.engaged)         # but the guard keeps its history


class TestClosedLoopAcrossSharedMemory(unittest.TestCase):
    """End-to-end: run the real controller with its feedback coming only through
    shared memory, exactly as P_control does, and confirm it still converges.

    This is the test that would catch a sign, scale or differencing error in the
    new feedback path -- the kind of bug that on hardware looks like "it drifts
    left now".
    """

    def _make(self):
        bundle = _ipc()
        sides = SideMapping()
        encs = {t: SimulatedEncoder(0, 0, name=t) for t in ENCODER_TAGS}
        # Producer side (stands in for P_sensors) and consumer side (P_control).
        wheels = WheelEncoders(sides, encoders=encs)
        reader = ipc_mod.SharedEncoderReader(bundle, sides, modes={})
        plant = SimulatedDrivePlant(wheels, CONFIG)

        class _Motor:
            last = (0, 0, 0, 0)

            def setMotorModel(self, *d):
                self.last = d

        ctrl = DriveController(_Motor(), reader, CONFIG, plant=plant)
        return bundle, wheels, ctrl

    def _spin(self, bundle, wheels, ctrl, ticks):
        dt = 1.0 / CONFIG.control.loop_hz
        tel = {}
        for _ in range(ticks):
            # P_sensors publishes, then P_control steps -- the real ordering.
            bundle.publish_encoders(wheels.raw_totals())
            tel = ctrl.step(dt)
        return tel

    def test_forward_velocity_converges_through_shared_memory(self):
        bundle, wheels, ctrl = self._make()
        ctrl.set_twist(0.2, 0.0)
        tel = self._spin(bundle, wheels, ctrl, 400)
        self.assertAlmostEqual(tel["wheel_speed"]["left"], 0.2, delta=0.02)
        self.assertAlmostEqual(tel["wheel_speed"]["right"], 0.2, delta=0.02)
        self.assertGreater(tel["pose"]["x"], 1.0)
        self.assertAlmostEqual(tel["pose"]["y"], 0.0, delta=0.05)

    def test_spin_changes_heading_through_shared_memory(self):
        bundle, wheels, ctrl = self._make()
        ctrl.set_twist(0.0, 1.0)
        tel = self._spin(bundle, wheels, ctrl, 60)
        self.assertGreater(abs(tel["pose"]["theta"]), 0.1)

    def test_drive_distance_reaches_target_through_shared_memory(self):
        bundle, wheels, ctrl = self._make()
        ctrl.drive_distance(0.5)
        self._spin(bundle, wheels, ctrl, 400)
        self.assertFalse(ctrl.move_active())
        self.assertAlmostEqual(ctrl.odom.pose.x, 0.5,
                               delta=CONFIG.position.tolerance * 4)

    def test_raw_totals_reach_telemetry(self):
        bundle, wheels, ctrl = self._make()
        ctrl.set_twist(0.2, 0.0)
        tel = self._spin(bundle, wheels, ctrl, 40)
        self.assertEqual(set(tel["encoders"]), set(ENCODER_TAGS))
        self.assertTrue(any(v != 0 for v in tel["encoders"].values()))


class TestGuardVetoesForwardOnly(unittest.TestCase):
    """The guard's contract is unchanged by the move: veto net-forward drive,
    leave reverse and turn-in-place available to escape."""

    def _make(self, bundle):
        sides = SideMapping()
        encs = {t: SimulatedEncoder(0, 0, name=t) for t in ENCODER_TAGS}
        wheels = WheelEncoders(sides, encoders=encs)
        reader = ipc_mod.SharedEncoderReader(bundle, sides, modes={})

        class _Motor:
            last = (0, 0, 0, 0)

            def setMotorModel(self, *d):
                self.last = d

        motor = _Motor()
        guard = ipc_mod.SharedFrontGuard(bundle, CONFIG)
        ctrl = DriveController(motor, reader, CONFIG, guard=guard,
                              plant=SimulatedDrivePlant(wheels, CONFIG))
        return motor, wheels, ctrl

    def test_no_guard_means_never_engaged(self):
        bundle = _ipc()
        sides = SideMapping()
        encs = {t: SimulatedEncoder(0, 0, name=t) for t in ENCODER_TAGS}
        ctrl = DriveController(object(), WheelEncoders(sides, encoders=encs),
                               CONFIG)
        self.assertFalse(ctrl._front_guard_engaged())
        self.assertIsNone(ctrl.front_distance_cm)

    def test_forward_command_is_refused_up_front_while_engaged(self):
        """Commanding forward into an already-close obstacle is zeroed by
        set_twist itself, so the in-step veto never has to fire: the duties are
        zero and guard_blocked stays False because nothing was blocked."""
        bundle = _ipc()
        motor, wheels, ctrl = self._make(bundle)
        bundle.publish_distance(4.0, True, True)
        ctrl.set_twist(0.3, 0.0)
        dt = 1.0 / CONFIG.control.loop_hz
        tel = {}
        for _ in range(10):
            bundle.publish_encoders(wheels.raw_totals())
            tel = ctrl.step(dt)
        self.assertEqual(tel["duty"]["left"], 0)
        self.assertEqual(tel["duty"]["right"], 0)
        self.assertEqual(tel["front_distance_cm"], 4.0)

    def test_obstacle_appearing_mid_drive_is_vetoed_in_step(self):
        """The case the in-step veto exists for: already driving forward when
        P_sensors raises the guard. This is the path that crosses the process
        boundary every tick, so it is the one worth asserting."""
        bundle = _ipc()
        motor, wheels, ctrl = self._make(bundle)
        bundle.publish_distance(200.0, False, True)      # road is clear
        ctrl.set_twist(0.3, 0.0)
        dt = 1.0 / CONFIG.control.loop_hz
        for _ in range(10):
            bundle.publish_encoders(wheels.raw_totals())
            tel = ctrl.step(dt)
        self.assertFalse(tel["guard_blocked"])
        self.assertGreater(tel["duty"]["left"], 0)       # driving forward

        bundle.publish_distance(4.0, True, True)         # obstacle appears
        bundle.publish_encoders(wheels.raw_totals())
        tel = ctrl.step(dt)
        self.assertTrue(tel["guard_blocked"])
        self.assertEqual(tel["duty"]["left"], 0)
        self.assertEqual(tel["duty"]["right"], 0)
        self.assertEqual(tel["front_distance_cm"], 4.0)

    def test_reverse_is_allowed_while_engaged(self):
        bundle = _ipc()
        motor, wheels, ctrl = self._make(bundle)
        bundle.publish_distance(4.0, True, True)
        ctrl.set_twist(-0.3, 0.0)
        dt = 1.0 / CONFIG.control.loop_hz
        tel = {}
        for _ in range(10):
            bundle.publish_encoders(wheels.raw_totals())
            tel = ctrl.step(dt)
        self.assertFalse(tel["guard_blocked"])
        self.assertLess(tel["duty"]["left"], 0)

    def test_drive_distance_forward_is_refused_while_engaged(self):
        bundle = _ipc()
        motor, wheels, ctrl = self._make(bundle)
        bundle.publish_distance(4.0, True, True)
        ctrl.drive_distance(0.5)
        self.assertFalse(ctrl.move_active())


if __name__ == "__main__":
    unittest.main(verbosity=2)
