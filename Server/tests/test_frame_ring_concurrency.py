#!/usr/bin/python3
"""
Concurrency tests for the camera frame ring.

These exist because of a real bug: `FrameRing.read` used to hold the condition
for the whole read, copy included. That worked with one viewer and wedged the
web process with two -- both readers wait on the same condition, `notify_all`
wakes both, and `wait_for` only returns once it has REACQUIRED the lock, so the
two took turns holding it to copy ~30 KB each while the publisher blocked
trying to acquire the same lock to write the next frame. Those reads run in
asyncio's executor, blocked in an OS semaphore that yields nothing
cooperatively, so the stall propagated into the WebSocket command path and the
kart stopped accepting commands.

The original test suite only ever read SEQUENTIALLY, which is why it missed it.
Everything here runs readers and a writer at the same time.

Threads rather than processes: the ring's synchronisation primitives behave the
same either way, and threads keep the test fast and debuggable. The failure
being guarded against is about lock HOLD TIME, which is identical in both.
"""
import os
import sys
import threading
import time
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import ipc as ipc_mod                                            # noqa: E402


def _frame(seq: int, size: int = 4096) -> bytes:
    """A frame whose every byte identifies which frame it is, so a splice of
    two different frames is detectable by inspection."""
    return bytes([seq % 251]) * size


class TestConcurrentReaders(unittest.TestCase):
    def test_two_readers_and_a_writer_do_not_deadlock(self):
        """The regression. Two viewers plus a publisher, all live at once."""
        ring = ipc_mod.FrameRing(slots=4, slot_bytes=8192)
        stop = threading.Event()
        reads = {0: 0, 1: 0}
        errors = []

        def reader(idx):
            seq = 0
            try:
                while not stop.is_set():
                    data, seq = ring.read(seq, timeout=0.5)
                    if data is not None:
                        reads[idx] += 1
            except Exception as exc:                            # noqa: BLE001
                errors.append(exc)

        def writer():
            n = 0
            try:
                while not stop.is_set():
                    n += 1
                    ring.publish(_frame(n))
                    time.sleep(0.002)
            except Exception as exc:                            # noqa: BLE001
                errors.append(exc)

        threads = [threading.Thread(target=reader, args=(0,), daemon=True),
                   threading.Thread(target=reader, args=(1,), daemon=True),
                   threading.Thread(target=writer, daemon=True)]
        for t in threads:
            t.start()
        time.sleep(1.0)
        stop.set()
        for t in threads:
            t.join(timeout=3.0)

        self.assertEqual(errors, [], f"worker raised: {errors}")
        for t in threads:
            self.assertFalse(t.is_alive(), "a worker never exited -- deadlock")
        self.assertGreater(reads[0], 0, "reader 0 got nothing")
        self.assertGreater(reads[1], 0, "reader 1 got nothing")

    def test_publisher_is_not_starved_by_readers(self):
        """The specific symptom: with the copy inside the lock, two readers
        taking turns left the publisher queueing behind them. Publishing must
        stay fast while readers hammer the ring."""
        ring = ipc_mod.FrameRing(slots=4, slot_bytes=65536)
        stop = threading.Event()

        def reader():
            seq = 0
            while not stop.is_set():
                _data, seq = ring.read(seq, timeout=0.2)

        readers = [threading.Thread(target=reader, daemon=True)
                   for _ in range(3)]
        for t in readers:
            t.start()

        big = _frame(7, 60000)
        worst = 0.0
        try:
            for i in range(60):
                start = time.monotonic()
                ring.publish(big)
                worst = max(worst, time.monotonic() - start)
                time.sleep(0.005)
        finally:
            stop.set()
            for t in readers:
                t.join(timeout=2.0)

        # Generous bound: the point is "not blocked behind readers", not a
        # latency benchmark. The pre-fix code serialised a 60 KB copy per
        # reader into every publish.
        self.assertLess(worst, 0.25,
                        f"publish blocked for {worst * 1000:.0f} ms behind readers")

    def test_frames_are_never_spliced(self):
        """Copying outside the lock trades a mutex for a re-check. If that
        re-check were wrong, a reader could return the first half of one frame
        and the second half of another -- every byte of a frame is the same
        value here, so a splice is visible."""
        ring = ipc_mod.FrameRing(slots=3, slot_bytes=32768)
        stop = threading.Event()
        bad = []
        checked = [0]

        def reader():
            seq = 0
            while not stop.is_set():
                data, seq = ring.read(seq, timeout=0.2)
                if data is None:
                    continue
                checked[0] += 1
                if len(set(data)) != 1:
                    bad.append(seq)

        def writer():
            n = 0
            while not stop.is_set():
                n += 1
                ring.publish(_frame(n, 30000))

        threads = [threading.Thread(target=reader, daemon=True),
                   threading.Thread(target=reader, daemon=True),
                   threading.Thread(target=writer, daemon=True)]
        for t in threads:
            t.start()
        time.sleep(1.0)
        stop.set()
        for t in threads:
            t.join(timeout=3.0)

        self.assertGreater(checked[0], 0, "no frames were checked")
        self.assertEqual(bad, [], f"spliced frames at sequences {bad[:5]}")

    def test_a_stalled_reader_does_not_block_the_others(self):
        """A viewer on a slow link must not hold anything the others need."""
        ring = ipc_mod.FrameRing(slots=4, slot_bytes=8192)
        stop = threading.Event()
        fast_reads = [0]

        def slow_reader():
            seq = 0
            while not stop.is_set():
                _d, seq = ring.read(seq, timeout=0.2)
                time.sleep(0.05)               # pretends to be writing a socket

        def fast_reader():
            seq = 0
            while not stop.is_set():
                data, seq = ring.read(seq, timeout=0.2)
                if data is not None:
                    fast_reads[0] += 1

        def writer():
            n = 0
            while not stop.is_set():
                n += 1
                ring.publish(_frame(n))
                time.sleep(0.005)

        threads = [threading.Thread(target=slow_reader, daemon=True),
                   threading.Thread(target=fast_reader, daemon=True),
                   threading.Thread(target=writer, daemon=True)]
        for t in threads:
            t.start()
        time.sleep(1.0)
        stop.set()
        for t in threads:
            t.join(timeout=3.0)

        # The slow reader sleeps 50 ms per frame; the fast one should have got
        # far more than that pace allows if it were coupled to it.
        self.assertGreater(fast_reads[0], 25,
                           f"fast reader only got {fast_reads[0]} frames -- "
                           f"coupled to the slow one?")

    def test_torn_reads_are_counted_not_hidden(self):
        """A slot recycled mid-copy returns None and bumps a counter, so a ring
        that is too short for its viewers is diagnosable rather than silent."""
        ring = ipc_mod.FrameRing(slots=2, slot_bytes=8192)
        self.assertEqual(ring.torn, 0)
        self.assertTrue(ring.publish(_frame(1)), "frame did not fit the slot")
        data, seq = ring.read(0, timeout=0.2)
        self.assertIsNotNone(data)
        self.assertEqual(ring.torn, 0, "a quiet ring reported a torn read")

    def test_oversize_frames_are_rejected_not_torn(self):
        """A frame larger than a slot is a configuration error, counted under
        `dropped`. It must not be confused with a torn read, which is a
        timing problem with a different fix (more slots)."""
        ring = ipc_mod.FrameRing(slots=2, slot_bytes=1024)
        self.assertFalse(ring.publish(_frame(1, 4096)))
        self.assertEqual(ring.dropped, 1)
        self.assertEqual(ring.torn, 0)
        self.assertEqual(ring.seq, 0, "a rejected frame still bumped the seq")


class TestNonBlockingEnqueue(unittest.TestCase):
    """`put_drop_oldest(drop_timeout=0)` is what the event-loop paths use."""

    def test_zero_timeout_does_not_block_on_a_full_queue(self):
        import multiprocessing as mp
        q = mp.Queue(maxsize=1)
        q.put("first")
        time.sleep(0.05)                       # let the feeder flush

        start = time.monotonic()
        for i in range(20):
            ipc_mod.put_drop_oldest(q, i, drop_timeout=0.0)
        elapsed = time.monotonic() - start
        self.assertLess(elapsed, 0.2,
                        f"20 non-blocking enqueues took {elapsed:.3f}s")

    def test_zero_timeout_still_prefers_the_newest(self):
        import multiprocessing as mp
        q = mp.Queue(maxsize=2)
        for i in range(5):
            ipc_mod.put_drop_oldest(q, i, drop_timeout=0.0)
            time.sleep(0.01)
        drained = ipc_mod.drain(q)
        self.assertIn(4, drained, f"newest item lost; got {drained}")


if __name__ == "__main__":
    unittest.main(verbosity=2)
