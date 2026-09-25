#!/usr/bin/python3
"""
P_camera: Picamera2 capture and JPEG encoding, publishing into a shared ring.

The JPEG encode itself is done in C and mostly releases the GIL, so this process
is not about the encode. It is about everything around it: the per-frame Python
write callback, the Condition notify, and -- in the old design -- the aiohttp
event loop then concatenating a full copy of every ~30 KB frame per viewer
(``b'--frame...' + frame + b'\\r\\n'``) on the same thread that had to answer
WebSocket commands. Camera work and web work were sharing one GIL with the
control loop.

Frames now land in ``ipc.FrameRing``: anonymous shared memory, one memcpy in,
one memcpy out, no pickling and no pipe. Readers (the MJPEG handler and the
legacy TCP video loop) take the newest slot and are never blocked by a slow peer.
"""
from __future__ import annotations

import io
import signal
import sys
import time

from config import CONFIG
import ipc as ipc_mod


class RingOutput(io.BufferedIOBase):
    """File-like sink for ``picamera2.outputs.FileOutput``.

    Replaces ``server.StreamingOutput``, which held the newest frame in a Python
    attribute behind a ``threading.Condition`` -- fine within one process, and
    invisible from any other.
    """

    def __init__(self, ring: ipc_mod.FrameRing):
        self.ring = ring
        self.frames = 0
        self.oversize = 0

    def write(self, buf):
        if not self.ring.publish(buf):
            self.oversize += 1
        else:
            self.frames += 1
        return len(buf)


class CameraWorker:
    """Starts and stops the camera on request from P_web.

    P_web keeps the reference count (it knows how many viewers it has); this side
    only ever sees start/stop, and start is idempotent. Refcounting here would
    mean the count survives a P_web restart, which is exactly wrong.
    """

    def __init__(self, ipc: ipc_mod.IPC, config=CONFIG):
        self.ipc = ipc
        self.config = config
        self.camera = None
        self.output = None

    def start(self) -> bool:
        if self.camera is not None:
            return True
        try:
            from picamera2 import Picamera2
            from picamera2.encoders import JpegEncoder, Quality
            from picamera2.outputs import FileOutput

            camera = Picamera2()
            camera.configure(
                camera.create_video_configuration(main={"size": (400, 300)}))
            self.output = RingOutput(self.ipc.frames)
            camera.start_recording(JpegEncoder(q=90), FileOutput(self.output),
                                   quality=Quality.VERY_HIGH)
            self.camera = camera
        except Exception as exc:                                # noqa: BLE001
            self.ipc.log("camera", f"could not start camera: {exc}")
            self.camera = None
            self.output = None
            self._set_state(running=False)
            return False
        self.ipc.log("camera", "camera started")
        self._set_state(running=True)
        return True

    def stop(self) -> None:
        if self.camera is None:
            return
        try:
            self.camera.stop_recording()
            self.camera.close()
        except Exception as exc:                                # noqa: BLE001
            self.ipc.log("camera", f"error stopping camera: {exc}")
        self.camera = None
        self.output = None
        self.ipc.log("camera", "camera stopped")
        self._set_state(running=False)

    def _set_state(self, running: bool) -> None:
        with self.ipc.camera_state:
            self.ipc.camera_state[0] = 1 if running else 0


def run(ipc: ipc_mod.IPC, config=CONFIG) -> None:
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    worker = CameraWorker(ipc, config)

    def _bail(signum, frame):
        ipc.stop_evt.set()

    signal.signal(signal.SIGTERM, _bail)

    try:
        while not ipc.stop_evt.is_set():
            # Block on the queue rather than polling: this process should cost
            # nothing at all while the camera is off.
            try:
                msg = ipc.camera_q.get(timeout=0.5)
            except Exception:                                   # Empty / closed
                continue

            action = msg.get("action") if isinstance(msg, dict) else None
            if action == "start":
                worker.start()
            elif action == "stop":
                worker.stop()
            elif action == "shutdown":
                break
    except Exception as exc:                                    # noqa: BLE001
        import traceback
        ipc.log("camera", f"FATAL: {exc}")
        traceback.print_exc(file=sys.stderr)
    finally:
        worker.stop()
        ipc.log("camera", "stopped")
