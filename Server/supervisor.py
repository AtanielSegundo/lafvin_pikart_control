#!/usr/bin/python3
"""
Process lifecycle and the safety watchdog.

Splitting a robot into processes introduces a failure mode the single-process
design did not have: **partial death**. Before, if the drivetrain code crashed,
the whole program went with it. Now P_control can die while P_web happily keeps
serving a UI that looks fine -- and the PCA9685 latches its last duty in
hardware, so a kart that was driving forward keeps driving forward.

That is what the watchdog here is for, and it is the reason this file exists at
all rather than a few ``Process(...).start()`` calls inline. Escalation:

  1. heartbeat older than ``control_heartbeat_s``  -> log it;
  2. process not alive                             -> stop the motors directly;
  3. heartbeat older than ``control_kill_after_s`` -> SIGTERM (its handler brakes
     first), then stop the motors as a backstop.

Startup ordering is a correctness constraint, not a style choice: every child
must be forked BEFORE the parent starts any thread or the asyncio loop. Forking a
threaded process copies only the calling thread but inherits every lock in
whatever state it was in, so a child can block forever on a mutex held by a
thread that does not exist in it.
"""
from __future__ import annotations

import sys
import threading
import time

from config import CONFIG
import ipc as ipc_mod


class Child:
    def __init__(self, name: str, target, ipc: ipc_mod.IPC, config, ctx,
                 critical: bool = False):
        self.name = name
        self.target = target
        self.ipc = ipc
        self.config = config
        self.ctx = ctx
        self.critical = critical      # death triggers the emergency motor stop
        self.process = None

    def start(self) -> None:
        # With the fork context the args are inherited, not pickled -- which is
        # exactly why the IPC bundle (queues, locks, shared arrays) can be passed
        # straight in. Under spawn this call would fail to pickle them.
        #
        # daemon=True is deliberate and load-bearing. multiprocessing's exit
        # handler *joins* non-daemon children, and ours loop until stop_evt is
        # set -- so a parent that exited without calling Supervisor.shutdown()
        # would hang forever. Daemon children are SIGTERMed instead, and each
        # one's SIGTERM handler brakes the motors before unwinding. None of them
        # spawns processes of its own (threads only), which is the one thing
        # daemon processes may not do.
        self.process = self.ctx.Process(target=self.target,
                                        args=(self.ipc, self.config),
                                        name=self.name, daemon=True)
        self.process.start()

    def is_alive(self) -> bool:
        return self.process is not None and self.process.is_alive()

    @property
    def pid(self):
        return None if self.process is None else self.process.pid

    def terminate(self, timeout: float = 2.0) -> None:
        if self.process is None:
            return
        if self.process.is_alive():
            self.process.terminate()          # SIGTERM: children brake and unwind
            self.process.join(timeout=timeout)
        if self.process.is_alive():
            self.process.kill()
            self.process.join(timeout=1.0)


class Supervisor:
    """Owns the child processes and watches them."""

    def __init__(self, ipc: ipc_mod.IPC, config=CONFIG, *,
                 with_camera: bool = True, with_aux: bool = True):
        self.ipc = ipc
        self.config = config
        self.ctx = ipc.ctx
        self.children: list[Child] = []
        self._with_camera = with_camera
        self._with_aux = with_aux
        self._watchdog: threading.Thread | None = None
        self._log_thread: threading.Thread | None = None
        self._shutting_down = threading.Event()
        self._estop_done = False
        self.degraded: list[str] = []          # names of children known dead

    # -- startup -----------------------------------------------------------
    def start(self) -> None:
        """Fork every child. Call this before starting ANY thread."""
        if not ipc_mod.has_fork():
            raise RuntimeError(
                "the multiprocess stack requires fork() and so runs on Linux "
                "only (this is the Raspberry Pi target). The hardware-free "
                "control stack is still importable and testable anywhere -- see "
                "Server/tests -- but Server()/web.py cannot start here.")
        ipc_mod.assert_fork_safe()

        import proc_control
        import proc_sensors

        # Sensors first: P_control's very first tick then already has real
        # encoder totals to baseline against instead of zeros.
        self._add("P_sensors", proc_sensors.run)
        self._add("P_control", proc_control.run, critical=True)

        if self._with_camera:
            import proc_camera
            self._add("P_camera", proc_camera.run)
        if self._with_aux:
            import proc_aux
            self._add("P_aux", proc_aux.run)

        for child in self.children:
            child.start()
            print(f"[supervisor] {child.name} pid={child.pid}")

    def _add(self, name, target, critical: bool = False) -> None:
        self.children.append(Child(name, target, self.ipc, self.config,
                                   self.ctx, critical=critical))

    def start_monitors(self) -> None:
        """Start the watchdog and the log drain. Safe only after :meth:`start`,
        because these are threads."""
        self._watchdog = threading.Thread(target=self._watch, daemon=True,
                                          name="Watchdog")
        self._watchdog.start()
        self._log_thread = threading.Thread(target=self._drain_logs, daemon=True,
                                            name="LogDrain")
        self._log_thread.start()

    # -- monitoring --------------------------------------------------------
    def _drain_logs(self) -> None:
        """Children log through the event queue rather than the inherited stderr:
        concurrent writes to one fd interleave mid-line, and routing them here
        keeps one readable, redirectable stream."""
        while not self._shutting_down.is_set():
            for item in ipc_mod.drain(self.ipc.event_q, limit=64):
                if not isinstance(item, dict):
                    continue
                if item.get("kind") == "log":
                    print(f"[{item.get('source', '?')}] {item.get('message', '')}")
                else:
                    print(f"[event] {item}")
            time.sleep(0.2)

    def _watch(self) -> None:
        p = self.config.process
        while not self._shutting_down.is_set():
            time.sleep(0.5)
            if self._shutting_down.is_set():
                break

            for child in self.children:
                if child.is_alive():
                    continue
                if child.name in self.degraded:
                    continue
                self.degraded.append(child.name)
                code = None if child.process is None else child.process.exitcode
                print(f"[supervisor] {child.name} DIED (exitcode={code})",
                      file=sys.stderr)
                if child.critical:
                    self._emergency_stop(f"{child.name} died")

            self._check_heartbeat(p)

    def _check_heartbeat(self, p) -> None:
        control = self._child("P_control")
        if control is None or not control.is_alive():
            return
        ts, _dt, steps = self.ipc.read_heartbeat()
        if steps == 0:
            return                      # hasn't started ticking yet
        age = time.monotonic() - ts
        if age <= p.control_heartbeat_s:
            return

        if age > p.control_kill_after_s:
            # Alive but wedged: it still owns the PCA9685, so we cannot safely
            # write PWM alongside it. Kill it first (its SIGTERM handler brakes),
            # then stop the motors ourselves as a backstop.
            print(f"[supervisor] control loop wedged for {age:.1f}s -- "
                  f"terminating P_control", file=sys.stderr)
            control.terminate()
            self._emergency_stop("control loop wedged")
        else:
            print(f"[supervisor] control heartbeat stale ({age:.2f}s)",
                  file=sys.stderr)

    def _emergency_stop(self, reason: str) -> None:
        if self._estop_done:
            return
        self._estop_done = True
        print(f"[supervisor] EMERGENCY MOTOR STOP: {reason}", file=sys.stderr)
        # Safe to write the PCA9685 from here only because P_control is now
        # confirmed dead -- see ipc.emergency_motor_stop.
        ipc_mod.emergency_motor_stop()

    def _child(self, name: str):
        for child in self.children:
            if child.name == name:
                return child
        return None

    # -- status ------------------------------------------------------------
    def status(self) -> dict:
        ts, dt, steps = self.ipc.read_heartbeat()
        return {
            "children": {c.name: {"pid": c.pid, "alive": c.is_alive()}
                         for c in self.children},
            "control_heartbeat_age": (None if steps == 0
                                      else round(time.monotonic() - ts, 3)),
            "control_loop_dt": round(dt, 4),
            "control_steps": steps,
            "degraded": list(self.degraded),
            "camera_running": bool(self.ipc.camera_state[0]),
            "frames_dropped": self.ipc.frames.dropped,
        }

    # -- shutdown ----------------------------------------------------------
    def shutdown(self, timeout: float = 3.0) -> None:
        self._shutting_down.set()
        self.ipc.stop_evt.set()

        # Give everyone a chance to unwind on its own (P_control brakes, the
        # camera closes, the LED strip blanks) before forcing anything.
        deadline = time.monotonic() + timeout
        for child in self.children:
            if child.process is None:
                continue
            child.process.join(timeout=max(0.1, deadline - time.monotonic()))

        for child in self.children:
            if child.is_alive():
                print(f"[supervisor] {child.name} did not exit; terminating")
                child.terminate()

        # Final backstop regardless of how cleanly that went: the motors must be
        # off when this returns.
        ipc_mod.emergency_motor_stop()
        print("[supervisor] all children stopped")

    # NOTE: no install_signal_handlers() here on purpose. The entry points
    # already own their signals -- aiohttp turns SIGINT into the KeyboardInterrupt
    # that web.py's finally block acts on, and mainv3.py installs its own
    # handlers (including SIGUSR1/2). A third handler in here would just mean two
    # shutdown paths racing each other.
