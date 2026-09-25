"""
DEPRECATED. ``stop_thread`` has been removed; nothing in the stack uses it.

It worked by ctypes-injecting ``SystemExit`` into a running thread, seven times
over (``PyThreadState_SetAsyncExc``). The exception lands at an arbitrary
bytecode boundary, so it could -- and did -- arrive halfway through an I2C
transaction or while a lock was held, leaving the bus or the lock in a state
nothing else could recover from. That is why stopping an autonomous mode used to
sometimes wedge the PCA9685.

Every former caller now stops cooperatively instead:

  * the legacy autonomous modes (Light / Line_Tracking / Ultrasonic) take a
    ``stop_evt`` and shut their own motors off on the way out -- see
    Server/proc_aux.py:AuxWorker.stop_mode;
  * the LED animations re-check a stop event between passes;
  * the drive loop exits through ``DriveController.stop_loop``;
  * a whole subsystem is stopped with ``Process.terminate()``, whose SIGTERM
    handler brakes first -- see Server/supervisor.py.

Kept as a file only so old entry points fail with a clear message instead of an
ImportError from ``from Thread import *``.
"""


def stop_thread(thread):
    raise NotImplementedError(
        "stop_thread() was removed: injecting SystemExit into a thread could "
        "land mid-I2C-transaction. Use a stop Event (see proc_aux.AuxWorker) or "
        "Process.terminate() (see supervisor.Child.terminate)."
    )
