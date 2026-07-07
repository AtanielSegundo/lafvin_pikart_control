#!/usr/bin/python3
"""
Wire protocol + command routing for the robot.

Two message encodings are accepted transparently on any transport
(WebSocket or TCP):

  * Legacy text:  ``CMD_MOTOR#1000#1000#1000#1000``  (``#``-separated).
  * JSON object:  ``{"type": "drive", "linear": 0.3, "angular": 0.4}``.

Both are normalised into a :class:`Command` and dispatched through a
:class:`CommandRouter`. New commands are added by registering a handler -
no editing of a giant if/elif chain - which is what makes the control surface
extensible.

Telemetry flows the other way as JSON lines (see :func:`telemetry_message`).
"""
from __future__ import annotations

import json
import time
from dataclasses import dataclass, field
from typing import Any, Callable, Dict, List, Optional


# Map every accepted command name (legacy CMD_* and JSON "type") to a single
# canonical handler key.
ALIASES: Dict[str, str] = {
    # motion
    "CMD_MOTOR": "motor", "motor": "motor",
    "CMD_M_MOTOR": "mecanum", "mecanum": "mecanum", "m_motor": "mecanum",
    "CMD_CAR_ROTATE": "car_rotate", "car_rotate": "car_rotate",
    "drive": "drive",                       # closed-loop velocity (new)
    "drive_distance": "drive_distance",     # closed-loop distance (new)
    "turn": "turn",                         # closed-loop in-place turn (new)
    "reset_odometry": "reset_odometry",     # (new)
    "set_sign": "set_sign",                 # runtime encoder sign flip (new)
    # peripherals
    "CMD_SERVO": "servo", "servo": "servo",
    "CMD_LED": "led", "led": "led",
    "CMD_LED_MOD": "led_mode", "led_mode": "led_mode",
    "CMD_BUZZER": "buzzer", "buzzer": "buzzer",
    "CMD_SONIC": "sonic", "sonic": "sonic",
    "CMD_LIGHT": "light", "light": "light",
    "CMD_POWER": "power", "power": "power",
    "CMD_MODE": "mode", "mode": "mode",
    # meta
    "ping": "ping",
}


@dataclass
class Command:
    name: str                                  # canonical name
    raw_name: str = ""                         # as received
    args: List[str] = field(default_factory=list)   # legacy positional tokens
    kwargs: Dict[str, Any] = field(default_factory=dict)  # JSON fields
    raw: str = ""

    # -- convenience accessors, tolerant of both encodings -----------------
    def arg(self, index: int, default: Any = None) -> Any:
        return self.args[index] if 0 <= index < len(self.args) else default

    def arg_int(self, index: int, default: int = 0) -> int:
        try:
            return int(self.args[index])
        except (IndexError, ValueError, TypeError):
            return default

    def get(self, key: str, default: Any = None) -> Any:
        return self.kwargs.get(key, default)

    def num(self, key: str, index: int, default: float = 0.0) -> float:
        """Read a numeric value from kwargs[key], falling back to args[index]."""
        if key in self.kwargs:
            try:
                return float(self.kwargs[key])
            except (ValueError, TypeError):
                return default
        try:
            return float(self.args[index])
        except (IndexError, ValueError, TypeError):
            return default


def parse(raw: str) -> Optional[Command]:
    """Parse one raw message line into a :class:`Command`, or ``None``."""
    if raw is None:
        return None
    raw = raw.strip()
    if not raw:
        return None

    if raw[0] == "{":
        try:
            obj = json.loads(raw)
        except json.JSONDecodeError:
            return None
        if not isinstance(obj, dict):
            return None
        raw_name = obj.get("type") or obj.get("cmd") or ""
        canonical = ALIASES.get(raw_name, raw_name)
        kwargs = {k: v for k, v in obj.items() if k not in ("type", "cmd")}
        return Command(name=canonical, raw_name=raw_name, kwargs=kwargs, raw=raw)

    # Legacy text form.
    tokens = raw.split("#")
    raw_name = tokens[0]
    canonical = ALIASES.get(raw_name, raw_name)
    return Command(name=canonical, raw_name=raw_name, args=tokens[1:], raw=raw)


Handler = Callable[[Command], Any]


class CommandRouter:
    """Registry mapping canonical command names to handlers."""

    def __init__(self) -> None:
        self._handlers: Dict[str, Handler] = {}

    def register(self, name: str, handler: Handler) -> None:
        self._handlers[name] = handler

    def on(self, name: str) -> Callable[[Handler], Handler]:
        """Decorator form of :meth:`register`."""
        def deco(handler: Handler) -> Handler:
            self.register(name, handler)
            return handler
        return deco

    def has(self, name: str) -> bool:
        return name in self._handlers

    def dispatch(self, message) -> Any:
        """Dispatch a raw string or an already-parsed :class:`Command`."""
        cmd = message if isinstance(message, Command) else parse(message)
        if cmd is None:
            return None
        handler = self._handlers.get(cmd.name)
        if handler is None:
            return None
        return handler(cmd)


# ---------------------------------------------------------------------------
# Telemetry helpers (server -> client)
# ---------------------------------------------------------------------------
def telemetry_message(*, battery: float, mode: str,
                      drive: Optional[dict] = None,
                      extra: Optional[dict] = None) -> str:
    payload = {
        "type": "telemetry",
        "ts": round(time.time(), 3),
        "battery": battery,
        "mode": mode,
    }
    if drive is not None:
        payload["drive"] = drive
    if extra:
        payload.update(extra)
    return json.dumps(payload)


def sensor_message(sensor: str, **values) -> str:
    payload = {"type": "sensor", "sensor": sensor}
    payload.update(values)
    return json.dumps(payload)
