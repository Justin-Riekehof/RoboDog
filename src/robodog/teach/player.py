"""Routine player: replays a Routine through a RobotClient on any backend.

Deterministic by default (advances the backend's own clock via client.tick);
pass realtime=True to pace against wall time for hardware backends. The player
feeds the watchdog every tick and never bypasses the safety supervisor.
"""

from __future__ import annotations

import itertools
import math
import time
from collections.abc import Callable
from dataclasses import dataclass, field

from robodog.api.client import RobotClient
from robodog.api.types import LegId, LegTarget, RobotState, SetLegTarget
from robodog.errors import CapabilityError
from robodog.kinematics.easing import cosine, linear
from robodog.teach.format import Routine


@dataclass(frozen=True, slots=True)
class PlayEvent:
    t: float
    description: str


@dataclass(slots=True)
class PlayReport:
    duration: float
    ticks: int = 0
    events: list[PlayEvent] = field(default_factory=list)


def _interpolate_leg(routine: Routine, leg: LegId, t: float) -> LegTarget | None:
    """Target for one leg at time t, from that leg's own keyframe track."""
    track = [(kf.at, kf.legs[leg]) for kf in routine.keyframes if leg in kf.legs]
    if not track or t < track[0][0]:
        return None  # before this leg's first keyframe: hold whatever it had
    for (t0, p0), (t1, p1) in itertools.pairwise(track):
        if t0 <= t <= t1:
            rate = (t - t0) / (t1 - t0)
            ease = linear if routine.interpolation == "linear" else cosine
            return LegTarget(
                x=ease(p0.x, p1.x, rate),
                y=ease(p0.y, p1.y, rate),
                z=ease(p0.z, p1.z, rate),
            )
    return track[-1][1]  # past the last keyframe: hold it


def play_routine(
    routine: Routine,
    client: RobotClient,
    *,
    tick: float = 0.02,
    realtime: bool = False,
    sleep: Callable[[float], None] = time.sleep,
    on_event: Callable[[PlayEvent], None] | None = None,
    on_sample: Callable[[float, RobotState], None] | None = None,
    sample_interval: float = 0.5,
) -> PlayReport:
    missing = routine.requires - client.capabilities
    if missing:
        raise CapabilityError(
            f"routine {routine.name!r} requires "
            f"{', '.join(sorted(c.name for c in missing))}, "
            f"which backend {client.backend_name!r} does not provide"
        )

    report = PlayReport(duration=routine.duration)

    def emit(t: float, description: str) -> None:
        event = PlayEvent(t=t, description=description)
        report.events.append(event)
        if on_event is not None:
            on_event(event)

    n_ticks = max(1, math.ceil(routine.duration / tick))
    step_index = 0
    next_sample = 0.0

    for i in range(n_ticks + 1):
        t = min(i * tick, routine.duration)

        if routine.kind == "commands":
            while step_index < len(routine.steps) and routine.steps[step_index].at <= t:
                step = routine.steps[step_index]
                client.send(step.command)
                emit(t, f"sent {step.command!r} (scheduled at {step.at:.2f}s)")
                step_index += 1
        else:
            for leg in LegId:
                target = _interpolate_leg(routine, leg, t)
                if target is not None:
                    client.send(SetLegTarget(leg=leg, target=target))

        client.heartbeat()
        if on_sample is not None and t >= next_sample:
            on_sample(t, client.state())
            next_sample += sample_interval

        if i < n_ticks:
            client.tick(tick)
            report.ticks += 1
            if realtime:
                sleep(tick)

    if on_sample is not None:
        on_sample(routine.duration, client.state())
    emit(routine.duration, f"routine {routine.name!r} finished")
    return report
