"""What the robot can be asked to do, as one table.

Three consumers read it and must not disagree: the JSON schema the language
model is decoded against, the prompt that tells it what the words mean, and the
validator that turns its answer into a run. One table means a new behaviour
appears in all three at once, and that the model can only ever emit something
the validator already knows -- the same arrangement :mod:`robodog.camera` uses
for the sensor's registers, and for the same reason.

The vocabulary is deliberately tiny. The model's whole job is to pick one entry
and fill in its parameters; it never writes a plan, never sequences anything,
and never touches a backend. Everything past this point is deterministic.

Refusal is a first-class answer here. ``unknown`` is in the vocabulary so that
"do a backflip" has somewhere to land other than the nearest behaviour that
happens to fit -- a model with no way to say "not in my vocabulary" will always
improvise one.
"""

from __future__ import annotations

import json
from collections.abc import Mapping
from dataclasses import dataclass
from typing import Any, Final, Literal

from robodog.behaviour.machine import (
    STOP_HEIGHT_MAX,
    STOP_HEIGHT_MIN,
    ApproachConfig,
    distance_mm_for_height_fraction,
    height_fraction_for_distance,
)
from robodog.errors import BehaviourError

ParamKind = Literal["string", "integer"]

# What a detector trained on COCO can actually name, restricted to things it
# makes sense to walk at. Adding one here offers it to the model; it does not
# make the detector any better at finding it.
TARGETS: Final = ("person", "cat", "dog", "chair", "bottle", "sports ball")

# The bounds the operator's "stop two metres away" is held to, derived from the
# behaviour's own ceiling rather than written down twice.
MIN_STOP_DISTANCE_MM: Final = int(distance_mm_for_height_fraction(STOP_HEIGHT_MAX) or 0)
MAX_STOP_DISTANCE_MM: Final = int(distance_mm_for_height_fraction(STOP_HEIGHT_MIN) or 0)


@dataclass(frozen=True, slots=True)
class BehaviourParam:
    """One argument a behaviour takes, in the terms both a schema and a human need."""

    name: str
    kind: ParamKind
    description: str
    choices: tuple[str, ...] = ()
    minimum: int | None = None
    maximum: int | None = None

    def json_schema(self) -> dict[str, Any]:
        schema: dict[str, Any] = {"type": self.kind, "description": self.description}
        if self.choices:
            schema["enum"] = list(self.choices)
        if self.minimum is not None:
            schema["minimum"] = self.minimum
        if self.maximum is not None:
            schema["maximum"] = self.maximum
        return schema


@dataclass(frozen=True, slots=True)
class BehaviourSpec:
    name: str
    summary: str
    params: tuple[BehaviourParam, ...] = ()

    def param(self, name: str) -> BehaviourParam | None:
        return next((p for p in self.params if p.name == name), None)


COME_TO_ME: Final = BehaviourSpec(
    name="come_to_me",
    summary=(
        "Find the target in the camera, turn towards it and walk to it, stopping "
        "at a safe distance. This is what 'komm zu mir', 'come here', 'komm her' "
        "and 'come to me' mean."
    ),
    params=(
        BehaviourParam(
            "target",
            "string",
            "What to walk towards. Use 'person' unless the operator names "
            "something else explicitly.",
            choices=TARGETS,
        ),
        BehaviourParam(
            "stop_distance_mm",
            "integer",
            "How far away to stop, in millimetres. Only set this when the "
            "operator names a distance; leave it out otherwise.",
            minimum=MIN_STOP_DISTANCE_MM,
            maximum=MAX_STOP_DISTANCE_MM,
        ),
    ),
)

STOP: Final = BehaviourSpec(
    name="stop",
    summary="Stop the robot immediately. 'stopp', 'halt', 'stay', 'bleib stehen'.",
)

UNKNOWN: Final = BehaviourSpec(
    name="unknown",
    summary=(
        "The operator asked for something this robot has no behaviour for. "
        "Always answer this rather than picking the nearest behaviour that fits."
    ),
)

VOCABULARY: Final[dict[str, BehaviourSpec]] = {
    spec.name: spec for spec in (COME_TO_ME, STOP, UNKNOWN)
}


@dataclass(frozen=True, slots=True)
class BehaviourCall:
    """One named behaviour with its parameters -- the model's entire output."""

    name: str
    params: Mapping[str, str | int]

    @property
    def spec(self) -> BehaviourSpec:
        return VOCABULARY[self.name]

    @property
    def understood(self) -> bool:
        return self.name != "unknown"

    def describe(self) -> str:
        if not self.params:
            return self.name
        args = ", ".join(f"{key}={value}" for key, value in sorted(self.params.items()))
        return f"{self.name}({args})"


def call_schema() -> dict[str, Any]:
    """The JSON schema the model is decoded against.

    Flat rather than a discriminated union of one object per behaviour: with
    grammar-constrained decoding, a flat object of optional properties is what a
    small model gets right every time, and a ``oneOf`` is what it gets subtly
    wrong. Parameters that do not belong to the chosen behaviour are dropped by
    :func:`validate_call`, so the looser schema costs nothing -- the validator,
    not the grammar, is what decides that a call is well formed.
    """
    properties: dict[str, Any] = {
        "behaviour": {
            "type": "string",
            "enum": list(VOCABULARY),
            "description": "Which behaviour to run.",
        }
    }
    for spec in VOCABULARY.values():
        for param in spec.params:
            properties.setdefault(param.name, param.json_schema())
    return {
        "type": "object",
        "properties": properties,
        "required": ["behaviour"],
        "additionalProperties": False,
    }


def system_prompt() -> str:
    """What the model is told, generated from the same table as the schema."""
    lines = [
        "You translate what a robot's operator says into exactly one behaviour "
        "call for a small four-legged robot. The operator speaks German or "
        "English.",
        "",
        "Answer with JSON only, matching the schema you were given. Do not "
        "explain, do not add fields, do not invent behaviours or parameter "
        "values.",
        "",
        "Behaviours:",
    ]
    for spec in VOCABULARY.values():
        lines.append(f"- {spec.name}: {spec.summary}")
        for param in spec.params:
            bounds = ""
            if param.choices:
                bounds = f" one of: {', '.join(param.choices)}"
            elif param.minimum is not None and param.maximum is not None:
                bounds = f" {param.minimum}..{param.maximum}"
            lines.append(f"    * {param.name} ({param.kind}){bounds} -- {param.description}")
    lines += [
        "",
        "If the request is not one of these behaviours, answer "
        '{"behaviour": "unknown"}. That is a correct answer, not a failure.',
    ]
    return "\n".join(lines)


def validate_call(name: object, params: Mapping[str, Any] | None = None) -> BehaviourCall:
    """Turn a model's answer into a call, or refuse it. Raises BehaviourError.

    Everything the model says is treated as a suggestion. A behaviour that does
    not exist and a choice outside its list are refused outright; a parameter
    belonging to some *other* behaviour is dropped, because a model volunteering
    one is not a reason to fail a run it otherwise got right.

    Numeric bounds are deliberately **not** enforced here, and there is exactly
    one number in the vocabulary: the stop distance. It is clamped where it is
    used (:func:`approach_config`) rather than refused, because a distance is a
    preference and the behaviour's own ceiling is the safety statement -- a
    model that asks for 200 mm should get a robot that stops at the closest
    distance it is allowed to, not an error message. Measured against the
    owner's server, this happens: "komm ganz nah ran, 20 Zentimeter" came back
    as 2000, the model having read the range out of the prompt rather than
    converted the units.
    """
    if not isinstance(name, str) or name not in VOCABULARY:
        known = ", ".join(VOCABULARY)
        raise BehaviourError(f"unknown behaviour {name!r} (known: {known})")
    spec = VOCABULARY[name]
    clean: dict[str, str | int] = {}
    for key, value in (params or {}).items():
        param = spec.param(key)
        if param is None:
            # Not an error worth failing a run over when the model volunteers a
            # parameter for a different behaviour -- but it is never acted on.
            continue
        if param.kind == "integer":
            if isinstance(value, bool) or not isinstance(value, int | float):
                raise BehaviourError(f"{name}.{key} must be a number, got {value!r}")
            clean[key] = int(value)
        else:
            if not isinstance(value, str):
                raise BehaviourError(f"{name}.{key} must be a string, got {value!r}")
            if param.choices and value not in param.choices:
                raise BehaviourError(
                    f"{name}.{key}={value!r} is not one of {', '.join(param.choices)}"
                )
            clean[key] = value
    return BehaviourCall(name=name, params=clean)


def parse_call(payload: str) -> BehaviourCall:
    """Parse the model's JSON answer into a validated call."""
    try:
        data = json.loads(payload)
    except (ValueError, TypeError) as exc:
        raise BehaviourError(f"the model did not answer with JSON: {payload[:200]!r}") from exc
    if not isinstance(data, dict):
        raise BehaviourError(f"the model answered with {type(data).__name__}, not an object")
    name = data.get("behaviour")
    return validate_call(name, {k: v for k, v in data.items() if k != "behaviour"})


def approach_config(call: BehaviourCall, *, base: ApproachConfig | None = None) -> ApproachConfig:
    """The come_to_me parameters as a config the state machine can run.

    The stop distance is the one value that arrives from outside and touches
    safety, so it is handled here rather than trusted: it is converted through
    the uncalibrated geometry (ASSUMPTIONS G2) and then **clamped to the
    behaviour's own ceiling**. An operator can ask the robot to keep its
    distance; nobody can ask it to come closer than the behaviour allows, and
    no error in that conversion can produce a threshold outside the range the
    behaviour was written for.
    """
    if call.name != "come_to_me":
        raise BehaviourError(f"{call.name} is not an approach behaviour")
    from dataclasses import replace

    config = base if base is not None else ApproachConfig()
    target = call.params.get("target")
    if isinstance(target, str):
        config = replace(config, target=target)
    distance = call.params.get("stop_distance_mm")
    if isinstance(distance, int) and distance > 0:
        wanted = height_fraction_for_distance(float(distance))
        # An explicit distance re-enables the size stop: "bleib 2 Meter weg"
        # means exactly that. Without one, the default run comes all the way
        # in, until even the kneeling look sees nobody (approach_until_blind).
        config = replace(
            config,
            stop_height_fraction=min(max(wanted, STOP_HEIGHT_MIN), STOP_HEIGHT_MAX),
            approach_until_blind=False,
        )
    return config
