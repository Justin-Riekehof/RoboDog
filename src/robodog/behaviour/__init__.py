"""Vision-guided behaviours: what the robot does when nobody's hand is on it.

The split is the same one the rest of this repo uses. :mod:`.machine` is pure
logic -- detections and a clock in, drive intents out -- and is tested with a
scripted detector and a fake clock, no camera and no robot. :mod:`.runner` is
the loop that puts those intents on a real robot through the safety supervisor.
:mod:`.vocabulary` is the single table that decides what may be asked for at
all, shared by the language model's schema, its prompt and the validator.
"""

from robodog.behaviour.machine import (
    STOP_HEIGHT_DEFAULT,
    STOP_HEIGHT_MAX,
    STOP_HEIGHT_MIN,
    ApproachConfig,
    BehaviourState,
    ComeToMe,
    Intent,
    distance_mm_for_height_fraction,
    height_fraction_for_distance,
    pick_target,
)
from robodog.behaviour.runner import BehaviourReport, BehaviourRunner
from robodog.behaviour.vocabulary import (
    TARGETS,
    VOCABULARY,
    BehaviourCall,
    BehaviourSpec,
    approach_config,
    call_schema,
    parse_call,
    system_prompt,
    validate_call,
)

__all__ = [
    "STOP_HEIGHT_DEFAULT",
    "STOP_HEIGHT_MAX",
    "STOP_HEIGHT_MIN",
    "TARGETS",
    "VOCABULARY",
    "ApproachConfig",
    "BehaviourCall",
    "BehaviourReport",
    "BehaviourRunner",
    "BehaviourSpec",
    "BehaviourState",
    "ComeToMe",
    "Intent",
    "approach_config",
    "call_schema",
    "distance_mm_for_height_fraction",
    "height_fraction_for_distance",
    "parse_call",
    "pick_target",
    "system_prompt",
    "validate_call",
]
