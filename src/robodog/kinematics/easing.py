"""Keyframe interpolation, ported from ServoCtrl.h (linearCtrl / besselCtrl)."""

from __future__ import annotations

import math


def linear(start: float, end: float, rate: float) -> float:
    """Port of linearCtrl: straight interpolation, rate in [0, 1]."""
    return (end - start) * rate + start


def cosine(start: float, end: float, rate: float) -> float:
    """Port of besselCtrl: cosine ease-in-out, rate in [0, 1]."""
    return (end - start) * ((math.cos(rate * math.pi - math.pi) + 1) / 2) + start
