"""Shared fixtures. Tests never sleep and never touch hardware."""

from __future__ import annotations

import pytest

from robodog.api.client import RobotClient
from robodog.backends.mock import MockBackend


class FakeClock:
    """Manually advanced monotonic clock for deterministic watchdog tests."""

    def __init__(self, start: float = 0.0) -> None:
        self.now = start

    def __call__(self) -> float:
        return self.now

    def advance(self, dt: float) -> None:
        self.now += dt


@pytest.fixture
def clock() -> FakeClock:
    return FakeClock()


@pytest.fixture
def backend() -> MockBackend:
    return MockBackend()


@pytest.fixture
def client(backend: MockBackend, clock: FakeClock) -> RobotClient:
    robot = RobotClient(backend, clock=clock)
    robot.connect()
    return robot
