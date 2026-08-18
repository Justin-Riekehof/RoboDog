"""Backends: implementations of the Backend protocol (mock now, sim/serial later)."""

from robodog.backends.base import Backend
from robodog.backends.http import HttpBackend
from robodog.backends.mock import MockBackend

__all__ = ["Backend", "HttpBackend", "MockBackend"]
