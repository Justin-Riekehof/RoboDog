"""Reading the robot's MJPEG stream once, and handing it to everyone who wants it.

This unit has **no PSRAM** (`psram=0`, read back 2026-08-22), so the firmware
runs `fb_count = 1`: while a browser holds `/stream`, every other frame grab
blocks. There can be exactly one consumer of the robot's camera, and this
module is it -- the host reads the stream, and the teach UI and the detector
both read the host. Point a second browser at the robot directly and the
vision loop goes blind (ASSUMPTIONS F4/G3).

Three pieces, split by how testable they are:

* :class:`MjpegParser` is pure: bytes in, complete JPEG frames out. It holds
  the whole multipart protocol and no I/O at all, so the awkward cases -- a
  frame split across three chunks, a part with no Content-Length, a boundary
  arriving one byte at a time -- are ordinary unit tests.
* :class:`FrameHub` is the one place a frame lives once it has been read. It
  hands out the latest frame and blocks a caller until a newer one arrives,
  which is what lets the UI re-serve the stream without polling.
* :class:`MjpegReader` is the thread that connects, feeds the parser and
  reconnects when the link drops. It owns the only socket.
"""

from __future__ import annotations

import threading
import time
import urllib.error
import urllib.request
from collections import deque
from collections.abc import Callable
from typing import Final

from robodog.errors import VisionError

# A JPEG from this camera is a few tens of kilobytes at VGA (ASSUMPTIONS F5).
# The ceiling is a guard against a stream that never produces a boundary, not
# a size the robot ever reaches.
MAX_PART: Final = 1 << 20
# How long a re-broadcast subscriber waits for a new frame before it writes
# nothing and asks again. Short enough that a closed tab is noticed, long
# enough that a stalled camera does not spin the CPU.
SUBSCRIBER_TIMEOUT: Final = 2.0
DEFAULT_STREAM_TIMEOUT: Final = 5.0
DEFAULT_RETRY: Final = 1.0
# How many frame arrivals the rate estimate averages over.
_RATE_WINDOW: Final = 30

_CRLF: Final = b"\r\n"
_HEADER_END: Final = b"\r\n\r\n"
_JPEG_SOI: Final = b"\xff\xd8"


class MjpegParser:
    """Incremental multipart/x-mixed-replace parser. Pure: no sockets, no clock.

    The boundary is taken from the Content-Type header when there is one, and
    discovered from the stream's first `--...` line when there is not -- the
    ESP32 sends both, but a reader that needs the header cannot be handed a
    recorded stream, and recorded streams are how this gets tested.

    Parts without a Content-Length are supported (scan to the next boundary),
    because nothing in the spec requires one and the vendor's handler has
    changed before.
    """

    def __init__(self, *, boundary: str | bytes | None = None, max_part: int = MAX_PART) -> None:
        if isinstance(boundary, str):
            boundary = boundary.strip().strip('"').encode("ascii", "replace")
        self._boundary: bytes | None = boundary
        self._max_part = max_part
        self._buffer = bytearray()
        self._state = "boundary"
        self._expected: int | None = None
        self._is_image = True
        self.frames = 0

    @property
    def boundary(self) -> bytes | None:
        return self._boundary

    def feed(self, chunk: bytes) -> list[bytes]:
        """Add received bytes; return every complete frame they completed."""
        self._buffer += chunk
        frames: list[bytes] = []
        while True:
            if self._state == "boundary":
                if not self._take_boundary():
                    break
            elif self._state == "headers":
                if not self._take_headers():
                    break
            else:
                part = self._take_body()
                if part is None:
                    break
                if part:
                    self.frames += 1
                    frames.append(part)
        return frames

    # --- the three states -------------------------------------------------

    def _take_boundary(self) -> bool:
        if self._boundary is None:
            start = self._buffer.find(b"--")
            if start < 0:
                self._guard("no multipart boundary in the first")
                return False
            end = self._buffer.find(_CRLF, start)
            if end < 0:
                self._guard("no multipart boundary in the first")
                return False
            self._boundary = bytes(self._buffer[start + 2 : end]).strip()
            if not self._boundary:
                raise VisionError("multipart stream has an empty boundary")
            del self._buffer[: end + 2]
            self._state = "headers"
            return True
        marker = b"--" + self._boundary
        start = self._buffer.find(marker)
        if start < 0:
            self._guard("no boundary found in the last")
            return False
        end = self._buffer.find(_CRLF, start)
        if end < 0:
            return False  # the boundary line is not complete yet
        del self._buffer[: end + 2]
        self._state = "headers"
        return True

    def _take_headers(self) -> bool:
        end = self._buffer.find(_HEADER_END)
        if end < 0:
            # A part with no headers at all is legal; the blank line still
            # terminates them, so this only means "not yet".
            self._guard("part headers never ended in the last")
            return False
        raw = bytes(self._buffer[:end])
        del self._buffer[: end + len(_HEADER_END)]
        self._expected = None
        self._is_image = True
        for line in raw.split(_CRLF):
            name, _, value = line.partition(b":")
            key = name.strip().lower()
            if key == b"content-length":
                try:
                    self._expected = int(value.strip())
                except ValueError:
                    self._expected = None
            elif key == b"content-type":
                self._is_image = b"image/" in value.strip().lower()
        if self._expected is not None and self._expected > self._max_part:
            raise VisionError(
                f"stream announced a {self._expected} byte part, above the "
                f"{self._max_part} byte ceiling"
            )
        self._state = "body"
        return True

    def _take_body(self) -> bytes | None:
        """The part's payload, or None while it is still arriving.

        Returns b"" for a part that is not an image, so the caller advances
        without emitting anything.
        """
        if self._expected is not None:
            if len(self._buffer) < self._expected:
                return None
            part = bytes(self._buffer[: self._expected])
            del self._buffer[: self._expected]
        else:
            assert self._boundary is not None  # set before any body is read
            end = self._buffer.find(_CRLF + b"--" + self._boundary)
            if end < 0:
                self._guard("no end to the part in the last")
                return None
            part = bytes(self._buffer[:end])
            del self._buffer[:end]
        self._state = "boundary"
        self._expected = None
        if not self._is_image:
            return b""
        # A part that is not a JPEG is dropped rather than passed on: a
        # detector handed an HTML error page fails somewhere far less obvious.
        return part if part.startswith(_JPEG_SOI) else b""

    def _guard(self, what: str) -> None:
        if len(self._buffer) > self._max_part:
            raise VisionError(f"{what} {len(self._buffer)} bytes -- is this an MJPEG stream?")


class FrameHub:
    """The latest frame, plus a way to block until there is a newer one.

    Sequence numbers rather than timestamps: a subscriber says which frame it
    last saw, and gets the next one whenever it appears. That makes the
    re-broadcast exactly as fast as the camera and never faster, with no
    polling interval to tune.
    """

    def __init__(self, *, clock: Callable[[], float] = time.monotonic) -> None:
        self._clock = clock
        self._condition = threading.Condition()
        self._frame: bytes | None = None
        self._seq = 0
        self._arrivals: deque[float] = deque(maxlen=_RATE_WINDOW)
        self._closed = False
        self.note = ""

    @property
    def seq(self) -> int:
        return self._seq

    @property
    def rate(self) -> float:
        """Frames per second over the recent window; 0.0 before there are two."""
        with self._condition:
            if len(self._arrivals) < 2:
                return 0.0
            span = self._arrivals[-1] - self._arrivals[0]
            return (len(self._arrivals) - 1) / span if span > 0 else 0.0

    def publish(self, frame: bytes) -> int:
        with self._condition:
            self._frame = frame
            self._seq += 1
            self._arrivals.append(self._clock())
            self._condition.notify_all()
            return self._seq

    def latest(self) -> tuple[int, bytes | None]:
        with self._condition:
            return self._seq, self._frame

    def wait_for(self, after: int, timeout: float = SUBSCRIBER_TIMEOUT) -> tuple[int, bytes | None]:
        """The first frame newer than ``after``; (after, None) if none arrived."""
        with self._condition:
            if self._seq > after or self._closed:
                return self._seq, self._frame
            self._condition.wait(timeout)
            if self._seq > after:
                return self._seq, self._frame
            return after, None

    def close(self) -> None:
        """Wake every subscriber so a shutdown does not wait out their timeouts."""
        with self._condition:
            self._closed = True
            self._condition.notify_all()

    @property
    def closed(self) -> bool:
        return self._closed


class MjpegReader:
    """The single consumer of the robot's stream, in a thread of its own.

    Reconnects on its own, because the interesting failure is not a stream that
    never starts but one that stops: the robot reboots, the access point drops,
    a second viewer steals the frame buffer. None of those should end a teach
    session -- the picture comes back when the robot does, and `note` says what
    is happening in the meantime.
    """

    def __init__(
        self,
        url: str,
        hub: FrameHub,
        *,
        timeout: float = DEFAULT_STREAM_TIMEOUT,
        retry: float = DEFAULT_RETRY,
        opener: urllib.request.OpenerDirector | None = None,
        sleep: Callable[[float], None] = time.sleep,
    ) -> None:
        self.url = url
        self.hub = hub
        self.timeout = timeout
        self.retry = retry
        self._opener = opener
        self._sleep = sleep
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self.connects = 0

    def read_once(self) -> None:
        """One connection, from open until the stream ends or stop() is set."""
        self.connects += 1
        parser = MjpegParser()
        opener = self._opener or urllib.request.build_opener(urllib.request.ProxyHandler({}))
        try:
            response = opener.open(self.url, timeout=self.timeout)
        except (urllib.error.URLError, OSError, TimeoutError) as exc:
            raise VisionError(f"cannot read the camera stream at {self.url}: {exc}") from exc
        with response:
            content_type = response.headers.get("Content-Type", "")
            _, _, boundary = content_type.partition("boundary=")
            if boundary.strip():
                parser = MjpegParser(boundary=boundary)
            self.hub.note = ""
            while not self._stop.is_set():
                chunk = response.read(4096)
                if not chunk:
                    return
                for frame in parser.feed(chunk):
                    self.hub.publish(frame)

    def _run(self) -> None:
        while not self._stop.is_set():
            try:
                self.read_once()
                self.hub.note = "the camera stream ended; reconnecting"
            except VisionError as exc:
                self.hub.note = str(exc)
            except OSError as exc:  # the socket died mid-stream
                self.hub.note = f"the camera stream dropped: {exc}"
            if self._stop.wait(self.retry):
                return

    def start(self) -> None:
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, daemon=True, name="vision-stream")
        self._thread.start()

    def stop(self, timeout: float = 2.0) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=timeout)
            self._thread = None


def multipart_chunk(frame: bytes, boundary: str) -> bytes:
    """One part of an MJPEG response -- what the re-broadcast writes per frame."""
    head = (
        f"--{boundary}\r\nContent-Type: image/jpeg\r\nContent-Length: {len(frame)}\r\n\r\n"
    ).encode("ascii")
    return head + frame + _CRLF
