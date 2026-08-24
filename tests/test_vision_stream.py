"""MJPEG parsing, the frame hub and the file source. No camera, no robot.

The parser is pure, so the awkward cases are ordinary unit tests: a frame split
across chunks, a part without a Content-Length, a boundary the stream never
declared. The reader gets a real socket, but it is a local server in this
process serving a stream this file wrote -- the same arrangement the HTTP
backend tests use for the firmware.
"""

from __future__ import annotations

import threading
import time
from collections.abc import Iterator
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import pytest

from robodog.errors import VisionError
from robodog.vision import Box, Detection, ScriptedDetector
from robodog.vision.service import FileFrameSource, VisionService
from robodog.vision.stream import FrameHub, MjpegParser, MjpegReader, multipart_chunk
from tests.conftest import FakeClock

JPEG_A = b"\xff\xd8\xff\xe0frame-a\xff\xd9"
JPEG_B = b"\xff\xd8\xff\xe0frame-b-longer\xff\xd9"


def stream_bytes(*frames: bytes, boundary: str = "sep") -> bytes:
    return b"".join(multipart_chunk(frame, boundary) for frame in frames)


def feed_in_chunks(parser: MjpegParser, data: bytes, size: int) -> list[bytes]:
    out: list[bytes] = []
    for i in range(0, len(data), size):
        out += parser.feed(data[i : i + size])
    return out


# --- the parser -------------------------------------------------------------


def test_parses_two_frames_from_one_chunk() -> None:
    parser = MjpegParser()
    assert parser.feed(stream_bytes(JPEG_A, JPEG_B)) == [JPEG_A, JPEG_B]
    assert parser.frames == 2


@pytest.mark.parametrize("size", [1, 3, 7, 64, 4096])
def test_chunk_size_never_changes_the_frames(size: int) -> None:
    """A frame split across arbitrary reads is still exactly one frame."""
    parser = MjpegParser()
    assert feed_in_chunks(parser, stream_bytes(JPEG_A, JPEG_B, JPEG_A), size) == [
        JPEG_A,
        JPEG_B,
        JPEG_A,
    ]


def test_boundary_is_discovered_when_it_was_not_declared() -> None:
    parser = MjpegParser()
    parser.feed(stream_bytes(JPEG_A, boundary="whatever-the-robot-uses"))
    assert parser.boundary == b"whatever-the-robot-uses"


def test_declared_boundary_is_used() -> None:
    parser = MjpegParser(boundary='"sep"')
    assert parser.feed(stream_bytes(JPEG_A)) == [JPEG_A]


def test_parts_without_a_content_length_are_read_to_the_next_boundary() -> None:
    """Nothing in the spec requires the header, and the vendor has changed before."""
    body = (
        b"--sep\r\nContent-Type: image/jpeg\r\n\r\n"
        + JPEG_A
        + b"\r\n--sep\r\nContent-Type: image/jpeg\r\n\r\n"
        + JPEG_B
        + b"\r\n--sep--\r\n"
    )
    assert MjpegParser().feed(body) == [JPEG_A, JPEG_B]


def test_a_part_that_is_not_a_jpeg_is_dropped_not_forwarded() -> None:
    """A detector handed an HTML error page fails somewhere far less obvious."""
    body = (
        b"--sep\r\nContent-Type: text/html\r\nContent-Length: 5\r\n\r\nOOPS!\r\n"
        + multipart_chunk(JPEG_A, "sep")
    )
    assert MjpegParser().feed(body) == [JPEG_A]


def test_junk_that_claims_to_be_an_image_is_still_dropped() -> None:
    body = b"--sep\r\nContent-Type: image/jpeg\r\nContent-Length: 4\r\n\r\nnope\r\n"
    assert MjpegParser().feed(body) == []


def test_a_stream_that_is_not_multipart_is_refused_rather_than_buffered() -> None:
    parser = MjpegParser(max_part=512)
    with pytest.raises(VisionError, match="is this an MJPEG stream"):
        parser.feed(b"<html>" + b"x" * 1024)


def test_an_oversized_announced_part_is_refused() -> None:
    parser = MjpegParser(max_part=16)
    with pytest.raises(VisionError, match="above the 16 byte ceiling"):
        parser.feed(b"--sep\r\nContent-Length: 99999\r\n\r\n")


# --- the hub ----------------------------------------------------------------


def test_hub_hands_out_the_latest_frame_and_counts_it() -> None:
    hub = FrameHub()
    assert hub.latest() == (0, None)
    assert hub.publish(JPEG_A) == 1
    assert hub.publish(JPEG_B) == 2
    assert hub.latest() == (2, JPEG_B)


def test_hub_waits_only_for_frames_it_has_not_seen() -> None:
    hub = FrameHub()
    hub.publish(JPEG_A)
    assert hub.wait_for(0) == (1, JPEG_A)
    # Nothing newer: the subscriber is told so rather than handed a repeat.
    assert hub.wait_for(1, timeout=0.0) == (1, None)


def test_closing_the_hub_releases_a_waiting_subscriber() -> None:
    hub = FrameHub()
    hub.close()
    assert hub.closed
    assert hub.wait_for(5, timeout=0.0) == (0, None)


def test_hub_reports_the_frame_rate_from_arrival_times() -> None:
    clock = FakeClock()
    hub = FrameHub(clock=clock)
    assert hub.rate == 0.0
    for _ in range(5):
        hub.publish(JPEG_A)
        clock.advance(0.25)
    assert hub.rate == pytest.approx(4.0)


# --- the reader, against a local server -------------------------------------


def serve_stream(payload: bytes, *, boundary: str = "sep") -> Iterator[str]:
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args: object) -> None:
            pass

        def do_GET(self) -> None:
            self.send_response(200)
            self.send_header("Content-Type", f"multipart/x-mixed-replace; boundary={boundary}")
            self.end_headers()
            self.wfile.write(payload)

    httpd = ThreadingHTTPServer(("127.0.0.1", 0), Handler)
    # A short poll interval so shutdown() returns at once: at the default
    # half second, a file of these tests spends most of its time in teardown.
    threading.Thread(
        target=httpd.serve_forever, kwargs={"poll_interval": 0.01}, daemon=True
    ).start()
    try:
        yield f"http://127.0.0.1:{httpd.server_address[1]}/stream"
    finally:
        httpd.shutdown()
        httpd.server_close()


@pytest.fixture
def stream_url() -> Iterator[str]:
    yield from serve_stream(stream_bytes(JPEG_A, JPEG_B))


def test_reader_publishes_every_frame_it_reads(stream_url: str) -> None:
    hub = FrameHub()
    MjpegReader(stream_url, hub).read_once()
    assert hub.seq == 2
    assert hub.latest()[1] == JPEG_B


def test_reader_reports_an_unreachable_stream_by_name() -> None:
    hub = FrameHub()
    reader = MjpegReader("http://127.0.0.1:1/stream", hub, timeout=0.2)
    with pytest.raises(VisionError, match="cannot read the camera stream"):
        reader.read_once()


# --- frames from disk, which is how this runs with the robot switched off ----


def test_file_source_publishes_the_frames_it_was_given(tmp_path: Path) -> None:
    (tmp_path / "a.jpg").write_bytes(JPEG_A)
    (tmp_path / "b.jpg").write_bytes(JPEG_B)
    (tmp_path / "notes.txt").write_bytes(b"ignored")
    hub = FrameHub()
    source = FileFrameSource(tmp_path, hub, fps=50)
    assert [p.name for p in source.paths] == ["a.jpg", "b.jpg"]
    source.start()
    source.stop()
    assert hub.seq >= 1


def test_file_source_says_so_when_there_is_nothing_to_play(tmp_path: Path) -> None:
    with pytest.raises(VisionError, match=r"holds no \.jpg frames"):
        FileFrameSource(tmp_path, FrameHub())
    with pytest.raises(VisionError, match="no such frame source"):
        FileFrameSource(tmp_path / "nope", FrameHub())


# --- detections expire, because a stalled detector is not a quiet room -------


def test_detections_expire_when_the_detector_stops_answering(tmp_path: Path) -> None:
    """The safety property behind "never walk without a fresh detection".

    A frozen stream -- a second viewer stealing the robot's single frame buffer
    (ASSUMPTIONS F4/G3), a crashed model -- must look like a camera that sees
    nothing. Handing back the last answer would let a behaviour walk forward at
    a person who left minutes ago.
    """
    (tmp_path / "a.jpg").write_bytes(JPEG_A)
    found = Detection("person", 0.9, Box(0.4, 0.2, 0.6, 0.8))
    clock = FakeClock()
    service = VisionService(
        str(tmp_path), detector=ScriptedDetector([[found]]), detection_ttl=1.0, clock=clock
    )
    service.start()
    deadline = time.monotonic() + 10.0
    while not service.detections and time.monotonic() < deadline:
        time.sleep(0.01)
    assert service.detections == (found,)
    assert not service.stale
    # Nothing more will be produced, and time passes.
    service.stop()
    clock.advance(1.5)
    assert not service.detections
    assert service.stale
    assert service.status()["stale"] is True
