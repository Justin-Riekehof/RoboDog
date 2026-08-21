"""CLI: exit codes, output, and error handling (no hardware, no windows)."""

from __future__ import annotations

from pathlib import Path

import pytest

from robodog.cli import main

ROUTINES_DIR = Path(__file__).resolve().parent.parent / "routines"


def test_info_reports_capabilities(capsys: pytest.CaptureFixture[str]) -> None:
    assert main(["info", "--backend", "mock"]) == 0
    out = capsys.readouterr().out
    assert "backend:      mock" in out
    assert "LOCOMOTION" in out
    assert "FRONT_LEFT" in out
    assert "DISARMED" in out


def test_validate_accepts_shipped_routines(capsys: pytest.CaptureFixture[str]) -> None:
    assert main(["validate", str(ROUTINES_DIR / "patrol-demo.yaml")]) == 0
    assert "OK:" in capsys.readouterr().out


def test_validate_rejects_a_broken_file(tmp_path: Path, capsys: pytest.CaptureFixture[str]) -> None:
    path = tmp_path / "bad.yaml"
    path.write_text("schema: nope\n", encoding="utf-8")
    assert main(["validate", str(path)]) == 1
    assert "error:" in capsys.readouterr().err


def test_play_command_routine(capsys: pytest.CaptureFixture[str]) -> None:
    assert main(["play", str(ROUTINES_DIR / "patrol-demo.yaml"), "--backend", "mock"]) == 0
    out = capsys.readouterr().out
    assert "playing 'patrol-demo'" in out
    assert "done:" in out
    assert "DISARMED" in out


def test_play_motion_routine(capsys: pytest.CaptureFixture[str]) -> None:
    assert main(["play", str(ROUTINES_DIR / "bow.yaml"), "--backend", "mock"]) == 0
    assert "playing 'bow'" in capsys.readouterr().out


def test_play_verbose_lists_commands(capsys: pytest.CaptureFixture[str]) -> None:
    assert main(["play", str(ROUTINES_DIR / "patrol-demo.yaml"), "--verbose"]) == 0
    assert "sent " in capsys.readouterr().out


def test_unavailable_backends_fail_cleanly(capsys: pytest.CaptureFixture[str]) -> None:
    assert main(["info", "--backend", "serial"]) == 1
    assert "M5" in capsys.readouterr().err


def test_sim_backend_reports_its_capabilities(capsys: pytest.CaptureFixture[str]) -> None:
    pytest.importorskip("mujoco", reason="needs the sim extra")
    assert main(["info", "--backend", "sim"]) == 0
    out = capsys.readouterr().out
    assert "backend:      sim" in out
    assert "LEG_TARGET" in out
    # The twin measures its state, so it must not carry the estimated warning.
    assert "MODEL, not measured" not in out


def test_http_backend_reports_a_clear_error_when_no_robot_is_present(
    capsys: pytest.CaptureFixture[str],
) -> None:
    # Port 9 (discard) stands in for "nothing listening".
    assert main(["info", "--backend", "http", "--host", "127.0.0.1:9"]) == 1
    assert "cannot reach robot" in capsys.readouterr().err


def test_play_on_hardware_aborts_without_confirmation(
    monkeypatch: pytest.MonkeyPatch, capsys: pytest.CaptureFixture[str]
) -> None:
    """A real robot must never move because someone hit Enter."""
    monkeypatch.setattr("builtins.input", lambda *_a: "")
    assert main(["play", str(ROUTINES_DIR / "patrol-wifi.yaml"), "--backend", "http"]) == 1
    out = capsys.readouterr().out
    assert "MOVE THE REAL ROBOT" in out
    assert "Aborted" in out


def test_wifi_routine_is_locomotion_only(capsys: pytest.CaptureFixture[str]) -> None:
    assert main(["validate", str(ROUTINES_DIR / "patrol-wifi.yaml")]) == 0
    out = capsys.readouterr().out
    assert "requires: LOCOMOTION" in out


def test_viz_writes_a_png(tmp_path: Path, capsys: pytest.CaptureFixture[str]) -> None:
    out = tmp_path / "stand.png"
    assert main(["viz", "--pose", "stand", "--out", str(out)]) == 0
    assert out.exists() and out.stat().st_size > 0
    assert "wrote" in capsys.readouterr().out


def test_viz_crouch_pose(tmp_path: Path) -> None:
    out = tmp_path / "crouch.png"
    assert main(["viz", "--pose", "crouch", "--out", str(out)]) == 0
    assert out.exists()


def test_teach_repl_records_and_saves_a_routine(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch, capsys: pytest.CaptureFixture[str]
) -> None:
    from robodog.teach.format import load_routine

    out = tmp_path / "nod.yaml"
    script = iter(["cap", "leg fl fr", "y -10", "cap 0.5", "pose stand", "cap 0.5", "save", "quit"])
    monkeypatch.setattr("builtins.input", lambda *_a: next(script))
    assert main(["teach", "nod", "--backend", "mock", "--out", str(out), "--repl"]) == 0
    assert "saved" in capsys.readouterr().out

    routine = load_routine(out)
    assert routine.name == "nod"
    assert len(routine.keyframes) == 3
    assert routine.duration == pytest.approx(1.0)


def test_teach_default_serves_the_web_ui_until_quit(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    """The default teach mode is the browser UI; quitting from the page ends it."""
    import json
    import threading
    import time
    import urllib.request
    import webbrowser

    opened: list[str] = []
    monkeypatch.setattr(webbrowser, "open", lambda url: opened.append(url))

    out = tmp_path / "ui-run.yaml"
    result: list[int] = []
    runner = threading.Thread(
        target=lambda: result.append(
            main(["teach", "ui-run", "--backend", "mock", "--out", str(out)])
        ),
        daemon=True,
    )
    runner.start()

    deadline = time.monotonic() + 10
    while not opened and time.monotonic() < deadline:
        time.sleep(0.02)
    assert opened, "the teach UI never announced its URL"
    url = opened[0]

    with urllib.request.urlopen(url, timeout=5) as page:
        assert "RoboDog Teach" in page.read().decode("utf-8")

    request = urllib.request.Request(
        url.rstrip("/") + "/api/quit",
        data=b"{}",
        method="POST",
        headers={"Content-Type": "application/json"},
    )
    with urllib.request.urlopen(request, timeout=5) as response:
        assert json.loads(response.read())["ok"]

    runner.join(timeout=10)
    assert not runner.is_alive()
    assert result == [0]


def test_no_subcommand_exits_with_usage_error() -> None:
    with pytest.raises(SystemExit) as excinfo:
        main([])
    assert excinfo.value.code == 2
