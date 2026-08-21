"""`robodog` command-line interface: info, validate, play, viz, bringup."""

from __future__ import annotations

import argparse
import contextlib
import sys
from collections.abc import Sequence
from datetime import date
from pathlib import Path

from robodog import __version__
from robodog.api.client import RobotClient
from robodog.api.types import LegId, RobotState
from robodog.backends.base import Backend
from robodog.backends.http import DEFAULT_HOST, DEFAULT_TIMEOUT, HttpBackend
from robodog.backends.mock import MockBackend
from robodog.bringup import run_bringup, write_report
from robodog.errors import BackendError, RobodogError
from robodog.kinematics.poses import crouch_pose, stand_pose
from robodog.teach.format import load_routine
from robodog.teach.player import PlayEvent, play_routine

BACKENDS = ("mock", "http", "sim", "serial")
BRINGUP_REPORT_DIR = Path("docs/bringup")


def make_backend(
    name: str,
    *,
    host: str = DEFAULT_HOST,
    timeout: float = DEFAULT_TIMEOUT,
    viewer: bool = False,
) -> Backend:
    if name == "mock":
        return MockBackend()
    if name == "http":
        return HttpBackend(host, timeout=timeout)
    if name == "sim":
        from robodog.backends.sim import SimBackend

        return SimBackend(viewer=viewer)
    if name == "serial":
        raise BackendError("serial backend arrives with milestone M5")
    raise BackendError(f"unknown backend {name!r}")


def _format_state(t: float, state: RobotState) -> str:
    front_left = state.leg_targets[LegId.FRONT_LEFT]
    return (
        f"  t={t:6.2f}s drive=({state.drive.forward:+d},{state.drive.turn:+d}) "
        f"body=(p{state.body.pitch:+.1f} y{state.body.yaw:+.1f} r{state.body.roll:+.1f}) "
        f"FL=({front_left.x:6.1f},{front_left.y:6.1f},{front_left.z:6.1f})mm"
    )


def cmd_info(args: argparse.Namespace) -> int:
    backend = make_backend(
        args.backend, host=args.host, timeout=args.timeout, viewer=getattr(args, "viewer", False)
    )
    with RobotClient(backend) as client:
        print(f"robodog {__version__}")
        print(f"backend:      {client.backend_name}")
        print(f"capabilities: {', '.join(sorted(c.name for c in client.capabilities))}")
        print(f"safety state: {client.safety_state.name}")
        limits = client.limits
        print(
            f"limits:       y [{limits.height_min}, {limits.height_max}] mm, "
            f"x +/-{limits.x_abs_max} mm, z [{limits.z_min}, {limits.z_max}] mm, "
            f"joints +/-{limits.joint_angle_abs_max} deg"
        )
        state = client.state()
        if state.is_estimated:
            print("NOTE:         state below is a MODEL, not measured -- this")
            print("              transport returns no data at all.")
        print("initial pose (stand):")
        for leg in LegId:
            target = state.leg_targets[leg]
            angles = state.joint_angles[leg]
            print(
                f"  {leg.name:<11} target=({target.x:6.1f},{target.y:6.1f},{target.z:6.1f})mm "
                f"servos=(w{angles.wiggle:+6.2f} f{angles.fore:+6.2f} b{angles.back:+6.2f})deg"
            )
    return 0


def cmd_validate(args: argparse.Namespace) -> int:
    routine = load_routine(args.file)
    print(f"OK: {routine.source}")
    print(f"  name:     {routine.name}")
    print(f"  kind:     {routine.kind}")
    print(f"  requires: {', '.join(sorted(c.name for c in routine.requires))}")
    print(f"  duration: {routine.duration:.2f}s")
    if routine.kind == "commands":
        print(f"  steps:    {len(routine.steps)}")
    else:
        print(f"  keyframes: {len(routine.keyframes)} (interpolation: {routine.interpolation})")
    return 0


def cmd_play(args: argparse.Namespace) -> int:
    routine = load_routine(args.file)
    # Only the real robot needs a safety prompt and wall-clock pacing; the
    # simulator is neither dangerous nor real-time bound.
    on_hardware = args.backend in ("http", "serial")
    if on_hardware and not args.yes:
        print(f"'{routine.name}' will MOVE THE REAL ROBOT for {routine.duration:.1f}s.")
        print("The robot must be on a stand with its legs free; a dropped Wi-Fi")
        print("link cannot be recovered by any command (ASSUMPTIONS D10).")
        if input("Type 'yes' to continue: ").strip().lower() not in ("yes", "y", "j", "ja"):
            print("Aborted; nothing was sent.")
            return 1

    with_viewer = getattr(args, "viewer", False)
    backend = make_backend(args.backend, host=args.host, timeout=args.timeout, viewer=with_viewer)
    # On hardware the timeline must be paced against the wall clock, otherwise
    # 'forward' and 'stop' would be issued milliseconds apart. With the viewer
    # open the reason is different but just as necessary: the simulation runs
    # some thirty times faster than real time, so there would be nothing to see.
    realtime = args.realtime or on_hardware or with_viewer

    with RobotClient(backend) as client:
        client.arm()
        print(
            f"playing {routine.name!r} ({routine.kind}, {routine.duration:.2f}s) on {args.backend}"
        )

        def on_event(event: PlayEvent) -> None:
            print(f"  t={event.t:6.2f}s {event.description}")

        try:
            report = play_routine(
                routine,
                client,
                tick=args.tick,
                realtime=realtime,
                on_event=on_event if args.verbose else None,
                on_sample=lambda t, s: print(_format_state(t, s)),
                sample_interval=args.sample,
            )
        except KeyboardInterrupt:
            print("\ninterrupted -- sending E-stop")
            client.estop("operator interrupt")
            return 1

        hold_viewer = getattr(backend, "run_viewer_until_closed", None)
        if with_viewer and hold_viewer is not None:
            print("routine finished -- close the viewer window to exit (Ctrl-C also works)")
            with contextlib.suppress(KeyboardInterrupt):
                hold_viewer()

        client.disarm()
        print(
            f"done: {report.ticks} ticks, {len(report.events)} events, "
            f"safety={client.safety_state.name}"
        )
    return 0


def cmd_bringup(args: argparse.Namespace) -> int:
    report = run_bringup(
        args.host,
        ask=input,
        say=print,
        include_motion=not args.no_motion,
    )
    if not report.results:
        return 1
    path = write_report(report, Path(args.report_dir), date.today())
    print("")
    print(
        f"summary: {report.confirmed} confirmed, {report.differs} differ, "
        f"{report.skipped} skipped, {report.errors} errors"
    )
    print(f"report written to {path}")
    print("Next: transfer these outcomes into ASSUMPTIONS.md (verified / wrong).")
    return 1 if report.errors else 0


def cmd_teach(args: argparse.Namespace) -> int:
    import threading

    from robodog.teach.repl import RealtimeTicker, TeachRepl
    from robodog.teach.session import TeachSession
    from robodog.teach.webui import serve_teach_ui

    backend = make_backend(args.backend, viewer=args.viewer)
    with RobotClient(backend) as client:
        client.arm()
        session = TeachSession(
            client,
            name=args.name,
            description=args.description,
            default_spacing=args.spacing,
            default_path=Path(args.out) if args.out else None,
        )
        for leg, message in session.start().items():
            print(f"warning: {leg.name}: {message}")

        lock = threading.Lock()
        ticker = RealtimeTicker(client, lock)
        ticker.start()
        try:
            if args.repl:
                repl = TeachRepl(
                    session, client, ask=input, say=print, realtime=args.viewer, lock=lock
                )
                repl.run()
                result = 0
            else:
                result = serve_teach_ui(
                    session,
                    client,
                    lock=lock,
                    realtime=args.viewer,
                    port=args.port,
                    open_browser=not args.no_browser,
                    say=print,
                )
        finally:
            ticker.stop()
            client.disarm()
    return result


def cmd_viz(args: argparse.Namespace) -> int:
    from robodog.viz.stick import render_pose  # matplotlib import stays optional

    if args.pose == "crouch":
        targets = crouch_pose()
    else:
        targets = stand_pose(args.height) if args.height else stand_pose()
    render_pose(
        targets,
        title=f"RoboDog — {args.pose} pose",
        out=args.out,
        show=args.show,
    )
    if args.out:
        print(f"wrote {args.out}")
    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog="robodog", description="WAVEGO robot platform CLI")
    parser.add_argument("--version", action="version", version=f"robodog {__version__}")
    sub = parser.add_subparsers(dest="command", required=True)

    p_info = sub.add_parser("info", help="show backend capabilities and initial state")
    p_info.add_argument("--backend", choices=BACKENDS, default="mock")
    p_info.add_argument("--host", default=DEFAULT_HOST, help="robot address (http backend)")
    p_info.add_argument(
        "--timeout", type=float, default=DEFAULT_TIMEOUT, help="per-request timeout in seconds"
    )
    p_info.set_defaults(func=cmd_info)

    p_validate = sub.add_parser("validate", help="validate a routine file")
    p_validate.add_argument("file")
    p_validate.set_defaults(func=cmd_validate)

    p_play = sub.add_parser("play", help="play a routine on a backend")
    p_play.add_argument("file")
    p_play.add_argument("--backend", choices=BACKENDS, default="mock")
    p_play.add_argument("--host", default=DEFAULT_HOST, help="robot address (http backend)")
    p_play.add_argument(
        "--timeout", type=float, default=DEFAULT_TIMEOUT, help="per-request timeout in seconds"
    )
    p_play.add_argument("--tick", type=float, default=0.02, help="tick interval in seconds")
    p_play.add_argument("--sample", type=float, default=0.5, help="state sample interval")
    p_play.add_argument(
        "--realtime", action="store_true", help="pace against wall time (implied on hardware)"
    )
    p_play.add_argument("--verbose", action="store_true", help="print every sent command")
    p_play.add_argument("--yes", action="store_true", help="skip the hardware safety prompt")
    p_play.add_argument(
        "--viewer", action="store_true", help="open the MuJoCo viewer (sim backend)"
    )
    p_play.set_defaults(func=cmd_play)

    p_bringup = sub.add_parser(
        "bringup", help="guided hardware bring-up over Wi-Fi, writes a report"
    )
    p_bringup.add_argument("--host", default=DEFAULT_HOST, help="robot address")
    p_bringup.add_argument(
        "--no-motion", action="store_true", help="only run checks that do not move the robot"
    )
    p_bringup.add_argument(
        "--report-dir", default=str(BRINGUP_REPORT_DIR), help="where to write the report"
    )
    p_bringup.set_defaults(func=cmd_bringup)

    p_teach = sub.add_parser(
        "teach", help="interactive teach-in: pose the robot, capture keyframes, save a routine"
    )
    p_teach.add_argument("name", help="routine name (lowercase slug, e.g. 'wave')")
    p_teach.add_argument(
        "--backend",
        choices=("sim", "mock"),
        default="sim",
        help="sim = pose against live MuJoCo physics; mock = headless (hardware needs M4)",
    )
    p_teach.add_argument(
        "--viewer", action="store_true", help="open the MuJoCo viewer next to the console"
    )
    p_teach.add_argument("--out", default=None, help="output file (default routines/<name>.yaml)")
    p_teach.add_argument("--description", default="", help="one-line routine description")
    p_teach.add_argument(
        "--spacing", type=float, default=1.0, help="default seconds between keyframes"
    )
    p_teach.add_argument(
        "--repl", action="store_true", help="text console instead of the web UI (scriptable)"
    )
    p_teach.add_argument(
        "--port", type=int, default=0, help="web UI port (default: pick a free one)"
    )
    p_teach.add_argument(
        "--no-browser", action="store_true", help="do not open the browser automatically"
    )
    p_teach.set_defaults(func=cmd_teach)

    p_viz = sub.add_parser("viz", help="render a stick-figure pose (needs viz extra)")
    p_viz.add_argument("--pose", choices=("stand", "crouch"), default="stand")
    p_viz.add_argument("--height", type=float, default=None, help="stand height in mm")
    p_viz.add_argument("--out", default=None, help="write a PNG to this path")
    p_viz.add_argument("--show", action="store_true", help="open an interactive window")
    p_viz.set_defaults(func=cmd_viz)

    return parser


def main(argv: Sequence[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    try:
        result: int = args.func(args)
    except RobodogError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1
    return result


if __name__ == "__main__":
    sys.exit(main())
