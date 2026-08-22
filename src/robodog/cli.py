"""`robodog` command-line interface: info, validate, play, teach, viz, bringup, calibrate."""

from __future__ import annotations

import argparse
import contextlib
import sys
from collections.abc import Sequence
from dataclasses import replace
from datetime import date
from pathlib import Path

from robodog import __version__
from robodog.api.client import RobotClient
from robodog.api.types import LegId, RobotState
from robodog.backends.base import Backend, tick_for
from robodog.backends.http import (
    DEFAULT_HOST,
    DEFAULT_TIMEOUT,
    FIRMWARE_CHOICES,
    HttpBackend,
)
from robodog.backends.mock import MockBackend
from robodog.bringup import run_bringup, write_report
from robodog.calibrate import (
    DEFAULT_COARSE_STEP,
    DEFAULT_FINE_STEP,
    DEFAULT_ZERO_STEP,
    parse_direction_arg,
    parse_joints_arg,
    parse_legs_arg,
    run_roll_calibration,
    run_servo_calibration,
    write_servo_report,
)
from robodog.calibrate import write_report as write_calibration_report
from robodog.calibration import (
    DEFAULT_PATH as DEFAULT_CALIBRATION_PATH,
)
from robodog.calibration import (
    format_table,
    load_calibration_or_default,
    save_calibration,
)
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
    firmware: str = "auto",
) -> Backend:
    if name == "mock":
        return MockBackend()
    if name == "http":
        return HttpBackend(host, timeout=timeout, firmware=firmware)
    if name == "sim":
        from robodog.backends.sim import SimBackend

        return SimBackend(viewer=viewer)
    if name == "serial":
        raise BackendError(
            "the serial backend is not built (ROADMAP M7, outlook). Note it would "
            "not help with poses: the firmware's serial parser takes funcMode, "
            "move, ges, light and buzzer -- no per-servo channel at all, unlike "
            "Wi-Fi's sconfig (ASSUMPTIONS D2)"
        )
    raise BackendError(f"unknown backend {name!r}")


def _tick_for(backend: Backend, requested: float | None) -> float:
    """Player tick: what was asked for, else what the backend can keep up with.

    A motion routine interpolated at 50 Hz and sent over a channel that carries
    ten poses a second does not play smoothly -- it plays *long*. Measured on
    the robot: a 3-second bow took fourteen. Matching the tick to the transport
    keeps a routine the length it was written to be.
    """
    return requested if requested is not None else tick_for(backend)


def _format_state(t: float, state: RobotState) -> str:
    front_left = state.leg_targets[LegId.FRONT_LEFT]
    return (
        f"  t={t:6.2f}s drive=({state.drive.forward:+d},{state.drive.turn:+d}) "
        f"body=(p{state.body.pitch:+.1f} y{state.body.yaw:+.1f} r{state.body.roll:+.1f}) "
        f"FL=({front_left.x:6.1f},{front_left.y:6.1f},{front_left.z:6.1f})mm"
    )


def cmd_info(args: argparse.Namespace) -> int:
    backend = make_backend(
        args.backend,
        host=args.host,
        timeout=args.timeout,
        viewer=getattr(args, "viewer", False),
        firmware=getattr(args, "firmware", "auto"),
    )
    with RobotClient(backend) as client:
        print(f"robodog {__version__}")
        print(f"backend:      {client.backend_name}")
        print(f"capabilities: {', '.join(sorted(c.name for c in client.capabilities))}")
        print(f"safety state: {client.safety_state.name}")
        limits = client.limits
        print(
            f"limits:       reach [{limits.plane_depth_min}, {limits.plane_depth_max}] mm "
            f"in the leg plane, x +/-{limits.x_abs_max} mm, "
            f"roll [{limits.roll_min}, {limits.roll_max}] deg, "
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
    if routine.repeat != 1:
        loop = "endless (until stopped)" if routine.repeat == 0 else f"{routine.repeat}x"
        print(f"  repeat:   {loop}")
    if routine.kind == "sequence":
        print(
            f"  moves:    {len(routine.moves)} "
            f"(gap {routine.gap:g}s, {len(routine.steps)} drive steps)"
        )
        for move in routine.moves:
            print(f"    {move.seconds:6.2f}s {move.move}")
    elif routine.kind == "commands":
        print(f"  steps:    {len(routine.steps)}")
    else:
        print(f"  keyframes: {len(routine.keyframes)} (interpolation: {routine.interpolation})")
    return 0


def cmd_play(args: argparse.Namespace) -> int:
    routine = load_routine(args.file)
    # Only the real robot needs a safety prompt and wall-clock pacing; the
    # simulator is neither dangerous nor real-time bound.
    on_hardware = args.backend in ("http", "serial")
    # An endless routine has no duration to quote -- say so instead of lying.
    how_long = (
        "until you interrupt it (Ctrl-C)"
        if routine.repeat == 0
        else f"for {routine.total_duration:.1f}s"
    )
    if on_hardware and not args.yes:
        print(f"'{routine.name}' will MOVE THE REAL ROBOT {how_long}.")
        print("The robot must be on a stand with its legs free; a dropped Wi-Fi")
        print("link cannot be recovered by any command (ASSUMPTIONS D10).")
        if input("Type 'yes' to continue: ").strip().lower() not in ("yes", "y", "j", "ja"):
            print("Aborted; nothing was sent.")
            return 1

    with_viewer = getattr(args, "viewer", False)
    backend = make_backend(
        args.backend,
        host=args.host,
        timeout=args.timeout,
        viewer=with_viewer,
        firmware=getattr(args, "firmware", "auto"),
    )
    # On hardware the timeline must be paced against the wall clock, otherwise
    # 'forward' and 'stop' would be issued milliseconds apart. With the viewer
    # open the reason is different but just as necessary: the simulation runs
    # some thirty times faster than real time, so there would be nothing to see.
    realtime = args.realtime or on_hardware or with_viewer

    # No budget passed: the client asks the backend at connect, which is the
    # first moment a transport that probes for its firmware can answer.
    with RobotClient(backend) as client:
        client.arm()
        if routine.repeat == 1:
            pacing = f"{routine.duration:.2f}s"
        elif routine.repeat == 0:
            pacing = f"{routine.duration:.2f}s per pass, endless"
        else:
            pacing = f"{routine.duration:.2f}s x {routine.repeat} = {routine.total_duration:.2f}s"
        print(f"playing {routine.name!r} ({routine.kind}, {pacing}) on {args.backend}")

        def on_event(event: PlayEvent) -> None:
            print(f"  t={event.t:6.2f}s {event.description}")

        try:
            report = play_routine(
                routine,
                client,
                tick=_tick_for(backend, args.tick),
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


def cmd_calibrate_roll(args: argparse.Namespace) -> int:
    try:
        legs = parse_legs_arg(args.legs)
        directions = parse_direction_arg(args.direction)
    except ValueError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 2
    factory = None
    if args.backend == "mock":
        print("REHEARSAL against the mock backend -- no robot is touched.")

        def factory(_host: str) -> RobotClient:
            return RobotClient(MockBackend())

    report = run_roll_calibration(
        args.host,
        ask=input,
        say=print,
        legs=legs,
        directions=directions,
        coarse_step=args.coarse_step,
        fine_step=args.fine_step,
        client_factory=factory,
    )
    path = write_calibration_report(report, Path(args.report_dir), date.today())
    print("")
    print(f"report written to {path}")
    if report.aborted:
        print(f"run did not complete: {report.aborted}")
        return 1
    print("Next: carry the suggested limits into LimitConfig and update ASSUMPTIONS C13.")
    return 0


def cmd_calibrate_servos(args: argparse.Namespace) -> int:
    """Measure each servo's zero and store it as the versioned offset table."""
    try:
        legs = parse_legs_arg(args.legs)
        joints = parse_joints_arg(args.joints)
    except ValueError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 2

    out = Path(args.out)
    known = load_calibration_or_default(out)

    if args.show:
        print(f"servo zero calibration: {out if out.exists() else 'not measured yet'}")
        if known.measured is not None:
            print(f"measured: {known.measured.isoformat()}")
        if known.reference:
            print(f"reference: {known.reference}")
        for line in format_table(known):
            print(line)
        return 0

    rehearsal = args.backend == "mock"
    factory = None
    if rehearsal:
        print("REHEARSAL against the mock backend -- no robot is touched, and")
        print("the calibration table is NOT written: these numbers measure nothing.")

        def factory(_host: str) -> RobotClient:
            return RobotClient(MockBackend())

    report, calibration = run_servo_calibration(
        args.host,
        ask=input,
        say=print,
        legs=legs,
        joints=joints,
        step=args.step,
        known=known,
        rehearsal=rehearsal,
        client_factory=factory,
    )
    path = write_servo_report(report, Path(args.report_dir), date.today())
    print("")
    print(f"report written to {path}")

    if not report.measured:
        print("nothing was measured; the table is unchanged.")
        return 1 if report.aborted else 0

    if rehearsal:
        print("rehearsal finished; the table was not touched. Drop --backend mock")
        print("when the robot is in front of you.")
        return 0

    calibration = replace(
        calibration,
        measured=date.today(),
        robot=calibration.robot or "wavego-standard-basic",
        reference=calibration.reference or "upper arms vertical, leg plane vertical",
        resolution=args.step,
    )
    written = save_calibration(calibration, out)
    print(f"calibration table written to {written}")
    disagreements = report.disagreements()
    if disagreements:
        print("")
        print(f"{len(disagreements)} joint(s) landed further from their stored value than")
        print(f"the {args.step}-count step this procedure resolves:")
        for result in disagreements:
            print(
                f"  {result.leg.name.lower():<11} {result.joint:<6} "
                f"stored {result.previous:+d}, now {result.offset:+d} "
                f"({result.drift:+d} counts)"
            )
        print("Those zeros are softer than the table's resolution suggests.")
    elif report.remeasured:
        print(f"{len(report.remeasured)} re-measured joint(s) agreed within the step.")
    for line in format_table(calibration):
        print(line)
    if report.aborted:
        print(f"run did not complete: {report.aborted}")
        return 1
    print("Next: `robodog calibrate-servos --show` to review, and commit the table.")
    return 0


def cmd_teach(args: argparse.Namespace) -> int:
    import threading

    from robodog.api.types import Capability
    from robodog.teach.repl import (
        TEACH_VIEWER_WATCHDOG,
        TEACH_WATCHDOG,
        RealtimeTicker,
        TeachRepl,
    )
    from robodog.teach.sequence import SequenceSession
    from robodog.teach.session import TeachSession
    from robodog.teach.webui import serve_teach_ui

    # Over Wi-Fi only the Sequence tab has anything to send (ASSUMPTIONS D2),
    # and everything it sends moves a real robot.
    on_hardware = args.backend in ("http", "serial")
    if on_hardware and args.repl:
        print(
            "error: the text console authors poses, which the stock firmware cannot take "
            "over Wi-Fi; use the web UI (drop --repl) to build drive sequences",
            file=sys.stderr,
        )
        return 2
    if on_hardware and not args.yes:
        print("Sequences started from this UI will MOVE THE REAL ROBOT.")
        print("The robot must be on a stand with its legs free; a dropped Wi-Fi")
        print("link cannot be recovered by any command (ASSUMPTIONS D10).")
        if input("Type 'yes' to continue: ").strip().lower() not in ("yes", "y", "j", "ja"):
            print("Aborted; nothing was sent.")
            return 1

    backend = make_backend(
        args.backend,
        host=args.host,
        timeout=args.timeout,
        viewer=args.viewer,
        firmware=args.firmware,
    )
    # On hardware the transport decides, at connect (see RobotClient.connect).
    # Off it, the budget is about what a stalled loop can cost here instead --
    # see TEACH_WATCHDOG.
    watchdog = None if on_hardware else (TEACH_VIEWER_WATCHDOG if args.viewer else TEACH_WATCHDOG)
    with RobotClient(backend, watchdog_timeout=watchdog) as client:
        client.arm()
        pose_capable = Capability.LEG_TARGET in client.capabilities
        out = Path(args.out) if args.out else None
        folder = out.parent if out is not None else Path("routines")
        session = TeachSession(
            client,
            name=args.name,
            description=args.description,
            default_spacing=args.spacing,
            default_path=out if pose_capable else folder / f"{args.name}.yaml",
        )
        # With no pose tab the sequence is the routine and gets the plain name;
        # next to one it needs its own file, since the two kinds cannot share.
        sequence = SequenceSession(
            name=args.name if not pose_capable else f"{args.name}-moves",
            description=args.description,
            default_path=(
                (out or folder / f"{args.name}.yaml")
                if not pose_capable
                else folder / f"{args.name}-moves.yaml"
            ),
        )
        if pose_capable:
            for leg, message in session.start().items():
                print(f"warning: {leg.name}: {message}")
        else:
            print(f"note: backend {client.backend_name!r} takes no leg targets, so the Pose")
            print("      tab is off. The Sequence tab drives the robot with forward/left/")
            print("      right/backward moves -- keep the Stop button in reach.")

        lock = threading.Lock()
        # The ticker both feeds the watchdog and flushes staged poses, so it is
        # paced by the transport too: at 50 Hz over Wi-Fi it would spend nearly
        # every millisecond inside a request, holding the lock the browser needs.
        ticker = RealtimeTicker(client, lock, tick=client.suggested_tick)
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
                    # Hardware and the viewer both need the timeline paced
                    # against the wall clock, for opposite reasons -- see cmd_play.
                    realtime=args.viewer or on_hardware,
                    port=args.port,
                    open_browser=not args.no_browser,
                    sequence=sequence,
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
    p_info.add_argument(
        "--firmware",
        choices=FIRMWARE_CHOICES,
        default="auto",
        help="which firmware the robot runs; 'auto' probes for it (the stock "
        "firmware answers 500 to the probe, ours answers 200)",
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
    p_play.add_argument(
        "--firmware",
        choices=FIRMWARE_CHOICES,
        default="auto",
        help="which firmware the robot runs; 'auto' probes for it (the stock "
        "firmware answers 500 to the probe, ours answers 200)",
    )
    p_play.add_argument(
        "--tick",
        type=float,
        default=None,
        help="tick interval in seconds (default: what the backend can sustain -- "
        "0.02 in simulation, 0.1 over Wi-Fi)",
    )
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

    p_cal = sub.add_parser(
        "calibrate-roll",
        help="measure a leg's real roll range on the robot (wiggle servo, writes a report)",
    )
    p_cal.add_argument("--host", default=DEFAULT_HOST, help="robot address")
    p_cal.add_argument(
        "--backend",
        choices=("http", "mock"),
        default="http",
        help="http = the real robot; mock = rehearse the procedure with no robot",
    )
    p_cal.add_argument("--legs", default="front_left", help="leg names, comma separated, or 'all'")
    p_cal.add_argument(
        "--direction",
        choices=("both", "up", "down"),
        default="both",
        help="which end to measure: up = outward/folding up, down = inward",
    )
    p_cal.add_argument(
        "--coarse-step",
        type=int,
        default=DEFAULT_COARSE_STEP,
        help=f"PWM counts per coarse step (default {DEFAULT_COARSE_STEP}, about 9 deg)",
    )
    p_cal.add_argument(
        "--fine-step",
        type=int,
        default=DEFAULT_FINE_STEP,
        help=f"PWM counts per fine step (default {DEFAULT_FINE_STEP}, about 0.9 deg)",
    )
    p_cal.add_argument(
        "--report-dir", default=str(BRINGUP_REPORT_DIR), help="where to write the report"
    )
    p_cal.set_defaults(func=cmd_calibrate_roll)

    p_servo = sub.add_parser(
        "calibrate-servos",
        help="measure each servo's zero on the robot and store the offset table (M3)",
    )
    p_servo.add_argument("--host", default=DEFAULT_HOST, help="robot address")
    p_servo.add_argument(
        "--backend",
        choices=("http", "mock"),
        default="http",
        help="http = the real robot; mock = rehearse the procedure with no robot",
    )
    p_servo.add_argument(
        "--legs", default="front_left", help="leg names, comma separated, or 'all'"
    )
    p_servo.add_argument(
        "--joints", default="all", help="fore, back, wiggle (comma separated) or 'all'"
    )
    p_servo.add_argument(
        "--step",
        type=int,
        default=DEFAULT_ZERO_STEP,
        help=f"PWM counts per nudge (default {DEFAULT_ZERO_STEP}, about 2.2 deg)",
    )
    p_servo.add_argument(
        "--out", default=str(DEFAULT_CALIBRATION_PATH), help="calibration table to update"
    )
    p_servo.add_argument(
        "--report-dir", default=str(BRINGUP_REPORT_DIR), help="where to write the report"
    )
    p_servo.add_argument(
        "--show", action="store_true", help="print the stored table and exit (no robot)"
    )
    p_servo.set_defaults(func=cmd_calibrate_servos)

    p_teach = sub.add_parser(
        "teach", help="interactive teach-in: pose the robot, capture keyframes, save a routine"
    )
    p_teach.add_argument("name", help="routine name (lowercase slug, e.g. 'wave')")
    p_teach.add_argument(
        "--backend",
        choices=("sim", "mock", "http"),
        default="sim",
        help="sim = pose against live MuJoCo physics; mock = headless; "
        "http = the real robot over Wi-Fi (drive sequences only, pose teach-in needs M4)",
    )
    p_teach.add_argument("--host", default=DEFAULT_HOST, help="robot address (http backend)")
    p_teach.add_argument(
        "--timeout", type=float, default=DEFAULT_TIMEOUT, help="per-request timeout in seconds"
    )
    p_teach.add_argument(
        "--firmware",
        choices=FIRMWARE_CHOICES,
        default="auto",
        help="which firmware the robot runs; 'auto' probes for it (the stock "
        "firmware answers 500 to the probe, ours answers 200)",
    )
    p_teach.add_argument("--yes", action="store_true", help="skip the hardware safety prompt")
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
