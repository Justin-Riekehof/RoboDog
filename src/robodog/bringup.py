"""Guided hardware bring-up over Wi-Fi.

Walks the operator through the unverified protocol assumptions one at a time,
each as a single observable action with a yes/no question, and writes a dated
Markdown report. The point is to turn ASSUMPTIONS section B/D from "read in the
firmware source" into "seen on our robot" — or to catch where reality differs.

Nothing here runs without a human answering, and every motion step is preceded
by its own warning. The robot must be on a stand: over Wi-Fi a dropped link
cannot be recovered by any command we send (ASSUMPTIONS D10).
"""

from __future__ import annotations

from collections.abc import Callable, Iterator
from dataclasses import dataclass, field
from datetime import date
from pathlib import Path

from robodog.api.client import RobotClient
from robodog.api.types import FunctionMode
from robodog.backends.http import AP_PASSWORD, AP_SSID, HttpBackend
from robodog.errors import BackendError, EStopActiveError, RobodogError

Answer = str
Asker = Callable[[str], Answer]
Printer = Callable[[str], None]

# A bring-up is paced by a human reading prompts, so seconds pass between
# commands. The watchdog exists to catch a dead control loop, not a thinking
# operator: it is fed immediately before every action, and given generous head
# room so that an action which needs several HTTP round trips cannot trip it.
BRINGUP_WATCHDOG = 15.0


@dataclass(frozen=True, slots=True)
class Step:
    """One bring-up check: do this, then confirm what you saw."""

    key: str
    assumption: str
    title: str
    instruction: str
    question: str
    action: Callable[[RobotClient], None] | None = None
    motion: bool = False
    # Key of the action step this one observes. If that step did not run, the
    # observation is meaningless and must not be recorded as a finding.
    interprets: str | None = None


@dataclass(slots=True)
class StepResult:
    step: Step
    outcome: str  # "confirmed" | "differs" | "skipped" | "error"
    note: str = ""


@dataclass(slots=True)
class BringupReport:
    host: str
    results: list[StepResult] = field(default_factory=list)

    @property
    def confirmed(self) -> int:
        return sum(1 for r in self.results if r.outcome == "confirmed")

    @property
    def differs(self) -> int:
        return sum(1 for r in self.results if r.outcome == "differs")

    @property
    def skipped(self) -> int:
        return sum(1 for r in self.results if r.outcome == "skipped")

    @property
    def errors(self) -> int:
        return sum(1 for r in self.results if r.outcome == "error")

    def to_markdown(self, today: date) -> str:
        lines = [
            f"# Bring-up report {today.isoformat()}",
            "",
            f"Transport: Wi-Fi / HTTP, host `{self.host}` (stock firmware).",
            "",
            f"Confirmed: {self.confirmed} · Differs: {self.differs} · "
            f"Skipped: {self.skipped} · Errors: {self.errors}",
            "",
            "| Assumption | Check | Outcome | Note |",
            "|---|---|---|---|",
        ]
        for result in self.results:
            note = result.note.replace("|", "\\|") or "—"
            lines.append(
                f"| {result.step.assumption} | {result.step.title} "
                f"| **{result.outcome}** | {note} |"
            )
        lines += [
            "",
            "## What to do with this",
            "",
            "Transfer every `confirmed` row into ASSUMPTIONS.md as "
            "`verified <date>`, and every `differs` row as `wrong` with the "
            "correction — do not delete the original claim.",
        ]
        return "\n".join(lines) + "\n"


def _drive_forward_briefly(client: RobotClient) -> None:
    client.drive(forward=1)


def _stop(client: RobotClient) -> None:
    client.stop()


def _turn_left(client: RobotClient) -> None:
    client.drive(turn=-1)


def _stay_low(client: RobotClient) -> None:
    client.set_function(FunctionMode.STAY_LOW)


def _handshake(client: RobotClient) -> None:
    client.set_function(FunctionMode.HANDSHAKE)


def _middle_pos(client: RobotClient) -> None:
    client.set_function(FunctionMode.MIDDLE_POS)


def steps() -> tuple[Step, ...]:
    """The bring-up checklist, ordered from harmless to increasingly physical."""
    return (
        Step(
            key="reachable",
            assumption="D1",
            title="Robot answers on HTTP",
            instruction=(
                f"Join the robot's Wi-Fi access point (SSID {AP_SSID!r}, "
                f"password {AP_PASSWORD!r}). The connection check already ran."
            ),
            question="Did the connection succeed without you changing anything?",
        ),
        Step(
            key="boot_standup",
            assumption="D7",
            title="Robot stands up on power-on",
            instruction=(
                "Recall what happened when you switched the robot on: the "
                "firmware commands the stand pose about a second into boot, "
                "before Wi-Fi starts."
            ),
            question="Did the legs move into a stand by themselves at power-on?",
        ),
        Step(
            key="control_endpoint",
            assumption="D3",
            title="/control answers with an empty 200",
            instruction=(
                "Two stop commands were sent during connect. No response body "
                "is expected -- the firmware never returns data."
            ),
            question="Did that complete without an error message?",
        ),
        Step(
            key="move_forward",
            assumption="B3/D4",
            title="move=1 starts a forward gait",
            instruction=(
                "The robot will start WALKING and keep walking until the next "
                "step stops it. Legs must hang free."
            ),
            question="Did the robot start a forward walking gait?",
            action=_drive_forward_briefly,
            motion=True,
        ),
        Step(
            key="move_latches",
            assumption="B3/D4",
            title="Motion latches until an explicit stop",
            instruction="Watch the robot: no stop command has been sent yet.",
            question="Did it keep walking on its own, without further commands?",
            interprets="move_forward",
        ),
        Step(
            key="stop",
            assumption="D4",
            title="move=3 + move=6 stop the gait",
            instruction="Both axes are being stopped now.",
            question="Did the robot stop and settle into a still stand?",
            action=_stop,
            motion=True,
            interprets="move_forward",
        ),
        Step(
            key="turn",
            assumption="B3/D4",
            title="move=2 turns in place",
            instruction="The robot will turn left in place until stopped.",
            question="Did it turn left in place (not walk forward)?",
            action=_turn_left,
            motion=True,
        ),
        Step(
            key="turn_stop",
            assumption="D4",
            title="Turning stops independently",
            instruction="Stopping both axes again.",
            question="Did the turn stop?",
            action=_stop,
            motion=True,
            interprets="turn",
        ),
        Step(
            key="func_stay_low",
            assumption="B4",
            title="funcMode=2 runs the stay-low animation",
            instruction="The robot will crouch and rise again -- one shot.",
            question="Did it crouch and come back up by itself?",
            action=_stay_low,
            motion=True,
        ),
        Step(
            key="func_blocking",
            assumption="B8",
            title="Function animations are blocking",
            instruction=(
                "A handshake will start; it takes about four seconds. While it "
                "runs, try clicking Forward in the robot's own web UI."
            ),
            question="Was the robot unresponsive to commands until it finished?",
            action=_handshake,
            motion=True,
        ),
        Step(
            key="middle_pos",
            assumption="C12/D6",
            title="funcMode=9 moves all servos to the calibrated middle",
            instruction=(
                "All twelve servos go to their stored middle position. The legs "
                "will straighten and the body sits HIGHER than the walking "
                "envelope -- support the robot."
            ),
            question="Did all legs move to a straight/neutral position?",
            action=_middle_pos,
            motion=True,
        ),
        Step(
            key="no_watchdog",
            assumption="B9/D10",
            title="No link watchdog in the firmware",
            instruction=(
                "SAFETY TEST, do this with the robot lifted or on a stand, and "
                "read it fully first:\n"
                "  1. Start a forward walk from the robot's own web UI.\n"
                "  2. Switch off your PC's Wi-Fi without stopping the robot.\n"
                "  3. Watch what the robot does -- it has no watchdog, so it "
                "should just keep going.\n"
                "  4. Stop it from the web UI (or cut its power), then switch "
                "your Wi-Fi back ON and rejoin the robot's access point.\n"
                "Answer only once you are reconnected: this tool needs the link "
                "to put the robot into its safe state at the end."
            ),
            question="Did the robot keep walking after the link was gone?",
        ),
    )


def run_bringup(
    host: str,
    *,
    ask: Asker,
    say: Printer,
    client_factory: Callable[[str], RobotClient] | None = None,
    include_motion: bool = True,
) -> BringupReport:
    """Run the checklist interactively and return the report."""
    report = BringupReport(host=host)

    say("")
    say("=" * 72)
    say(" WAVEGO bring-up over Wi-Fi (stock firmware)")
    say("=" * 72)
    say("")
    say(" SAFETY, read before continuing:")
    say("  * Put the robot ON A STAND with the legs hanging free, or keep a")
    say("    hand on the power switch. Over Wi-Fi a dropped link cannot be")
    say("    recovered: no stop command reaches the robot (ASSUMPTIONS D10).")
    say("  * The firmware has no watchdog. Whatever it was last told to do, it")
    say("    keeps doing.")
    say("  * Ctrl-C triggers an E-stop attempt, but it can only work while the")
    say("    connection is alive.")
    say("")
    if ask("Type 'yes' when the robot is secured and you are ready: ").strip().lower() not in (
        "yes",
        "y",
        "j",
        "ja",
    ):
        say("Aborted; nothing was sent.")
        return report

    factory = client_factory if client_factory is not None else _default_client_factory
    client = factory(host)

    try:
        client.connect()
    except RobodogError as exc:
        say(f"\nCannot reach the robot: {exc}")
        say("Check that you are on the robot's access point and that the host is right.")
        report.results.append(
            StepResult(
                step=steps()[0],
                outcome="error",
                note=str(exc),
            )
        )
        return report

    try:
        client.arm()
        outcomes: dict[str, str] = {}
        for step in _selected(steps(), include_motion):
            result = _run_step(step, client, ask=ask, say=say, outcomes=outcomes)
            outcomes[step.key] = result.outcome
            report.results.append(result)
    except KeyboardInterrupt:
        say("\nInterrupted -- attempting E-stop.")
        _try_estop(client, say)
        raise
    finally:
        _try_estop(client, say)
        client.disconnect()

    return report


def _selected(all_steps: tuple[Step, ...], include_motion: bool) -> Iterator[Step]:
    for step in all_steps:
        if step.motion and not include_motion:
            continue
        yield step


def _run_step(
    step: Step,
    client: RobotClient,
    *,
    ask: Asker,
    say: Printer,
    outcomes: dict[str, str],
) -> StepResult:
    say("")
    say("-" * 72)
    say(f"[{step.assumption}] {step.title}")
    say(f"  {step.instruction}")

    if step.interprets is not None and outcomes.get(step.interprets) != "confirmed":
        say(f"  Skipping: the step it interprets ({step.interprets}) did not run, so")
        say("  whatever you see now says nothing about this assumption.")
        return StepResult(
            step=step,
            outcome="skipped",
            note=f"invalid without '{step.interprets}', which did not run",
        )

    if step.motion:
        say("  >>> THIS MOVES THE ROBOT <<<")
        answer = ask("  Press Enter to run it, or 's' to skip: ").strip().lower()
        if answer == "s":
            return StepResult(step=step, outcome="skipped", note="operator skipped")
    if step.action is not None:
        try:
            _act(step, client, say=say)
        except RobodogError as exc:
            say(f"  command failed: {exc}")
            return StepResult(step=step, outcome="error", note=str(exc))
    verdict = ask(f"  {step.question} [y/n/s] ").strip().lower()
    if verdict in ("y", "yes", "j", "ja"):
        return StepResult(step=step, outcome="confirmed")
    if verdict in ("n", "no", "nein"):
        note = ask("  What happened instead? ").strip()
        return StepResult(step=step, outcome="differs", note=note)
    return StepResult(step=step, outcome="skipped", note="not observed")


def _act(step: Step, client: RobotClient, *, say: Printer) -> None:
    """Run one step's command, absorbing the operator's thinking time.

    The watchdog is fed right here, so the seconds spent reading the prompt do
    not count as a dead control loop. If a previous step nevertheless latched
    the E-stop, recover once instead of failing every remaining check.
    """
    assert step.action is not None
    client.heartbeat()
    try:
        step.action(client)
    except EStopActiveError:
        say("  E-stop was latched -- releasing it and retrying this step.")
        client.reset()
        client.arm()
        client.heartbeat()
        step.action(client)


def _try_estop(client: RobotClient, say: Printer) -> None:
    try:
        client.estop("bring-up finished")
    except (RobodogError, BackendError) as exc:
        say(f"  WARNING: could not confirm the stop: {exc}")
        say("  If the robot is still moving, stop it from its web UI or cut its power.")


def _default_client_factory(host: str) -> RobotClient:
    return RobotClient(HttpBackend(host), watchdog_timeout=BRINGUP_WATCHDOG)


def write_report(report: BringupReport, directory: Path, today: date) -> Path:
    """Write the report, never overwriting an earlier run.

    Several sessions a day are normal — the first attempt often fails and gets
    repeated. Reports are the evidence behind every `verified` status, so a
    same-day re-run must not silently replace its predecessor.
    """
    directory.mkdir(parents=True, exist_ok=True)
    stem = f"bringup-{today.isoformat()}"
    path = directory / f"{stem}.md"
    run = 2
    while path.exists():
        path = directory / f"{stem}-run{run}.md"
        run += 1
    path.write_text(report.to_markdown(today), encoding="utf-8")
    return path
