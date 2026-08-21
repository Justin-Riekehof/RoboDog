# Contributing

Thanks for your interest. This is a hobby robotics platform for a specific
piece of hardware — a Waveshare WAVEGO (Standard BASIC, 12-DOF, no Raspberry
Pi). Issues and pull requests are welcome, especially from people running the
same robot.

## Ground rules

These are not style preferences; the project's correctness argument depends on
them.

1. **Safety is architecture.** Every command path goes through
   `SafetySupervisor` (E-stop latch, connection watchdog, workspace limits). Do
   not add a code path that reaches a backend around it. Weakening a workspace
   limit requires a corresponding entry in [ASSUMPTIONS.md](ASSUMPTIONS.md)
   explaining why the old limit was wrong.
2. **Unverified hardware behaviour goes in [ASSUMPTIONS.md](ASSUMPTIONS.md)**,
   not in code comments. Code that depends on an unproven firmware property
   references its entry (e.g. `# see ASSUMPTIONS B3`). When hardware testing
   confirms or refutes an assumption, update its status — entries are never
   deleted, because the reasoning that led to a wrong guess is worth keeping.
3. **The firmware port stays line-faithful.** `robodog.kinematics` mirrors
   [vendor/wavego-firmware/ServoCtrl.h](vendor/wavego-firmware/ServoCtrl.h)
   *including its quirks* (ASSUMPTIONS C6, C10, C11). The robot moves the way
   the firmware moves, bugs and all; that is what makes the model predictive.
   Improvements belong in separate, clearly-named functions — never in a silent
   "fix" to the port.
4. **`vendor/` is read-only.** It is a pinned upstream reference copy
   (MIT, Waveshare). Never edit those files.
5. **No hardware in tests.** CI and the default test suite must pass with no
   robot attached, ever. Hardware-touching code is exercised through the mock
   backend or recorded replays.
6. **English everywhere** in the repository — code, comments, docs, commit
   messages.

## Development setup

Requires Python >= 3.12 and [uv](https://docs.astral.sh/uv/).

```console
git clone https://github.com/Justin-Riekehof/RoboDog.git
cd RoboDog
uv sync --all-groups --all-extras
```

Before opening a pull request, all four must pass — this is exactly what CI
runs:

```console
uv run ruff check .
uv run ruff format .
uv run mypy
uv run pytest
```

## Conventions

- Fully typed; `mypy --strict` clean. A `# type: ignore` needs a reason comment.
- Units at API boundaries: **millimeters and degrees** (the firmware's
  convention), seconds for time. Frames and leg numbering are defined in
  [ARCHITECTURE.md](ARCHITECTURE.md).
- **Console output stays ASCII** (`+/-`, `--`). The Windows console is cp1252
  and mangles en/em dashes and `±`. Markdown and docstrings (UTF-8) may use
  proper typography.
- Pure logic — kinematics, gait, easing, validation — is dependency-free and
  deterministic. I/O and clocks are injected, so **tests never sleep**.
- Commits: imperative subject, body explains *why*, referencing the milestone
  (`M2: ...`) where applicable.

## Testing against the real robot

If you have the hardware, [docs/bringup.md](docs/bringup.md) is the guided
procedure. **Put the robot on a stand first.** The stock firmware has no link
watchdog: if Wi-Fi drops mid-command it keeps walking, and no stop command can
reach it.

Bring-up reports and session transcripts live under `docs/bringup/` — a report
from a second WAVEGO would be a genuinely useful contribution, since much of
[ASSUMPTIONS.md](ASSUMPTIONS.md) rests on a sample size of one robot (which has
a repaired, slightly non-standard hind-right leg).
