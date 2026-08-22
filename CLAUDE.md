# CLAUDE.md — project context and working rules

## What this is

Software platform for a Waveshare **WAVEGO Standard BASIC** (12-DOF quadruped,
SKU 22615, **no Raspberry Pi**, PWM servos **without position feedback**).
High-level code runs on the owner's Windows PC; the robot's ESP32 is reached
over Wi-Fi (HTTP) today; USB serial is a second transport in the M7 outlook.
Stock firmware for now, a custom fork is **M4**.

Read in this order when context is needed:
[ARCHITECTURE.md](ARCHITECTURE.md) (component cut, capability model, safety),
[ROADMAP.md](ROADMAP.md) (current milestone + acceptance criteria),
[ASSUMPTIONS.md](ASSUMPTIONS.md) (what is unverified about the hardware).

## Hard rules

- **Safety is architecture.** Every command path goes through
  `SafetySupervisor`. Never add a code path that sends to a backend around it.
  Never weaken workspace limits without a corresponding ASSUMPTIONS entry.
- **Assumptions live in ASSUMPTIONS.md**, not in code comments. Code that
  relies on unverified firmware behavior references the entry (e.g. `# see
  ASSUMPTIONS B3`). When hardware testing confirms/refutes one, update the
  status there — never delete entries.
- **The firmware port stays line-faithful.** `robodog.kinematics` mirrors
  [vendor/wavego-firmware/ServoCtrl.h](vendor/wavego-firmware/ServoCtrl.h)
  including quirks (see ASSUMPTIONS C6). Improvements go in separate,
  clearly-named functions, never by silently "fixing" the port.
- **vendor/ is read-only** — pinned upstream reference (MIT), never edited.
- **No hardware in tests.** CI and the default test suite must pass with no
  robot attached, ever. Hardware-touching code is exercised via mock/replay.

## Conventions

- Language: everything in the repo is **English** (code, comments, docs,
  commits). Conversation with the owner is German.
- Python ≥ 3.12, `src/` layout, uv for everything:
  `uv sync --all-groups`, `uv run pytest`, `uv run ruff check .`,
  `uv run ruff format .`, `uv run mypy`, `uv run robodog ...`.
- Fully typed (`mypy` clean, no `# type: ignore` without a reason comment).
- Units: millimeters and degrees at API boundaries (firmware convention);
  seconds for time. Frames/leg numbering as defined in ARCHITECTURE.md.
- **Console output stays ASCII** (`+/-`, `--`): the Windows console is cp1252 and
  mangles en/em dashes and `±`. Docstrings, comments and Markdown files (written
  as UTF-8) may use proper typography.
- Pure logic (kinematics, gait, easing, validation) is dependency-free and
  deterministic; I/O and clocks are injected — tests never sleep.
- Commits: imperative subject, body explains why; reference milestone
  (`M0: ...`) when applicable. Do not commit/push unless the owner asks.

## Current state

- M0 (offline foundations), M1 (Wi-Fi bring-up, run on the real robot on
  2026-08-11) and the core of M2 (MuJoCo digital twin) are done. **M3**
  (sim-to-real calibration) is next — see ROADMAP for its acceptance list.
- Optional dependencies stay optional: `matplotlib` (`viz` extra) and `mujoco`
  (`sim` extra) must never be imported from a mock/http code path, so the
  no-extras install and CI stay green without them.
- The owner's parametric CadQuery leg model (`wavego_leg.py`) is intentionally
  **out of scope for now** (custom part from a repair); `cad/` holds only the
  exported STL.
