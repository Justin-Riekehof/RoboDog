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
- **A backend does not know itself until `connect()`.** Its `capabilities`,
  `suggested_watchdog` and `suggested_tick` are answers, not constants: the
  Wi-Fi transport probes the robot for its firmware while connecting, and
  before that it can only report the pessimistic stock set. Reading any of
  them earlier has now shipped two bugs, most recently a teach session that
  E-stopped itself on open with a stock-sized budget.
- **vendor/ is read-only** — pinned upstream reference (MIT), never edited.
- **No hardware in tests.** CI and the default test suite must pass with no
  robot attached, ever. Hardware-touching code is exercised via mock/replay.

## Conventions

- Language: everything in the repo is **English** (code, comments, docs,
  commits). Conversation with the owner is German.
- Python ≥ 3.12, `src/` layout, uv for everything:
  `uv sync --all-groups --all-extras`, `uv run pytest`, `uv run ruff check .`,
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
- **Branch per task, land through a PR.** Several agents work this repo at
  once, so `main` is not a workspace: when the owner asks for a commit, it
  goes on its own branch and lands through a pull request, never straight
  onto `main`. CI runs on every branch, so a push is how you find out within
  minutes whether you broke something another agent depends on. Rebase on
  `main` before asking for a merge.
- **`--all-extras` is not optional** locally, despite the name: without it
  `matplotlib` and `mujoco` are uninstalled and the tests that assert the twin's
  geometry fail on a tree that is perfectly fine. Since M8 that sync also pulls
  `ultralytics` (and torch) for the `vision` extra, which is a couple of
  gigabytes; `uv sync --all-groups --extra viz --extra sim` passes the whole
  suite without it, because nothing in the vision path needs a model to be
  tested.
- CI (`.github/workflows/ci.yml`) runs ruff, ruff format, mypy and pytest on
  **every branch**, so work in parallel gets its own verdict without waiting for
  a merge. Run the same four locally before pushing -- they are what CI runs,
  in the same order.

## Current state

- M0 (offline foundations), M1 (Wi-Fi bring-up, run on the real robot on
  2026-08-11) and the core of M2 (MuJoCo digital twin) are done, as is most of
  M4 (our firmware fork). **M8** (vision-guided behaviours) was built
  2026-08-23 with the robot offline: it is complete and covered headlessly, and
  everything left on it needs hardware (ASSUMPTIONS G1-G5). **M3**
  (sim-to-real calibration) is still open — see ROADMAP for its acceptance list.
- Optional dependencies stay optional: `matplotlib` (`viz` extra), `mujoco`
  (`sim` extra) and `ultralytics` (`vision` extra) must never be imported from a
  mock/http code path, so the no-extras install and CI stay green without them.
  `robodog.vision.yolo` is the only module that touches ultralytics, and only
  `load_detector("yolo")` reaches it.
- The owner's parametric CadQuery leg model (`wavego_leg.py`) is intentionally
  **out of scope for now** (custom part from a repair); `cad/` holds only the
  exported STL.
