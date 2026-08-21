# Teach-in: authoring motion routines against the digital twin

`robodog teach` is the software answer to a robot with no servo feedback:
instead of physically guiding the legs (impossible on write-only PWM servos),
you pose the **digital twin** and watch real physics hold — or refuse — every
pose. The result is a plain motion-routine YAML in [routines/](../routines/),
replayable on mock and sim today and on the real robot once M4 (custom
firmware) adds pose-level commands.

## The web UI (default)

```console
uv run robodog teach wave --backend sim --viewer
```

This starts a local page (127.0.0.1 only, the browser opens automatically) and,
with `--viewer`, the MuJoCo window next to it showing live physics.

- **Two draggable views.** Side view (x forward / height) and top view
  (x forward / sideways): grab a foot and pull it where it should go. The
  browser sends world coordinates; all frame conversion, kinematics and safety
  checking stay in Python.
- **Every pose passes the safety supervisor.** A drag into a forbidden,
  unreachable or self-inconsistent pose (ASSUMPTIONS C10/C11) is rejected with
  the supervisor's message in the status bar, and the foot snaps back.
- **Ghost feet** (dashed circles) show where the backend *measured* the foot —
  on the sim backend that is physics, so you see sag and disturbance, not just
  your command.
- **Mirror left/right** applies your drags and nudges symmetrically to the
  paired leg — most poses are symmetric, so this halves the work.
- **Body height slider** moves all four feet together; **Stand/Crouch** buttons
  jump to the known poses; per-leg **±5 mm nudges** for fine trims.
- **Keyframes**: `Capture pose` records the current pose `dt` seconds after the
  previous one; the table offers *Go to* (drive the robot back into a frame)
  and *Delete*; `Preview` replays everything captured so far and returns to
  your working pose; cosine/linear easing is selectable.
- **Save** writes `routines/<name>.yaml` (rename in the text field; overwrite
  needs the checkbox). The file is re-validated through the same parser the
  player uses — a teach session cannot produce a file playback would reject.
- **Quit** ends the session (it warns about unsaved keyframes); Ctrl-C in the
  terminal works too.

The page has no external dependencies and the server binds to localhost only.
While a preview runs, the page shows a busy state and rejects edits.

## The console (`--repl`)

The same session is scriptable as a terminal REPL — useful headless, over SSH,
or in tests: `uv run robodog teach wave --backend mock --repl`. Commands:
`leg fl fr`, `x +5`, `y = 80`, `pose stand`, `cap 0.8`, `undo`, `preview`,
`save`, `quit` — type `help` for the full grammar. Axes per leg: x forward,
y down toward the ground, z outward.

## Replay

```console
uv run robodog play routines/wave.yaml --backend sim --viewer
uv run robodog play routines/wave.yaml --backend mock
```

## Properties worth knowing

- **Everything is validated twice** — live at every drag/nudge, and again when
  saving.
- **The file is the artifact.** Saved routines are ordinary v1 routine files:
  git-diffable, hand-editable, no session state left behind.
- **Physics is the honesty check.** If a pose tips the twin over, you watch it
  fall — information no kinematic editor gives you.
- **Deployment to hardware** waits on M4 (the stock firmware has no pose-level
  commands, ASSUMPTIONS D2) and on M3 calibration for the repaired hind-right
  leg (F1/F3). The routine files stay valid; only the backend changes.
