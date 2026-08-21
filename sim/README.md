# Simulation assets

The MuJoCo model of the WAVEGO is **not stored here — it is generated** at
runtime by [`robodog.sim.model`](../src/robodog/sim/model.py) from the linkage
dimensions in [`kinematics/constants.py`](../src/robodog/kinematics/constants.py),
so it cannot drift away from the firmware port that shares those constants.

To export a copy for inspection in an external viewer:

```python
from robodog.sim.model import write_mjcf

write_mjcf("sim/wavego.xml")
```

This directory holds exported models and any future meshes. Higher-fidelity leg
meshes may later come from the parametric CadQuery leg model (ROADMAP M6).

Two limitations of the twin are deliberate and documented in
[ASSUMPTIONS.md](../ASSUMPTIONS.md):

- **E4** — each leg is a serial hip-roll/hip-pitch/knee stand-in for the real
  closed five-bar linkage. Foot positions match the ported kinematics exactly,
  so gait geometry and stability transfer; mass distribution inside the leg, and
  therefore absolute servo loads, do not.
- **E1/E3/E5** — masses, inertias, centre of mass and the trunk/hip layout are
  approximated from product dimensions rather than measured.
