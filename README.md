# Bitsody_Robot — Unitree G1 motion authoring and playback

Tooling for authoring humanoid motions in simulation and replaying them on a real
**Unitree G1**, working directly at the low-level DDS control layer
(`LowCmd` / `LowState`) rather than through a high-level motion API — plus a
parametric bipedal walking controller.

The workflow is: pose the robot in MuJoCo → save keyframes to JSON → replay those
keyframes on hardware.

---

## Contents

### `simulate_python/` — main work

| File | What it does |
|---|---|
| `g1_recorder.py` | MuJoCo scene with a tkinter control panel. Enumerates every hinge joint with its real limits, lets you step through them one at a time, and saves the resulting pose as a keyframe into `positions/*.json`. |
| `g1_player.py` | Replays a keyframe file on the robot over `unitree_sdk2py` DDS. Builds `LowCmd_` messages for all 29 motors with per-group PD gains (legs / waist / arms), handles CRC and the motion-switcher handshake before taking low-level control. |
| `walk.py` | Parametric bipedal gait generator — cycle period and stride length are arguments — driving all 29 joints directly rather than calling a locomotion service. |
| `main.py` | CLI front end: `play <file> [--delay ms]` and `walk [--period s] [--stride m]`. |
| `positions/` | Recorded keyframes: `stand`, `t1`–`t3`, `walk_left`, `walk_right`. |
| `g1_test/` | Unitree's official G1 MuJoCo model (23 DOF and 29 DOF variants) with meshes and scenes. |

### `g1-test/` — earlier bring-up sandbox

SDK2 connectivity checks and gamepad input tests written while getting the robot
talking; kept for reference.

---

## Requirements

- Python 3.10 or 3.11
- MuJoCo
- [`unitree_sdk2_python`](https://github.com/unitreerobotics/unitree_sdk2_python)
- CycloneDDS

Full step-by-step installation is in **[SETUP.md](SETUP.md)**.

```bash
pip install -r requirements.txt
```

---

## Usage

Author a pose in simulation:

```bash
cd simulate_python
python g1_recorder.py          # opens the MuJoCo viewer + joint panel
```

Replay it on the robot:

```bash
python main.py play stand.json --delay 100
```

Walk:

```bash
python main.py walk --period 1.2
```

> ⚠️ `g1_player.py` and `walk.py` take **low-level control of a real humanoid**.
> Run them only with the robot on a stand or with a clear fall zone, and make sure
> you can reach the emergency stop.

---

## Attribution

The MuJoCo bridge (`unitree_mujoco.py`, `unitree_sdk2py_bridge.py`) and the G1
model files under `g1_test/` come from Unitree's own
[`unitree_mujoco`](https://github.com/unitreerobotics/unitree_mujoco) project.
The recorder, player, walking controller and CLI are mine.

CycloneDDS and `unitree_sdk2_python` are external dependencies and are installed
rather than vendored into this repository.
