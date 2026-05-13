# 🤖 Bitsody Robot - G1 Bipedal Walking Setup Guide

## Prerequisites

- **Python 3.8+** (recommended: 3.10 or 3.11)
- **pip** (Python package manager)
- **git** (for cloning)
- **MuJoCo license** (free community license available)
- Linux/Mac (Windows requires WSL2 for ROS2/DDS)

---

## Installation Steps

### 1. Clone Repository (If Not Already Done)

```bash
git clone <repository-url>
cd Bitsody_Robot
```

### 2. Create Virtual Environment (Recommended)

```bash
# Create virtual environment
python3 -m venv venv

# Activate it
source venv/bin/activate          # Linux/Mac
# OR
venv\Scripts\activate             # Windows
```

### 3. Install Dependencies

```bash
# Install from requirements.txt
pip install -r requirements.txt

# Or manually (if issues with above):
pip install unitree_sdk2py>=0.2.0
pip install numpy>=1.24.0
pip install mujoco>=3.1.0
pip install pygame>=2.4.0
```

### 4. Verify Installation

```bash
python3 -c "import unitree_sdk2py; import mujoco; import numpy; print('✓ All imports OK')"
```

---

## File Structure

```
Bitsody_Robot/
├── simulate_python/
│   ├── config.py                 # Configuration (robot, network, timesteps)
│   ├── main.py                   # ✨ Entry point (NEW)
│   ├── walk.py                   # ✨ Walking controller (NEW)
│   ├── g1_player.py              # Motion playback from JSON
│   ├── g1_recorder.py            # Motion recording GUI
│   ├── unitree_mujoco.py         # Physics simulation launcher
│   ├── unitree_sdk2py_bridge.py  # SDK ↔ Simulator bridge
│   │
│   ├── positions/
│   │   ├── stand.json            # ✨ Standing pose (NEW)
│   │   ├── walk_left.json        # ✨ Left swing peak (NEW)
│   │   ├── walk_right.json       # ✨ Right swing peak (NEW)
│   │   ├── positions.json        # Sample poses
│   │   ├── t1.json               # Pre-recorded motion
│   │   └── ...
│   │
│   └── test/
│       ├── gamepad_test.py
│       └── test_unitree_sdk2.py
│
├── unitree_sdk2_python/          # SDK library (git submodule)
├── cyclonedds/                   # DDS middleware
├── requirements.txt              # ✨ Dependencies (NEW)
└── README.md

```

---

## Quick Start

### Option A: Walk Mode (Recommended for Testing)

**Terminal 1** - Start Physics Simulation:
```bash
cd simulate_python
python unitree_mujoco.py
```

**Terminal 2** - Start Walking Controller:
```bash
cd simulate_python
python main.py walk
```

**Expected:**
- MuJoCo viewer opens with robot model
- 5 seconds of warmup (no motion)
- Robot starts walking forward with smooth gait
- Console shows phase info every 0.5s

### Option B: Playback Mode (Test Pre-recorded Motions)

**Terminal 1** - Physics Simulation:
```bash
cd simulate_python
python unitree_mujoco.py
```

**Terminal 2** - Play Motion:
```bash
cd simulate_python
python main.py play stand.json
```

Or with custom delay:
```bash
python main.py play t1.json --delay 50
```

### Option C: Direct walk.py (Debug Mode)

```bash
cd simulate_python
python walk.py
```

---

## Usage Examples

### Walking with Different Gaits

```bash
# Slow & stable (default)
python main.py walk

# Faster gait (0.8s per cycle)
python main.py walk --period 0.8

# Longer stride (0.25m)
python main.py walk --stride 0.25

# Fast AND long strides
python main.py walk --period 0.8 --stride 0.25
```

### Motion Playback

```bash
# Play standing pose
python main.py play stand.json

# Play swing poses
python main.py play walk_left.json
python main.py play walk_right.json

# Play with custom timing
python main.py play positions.json --delay 200
```

---

## Configuration

Edit `simulate_python/config.py` to customize:

```python
ROBOT = "g1"              # Robot model
DOMAIN_ID = 1             # DDS domain (network isolation)
INTERFACE = "lo"          # Network interface (localhost)

SIMULATE_DT = 0.005       # Physics timestep (5ms = 200Hz)
VIEWER_DT = 1/24          # Viewer framerate (24 FPS)

USE_JOYSTICK = 0          # 0=disabled, 1=enabled
JOYSTICK_TYPE = "xbox"    # "xbox" or "switch"
```

---

## Walking Parameters (In walk.py)

Customize in `WalkController()` initialization:

```python
WalkController(
    gait_period=1.2,      # Cycle time (seconds)
    stride_length=0.2,    # Forward distance per step (meters)
    warmup_dur=5.0,       # Initial warmup time (seconds)
)
```

**Balance tuning** (in `BalanceController`):
```python
BalanceController(
    kp_roll=20.0,    # Roll stabilization gain
    kp_pitch=15.0,   # Pitch stabilization gain
)
```

---

## Troubleshooting

### Issue: "ModuleNotFoundError: No module named 'unitree_sdk2py'"

**Solution:**
```bash
pip install unitree_sdk2py
# If still fails, check Python version: python --version (need 3.8+)
```

### Issue: MuJoCo viewer doesn't open

**Solution:**
```bash
# Install OpenGL libs (Linux)
sudo apt-get install libgl1-mesa-glx libxkbcommon-x11-0

# Or set dummy renderer
export MUJOCO_GL=osmesa
```

### Issue: Robot falls/isn't balanced

**Solution:**
In `walk.py`, increase balance gains:
```python
self.balance_ctrl = BalanceController(kp_roll=30.0, kp_pitch=20.0)
```

### Issue: DDS communication error

**Solution:**
```bash
# Verify INTERFACE setting in config.py
INTERFACE = "lo"  # localhost (simulator)
INTERFACE = "eth0" # actual hardware network

# Or check if cyclonedds is available:
python -c "import cyclonedds; print('OK')"
```

---

## Performance Tips

1. **Headless mode** (no viewer) - faster simulation:
   ```bash
   # Comment out viewer in unitree_mujoco.py
   ```

2. **Slower timestep for stability:**
   ```python
   # In walk.py
   gait_period = 1.5  # Slower = more stable
   stride_length = 0.15  # Shorter steps
   ```

3. **Monitor CPU usage:**
   ```bash
   watch -n 0.5 'ps aux | grep python'
   ```

---

## Next Steps

1. ✅ Install dependencies
2. ✅ Run `python main.py walk`
3. 📊 Observe gait in MuJoCo viewer
4. 🎯 Tune parameters (gait_period, stride_length, Kp/Kd)
5. 📈 Experiment with different positions in `positions/` directory

---

## File Dependencies Map

```
main.py
├── config.py              ← Configuration
├── walk.py                ← Bipedal walking
│   ├── numpy              ← Trajectories
│   ├── unitree_sdk2py     ← DDS communication
│   └── config.py
└── g1_player.py           ← Motion playback
    ├── numpy
    ├── unitree_sdk2py
    └── config.py

unitree_mujoco.py
├── mujoco                 ← Physics engine
├── unitree_sdk2py_bridge.py
│   ├── mujoco
│   ├── pygame             ← Joystick (optional)
│   └── unitree_sdk2py
└── config.py
```

---

## Documentation

- **walking mechanism:** `walk.py` docstrings & comments
- **motor control:** `unitree_sdk2py_bridge.py` (PD controller)
- **motion format:** `positions/*.json` examples
- **CLI help:** `python main.py -h`

---

## Support

For issues or questions:
1. Check console output (error messages usually helpful)
2. Verify all imports: `python -c "import <module>"`
3. Ensure MuJoCo is visible: `python -c "import mujoco; print(mujoco.__version__)"`
4. Test with `python main.py play stand.json` first (simpler than walking)

