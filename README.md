# MRCBench: Multi-Robot Collaboration Benchmark

A comprehensive MuJoCo-based simulation framework for benchmarking multi-robot collaboration scenarios using Hello Robot Stretch robots. This project provides a high-fidelity simulation environment for developing, testing, and evaluating multi-robot systems.

## Overview

MRCBench extends the [Hello Robot Stretch MuJoCo simulation](https://github.com/hello-robot/stretch_mujoco) to support multi-robot collaboration scenarios. It enables researchers and developers to:

- Simulate multiple Stretch robots in shared environments
- Test collaborative manipulation and navigation tasks
- Benchmark multi-robot coordination algorithms
- Develop and validate multi-agent control policies

## Features

### Core Capabilities

- **Multi-Robot Simulation**: Support for multiple Stretch robots in a single MuJoCo environment
- **High-Fidelity Physics**: Accurate robot dynamics using MuJoCo physics engine
- **Rich Sensor Suite**: 
  - RGB and depth cameras (D435i, D405)
  - Wide-angle navigation camera
  - 2D planar LiDAR
  - Joint encoders and IMU data
- **Flexible Control**: Position control for ranged joints, velocity control for mobile base
- **RoboCasa Integration**: Generate diverse kitchen environments for manipulation tasks
- **Headless Mode**: Run simulations without GUI for faster performance
- **Real-time Visualization**: Interactive viewer with camera feeds and sensor data

### Multi-Robot Features

- **Independent Control**: Control multiple robots simultaneously or independently
- **Shared Environment**: Robots interact in the same physics simulation
- **Collision Detection**: Full physics-based collision handling between robots and objects
- **Synchronized Simulation**: All robots run in a single synchronized physics step

## Installation

### Prerequisites

- Python 3.10 or 3.11
- MuJoCo 3.2.6 (automatically installed as dependency)

### Install from Source

```bash
# Clone the repository with submodules
git clone --recurse-submodules https://github.com/hello-robot/stretch_mujoco.git
cd stretch_mujoco

# Install using uv (recommended)
uv pip install -e .

# Or install using pip
pip install -e .
```

### Optional Dependencies

```bash
# For development
pip install -e ".[dev]"

# For Jupyter notebooks
pip install -e ".[jupyter]"

# For RoboCasa compatibility
pip install -e ".[robocasa]"
```

## Quick Start

### Single Robot Example

```python
from stretch_mujoco import StretchMujocoSimulator
from stretch_mujoco.enums.actuators import Actuators

# Create and start simulator
sim = StretchMujocoSimulator()
sim.start(headless=False)

try:
    # Move the lift
    sim.move_to(Actuators.lift, 0.5)
    sim.wait_until_at_setpoint(Actuators.lift)
    
    # Move the base
    sim.set_base_velocity(v_linear=0.1, omega=0.0)
    
    # Get status
    status = sim.pull_status()
    print(f"Lift position: {status.lift.pos}")
    print(f"Base position: ({status.base.x}, {status.base.y})")
    
finally:
    sim.stop()
```

### Multi-Robot Example

```python
from stretch_mujoco import StretchMujocoSimulator
from stretch_mujoco.enums.actuators import Actuators
from stretch_mujoco.utils import default_scene_xml_path

# Use the multi-robot scene
sim = StretchMujocoSimulator(scene_xml_path=default_scene_xml_path())
sim.start()

try:
    # Control robot 1 (default)
    sim.move_to(Actuators.lift, 0.5)
    
    # Control robot 2 using name-based commands
    from stretch_mujoco.datamodels.status_command import CommandMove
    cmd = sim.data_proxies.get_command()
    cmd.set_move_by(CommandMove(actuator_name="r2_lift", pos=0.5, trigger=True))
    sim.data_proxies.set_command(cmd)
    
finally:
    sim.stop()
```

## Usage Examples

The `examples/` directory contains several demonstration scripts:

### Keyboard Teleoperation

**Single Robot:**
```bash
python examples/keyboard_teleop.py --imagery
```

**Two Robots:**
```bash
python examples/keyboard_teleop_two.py --imagery
```

Controls:
- `W/A/S/D`: Move base
- `T/F/G/H`: Move head (tilt/pan)
- `I/J/K/L`: Move lift and arm
- `O/P`: Wrist yaw
- `C/V`: Wrist pitch
- `E/R`: Wrist roll
- `N/M`: Open/close gripper
- `1/2/0`: Switch between Robot 1, Robot 2, or Both (multi-robot mode)

### Camera Feeds

```bash
python examples/camera_feeds.py
```

Displays all camera streams in real-time:
- D435i RGB and depth (head camera)
- D405 RGB and depth (wrist camera)
- Navigation camera (wide-angle)

### LiDAR Visualization

```bash
python examples/laser_scan.py
```

Shows real-time 2D LiDAR scan visualization.

### RoboCasa Environment

```bash
python examples/robocasa_environment.py --task PnPCounterToCab --layout 0 --style 0
```

Generates and runs a RoboCasa kitchen environment with various manipulation tasks.

### Joint Control

```bash
python examples/move_joints.py
```

Demonstrates position control for all robot joints.

## API Overview

### StretchMujocoSimulator

Main simulator class for interfacing with MuJoCo.

```python
sim = StretchMujocoSimulator(
    scene_xml_path=None,           # Path to scene XML (None = default)
    model=None,                    # Pre-loaded MuJoCo model
    camera_hz=30,                  # Camera update frequency
    cameras_to_use=[],              # List of cameras to enable
)
```

#### Key Methods

**Control:**
- `move_to(actuator, position)`: Move joint to absolute position
- `move_by(actuator, delta)`: Move joint by relative amount
- `set_base_velocity(v_linear, omega)`: Set base translational and rotational velocity
- `wait_until_at_setpoint(actuator)`: Block until joint reaches target
- `wait_while_is_moving(actuator)`: Block while joint is moving
- `stow()`: Move robot to stowed configuration
- `stop()`: Stop simulation

**Data Access:**
- `pull_status()`: Get joint states and base pose
- `pull_camera_data()`: Get camera imagery and intrinsics
- `pull_sensor_data()`: Get LiDAR and other sensor data
- `is_running()`: Check if simulation is active

**Visualization:**
- `add_world_frame(position, rotation)`: Add coordinate frame visualization

### Enums

**Actuators:**
- `Actuators.lift`, `Actuators.arm`
- `Actuators.head_pan`, `Actuators.head_tilt`
- `Actuators.wrist_yaw`, `Actuators.wrist_pitch`, `Actuators.wrist_roll`
- `Actuators.gripper`
- `Actuators.base_translate`, `Actuators.base_rotate`

**Cameras:**
- `StretchCameras.cam_d435i_rgb`, `StretchCameras.cam_d435i_depth`
- `StretchCameras.cam_d405_rgb`, `StretchCameras.cam_d405_depth`
- `StretchCameras.cam_nav_rgb`
- `StretchCameras.all()`: Get all cameras

**Sensors:**
- `StretchSensors.base_lidar`

## Multi-Robot Collaboration

### Scene Configuration

Multi-robot scenarios use a scene XML file that includes multiple robot models:

```xml
<mujoco model="stretch scene">
  <!-- Stretch Robot #1 -->
  <include file="stretch.xml"/>
  
  <!-- Stretch Robot #2 -->
  <include file="stretch2.xml"/>
  
  <!-- Shared environment objects -->
  <worldbody>
    <geom name="floor" type="plane" ... />
    <!-- ... -->
  </worldbody>
</mujoco>
```

### Controlling Multiple Robots

**Robot 1 (Primary):**
Uses the standard API with `Actuators` enum:
```python
sim.move_to(Actuators.lift, 0.5)
sim.set_base_velocity(0.1, 0.0)
```

**Robot 2 (Secondary):**
Uses name-based commands:
```python
from stretch_mujoco.datamodels.status_command import CommandMove

cmd = sim.data_proxies.get_command()
cmd.set_move_by(CommandMove(actuator_name="r2_lift", pos=0.5, trigger=True))
sim.data_proxies.set_command(cmd)
```

### Multi-Robot Actuator Names

Robot 2 actuators follow the pattern `r2_<actuator_name>`:
- `r2_lift`, `r2_arm`
- `r2_head_pan`, `r2_head_tilt`
- `r2_wrist_yaw`, `r2_wrist_pitch`, `r2_wrist_roll`
- `r2_gripper`
- `r2_left_wheel_vel`, `r2_right_wheel_vel` (for differential drive)

## Performance

### Simulation Speed

- **Headless mode**: Fastest, suitable for training and batch evaluation
- **With cameras**: Performance depends on number of enabled cameras
- **With viewer**: Additional overhead for visualization

Monitor simulation speed:
```python
status = sim.pull_status()
print(status.sim_to_real_time_ratio_msg)
```

### Optimization Tips

1. **Disable unused cameras**: Only enable cameras you need
   ```python
   # Instead of StretchCameras.all()
   cameras_to_use = [StretchCameras.cam_d405_rgb]
   ```

2. **Use headless mode** for training:
   ```python
   sim.start(headless=True)
   ```

3. **Reduce camera resolution** in scene XML if needed

## Documentation

- **Getting Started Tutorial**: See `docs/getting_started.ipynb` for a comprehensive introduction
- **API Documentation**: Run `mkdocs serve` to view full documentation
- **Contributing Guide**: See `docs/contributing.md` for development guidelines

## Examples

| Example | Description |
|---------|-------------|
| `keyboard_teleop.py` | Single robot keyboard control |
| `keyboard_teleop_two.py` | Multi-robot keyboard control |
| `camera_feeds.py` | Camera visualization |
| `laser_scan.py` | LiDAR visualization |
| `move_joints.py` | Joint control demonstration |
| `draw_circles.py` | Trajectory following example |
| `gamepad_controller.py` | Gamepad control interface |
| `robocasa_environment.py` | RoboCasa kitchen tasks |

## Architecture

The simulator uses a client-server architecture:

- **Client** (`StretchMujocoSimulator`): Main interface for control and data access
- **Server** (`MujocoServer`): Separate process running MuJoCo physics and rendering
- **Communication**: Shared memory via `multiprocessing.Manager` proxies

This design allows:
- Isolated physics simulation
- Non-blocking control interface
- Efficient data sharing
- Clean process termination

## Requirements

- Python >= 3.10
- MuJoCo == 3.2.6
- NumPy
- OpenCV (for camera processing)
- Matplotlib (for visualization)

See `pyproject.toml` for complete dependency list.

## Contributing

Contributions are welcome! Please see `docs/contributing.md` for guidelines.

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Add tests if applicable
5. Submit a pull request

## License

See `LICENSE` file for details.

## Citation

If you use MRCBench in your research, please cite:

```bibtex
@software{mrcbench2024,
  title={MRCBench: Multi-Robot Collaboration Benchmark},
  author={...},
  year={2024},
  url={https://github.com/hello-robot/stretch_mujoco}
}
```

## Acknowledgments

- Built on [Hello Robot Stretch MuJoCo](https://github.com/hello-robot/stretch_mujoco)
- Uses [MuJoCo](https://mujoco.org/) physics engine
- Integrates with [RoboCasa](https://robocasa.ai/) for manipulation environments

## Support

- **Issues**: [GitHub Issues](https://github.com/hello-robot/stretch_mujoco/issues)
- **Documentation**: See `docs/` directory
- **Community**: [Hello Robot Forums](https://forum.hello-robot.com/)

---

**Note**: This project extends the Hello Robot Stretch MuJoCo simulation for multi-robot collaboration benchmarking. For single-robot use cases, refer to the original [stretch_mujoco](https://github.com/hello-robot/stretch_mujoco) repository.
