"""Wagon Control System - Autonomous Path Following for Differential-Drive Robots

A sophisticated four-layer control architecture for autonomous path following using
noisy sensor data (GPS at 1 Hz, IMU at 20 Hz).

## Architecture Overview

The system implements a hierarchical control pipeline with four layers:

### Layer 1: State Estimation (localizer.py)
Fuses noisy GPS and IMU sensor data using a complementary filter with bias compensation.
- GPS (1 Hz): Position corrections with outlier rejection
- IMU (20 Hz): Continuous state prediction via integration
- Output: Estimated position, heading, and velocity

### Layer 2: Path Following (follower.py)
Generates desired velocity commands to track the reference trajectory using Pure Pursuit.
- Time-based reference tracking (guarantees 20s completion)
- Adaptive lookahead distance (velocity-proportional)
- Output: Reference linear and angular velocities (v_ref, ω_ref)

### Layer 3: Motor Control (motor_controller.py)
Applies PI feedback to correct velocity tracking errors from model mismatch and disturbances.
- Proportional term: Immediate error correction
- Integral term: Eliminates steady-state errors
- Anti-windup: Prevents integral saturation
- Output: Corrected velocity commands (v_cmd, ω_cmd)

### Layer 4: Inverse Kinematics (model.py)
Converts desired robot velocities to individual wheel velocities for differential drive.
- Differential drive model (wheelbase = 0.5m)
- Velocity clamping to hardware limits [-2.0, 2.0] m/s
- Output: Left and right wheel velocities (v_left, v_right)

## Modules

### Core Control Modules
- `config.py` - Centralized configuration parameters with documentation
- `localizer.py` - Complementary filter state estimation
- `follower.py` - Pure Pursuit path following
- `motor_controller.py` - PI velocity feedback control
- `model.py` - Differential drive inverse kinematics
- `path.py` - Reference trajectory generation (Lemniscate of Gerono)

### Communication & Data
- `client.py` - WebSocket client and main control loop
- `data_collector.py` - CSV data logging for all system signals

### Visualization
- `plot_styles.py` - Shared plotting utilities and color scheme
- `visualization.py` - Post-run sensor data visualization
- `diagnostic_plots.py` - Performance analysis (position/heading/velocity errors)
- `live_plot.py` - Real-time trajectory visualization
- `plot_results.py` - CLI for visualization tools

## Quick Start

```python
# Run the complete system with live visualization
from wagon_control.client import main
import asyncio

asyncio.run(main())
```

Or use the command-line interface:
```bash
python -m wagon_control.client
```

## Configuration

All control parameters are centralized in `config.py`:
- Localization: Sensor fusion gains, bias compensation
- Path Following: Lookahead distances, adaptive parameters
- Motor Control: PI feedback gains, anti-windup limits
- Physical: Wheelbase, velocity/acceleration limits

See `docs/APPROACH.md` for detailed design rationale and trade-offs.

## Performance

- **Typical L2 Error**: 12.45m ± 4.96m cumulative over 20s (20-run validated)
  - Average instantaneous deviation: 30-60cm from reference path
  - Normalized: ≈0.62m/s accumulated error
- **Control Frequency**: 20 Hz (50ms update cycle)
- **Completion Time**: 20.0s ± 0.1s
- **Robustness**: Handles GPS noise spikes and initialization errors (2.5m outlier rejection)

## Author

Nishalan Govender

## Version

0.1.0 - Initial implementation
"""

__version__ = "0.1.0"
__author__ = "Nishalan Govender"

# Export key classes for convenience
from .data_collector import DataCollector
from .follower import PurePursuitFollower
from .localizer import WagonLocalizer
from .motor_controller import MotorController

__all__ = [
    "WagonLocalizer",
    "PurePursuitFollower",
    "MotorController",
    "DataCollector",
]
