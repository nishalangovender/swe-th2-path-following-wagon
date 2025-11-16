# Wagon Path Following Control System

A sophisticated four-layer control system for autonomous path following of a differential-drive wagon using noisy sensor data.

## Overview

This project implements an **autonomous control system** for a differential-drive wagon that follows a Lemniscate of Gerono path. The system features a hierarchical four-layer architecture: **state estimation** (complementary filter), **path following** (Pure Pursuit), **motor control** (PI feedback), and **inverse kinematics**.

The system processes noisy sensor data (GPS at 1 Hz, IMU at 20 Hz), fuses them for state estimation, and generates wheel velocity commands to achieve accurate path tracking with typical errors of 0.15-0.30m.

## Architecture

**Four-Layer Control Pipeline:**

1. **State Estimation** (localizer.py) - Complementary filter fusing GPS and IMU with bias compensation
2. **Path Following** (follower.py) - Pure Pursuit with adaptive lookahead distance
3. **Motor Control** (motor_controller.py) - PI feedback for velocity tracking
4. **Inverse Kinematics** (model.py) - Differential drive model (wheelbase = 0.5m)

See [`docs/APPROACH.md`](docs/APPROACH.md) for detailed architecture explanation, design trade-offs, and generalization analysis.

## Features

- **Autonomous Path Following**: Tracks Lemniscate of Gerono reference path for 20 seconds
- **Sensor Fusion**: Complementary filter combines GPS (1 Hz) and IMU (20 Hz) with outlier rejection
- **Adaptive Control**: Velocity-dependent lookahead and PI feedback for robust tracking
- **Real-time Visualization**: Live trajectory plots with reference path overlay
- **Diagnostic Analysis**: Post-run position/heading/velocity error analysis
- **Centralized Configuration**: All control parameters documented in `config.py`

## Requirements

- Python 3.7+
- Dependencies listed in `requirements.txt`

## Setup

The system uses a virtual environment for dependency management. Setup is automated via the run script:

```bash
./run.sh
```

This will:
1. Check for Python 3
2. Create a virtual environment (if not exists)
3. Install dependencies
4. Run the wagon control system

### Manual Setup

If you prefer manual setup:

```bash
# Create virtual environment
python3 -m venv venv

# Activate virtual environment
source venv/bin/activate  # On Linux/Mac
# or
venv\Scripts\activate     # On Windows

# Install dependencies
pip install -r requirements.txt
```

## Usage

### Running the Control System

The main runner script `run.sh` handles the complete workflow. **By default, it shows real-time live visualization as data is collected.**

```bash
# Run with live real-time visualization (DEFAULT)
./run.sh

# Run multiple sequential experiments (live plots auto-disabled)
./run.sh 5                    # 5 consecutive runs for statistical analysis
./run.sh 10 --verbose         # 10 runs with verbose logging

# Run without live plotting, show static plots after collection
./run.sh --no-live

# Run data collection only (skip all visualization)
./run.sh --no-plot

# Only visualize the most recent run (no new collection)
./run.sh --plot-only

# Run in verbose mode with live visualization
./run.sh --verbose

# Show all options
./run.sh --help
```

#### Sequential Runs

When running multiple experiments, specify the number of runs as the first argument:

```bash
./run.sh 5    # Runs the control system 5 times sequentially
```

**Features:**
- Live plots are automatically disabled to avoid manual plot closing
- Each run saves to a separate timestamped directory in `results/`
- Progress indicators show "Run X of Y" status
- 2-second pause between runs to allow system reset
- Useful for gathering statistical data on control performance

**Example Output:**
```
[Sequential Mode] Running 5 times (live plots disabled)
--- Run 1 of 5 ---
--- Run 2 of 5 ---
...
```

#### Live Visualization

The default mode launches a real-time plot window that updates as data is collected:
- **GPS Trajectory**: Shows the wagon's path with a color gradient indicating time progression, overlaid with the reference path (dashed yellow-orange line)
- **Accelerometer**: X and Y acceleration plotted over time
- **Gyroscope**: Angular velocity plotted over time

The plots update automatically every 500ms. When data collection completes (~20 seconds), the live plot remains open for inspection. Close the plot window when you're done.

The system will:
- Connect to the WebSocket server
- Send velocity commands
- Collect sensor data for ~20 seconds
- Save data to `results/run_YYYYMMDD_HHMMSS/`
- Display the L2 score
- (Optional) Visualize the results

### Manual Execution

You can also run components individually:

```bash
# Run data collection only
python -m wagon_control.client
python -m wagon_control.client --verbose

# Visualize results
python -m wagon_control.plot_results              # Most recent run
python -m wagon_control.plot_results --run run_20251114_184704  # Specific run
python -m wagon_control.plot_results --save       # Save plots as PNG
python -m wagon_control.plot_results --list       # List available runs
```

The visualization tool generates:
- GPS trajectory plot (x vs y position with time gradient)
- IMU data plots (accelerometer and gyroscope over time)

## Project Structure

```
wagon_control/              # Main Python package
├── config.py              # Centralized configuration parameters
├── model.py               # Inverse kinematics and robot parameters
├── path.py                # Reference path definition (Lemniscate of Gerono)
├── localizer.py           # State estimation (complementary filter)
├── follower.py            # Path following (Pure Pursuit)
├── motor_controller.py    # Velocity feedback control (PI)
├── client.py              # WebSocket client and control loop
├── data_collector.py      # CSV data logging
├── plot_styles.py         # Shared visualization utilities
├── visualization.py       # Post-run plotting (GPS, IMU)
├── diagnostic_plots.py    # Performance analysis plots
├── live_plot.py           # Real-time visualization
└── plot_results.py        # CLI for visualization

docs/                      # Documentation
├── APPROACH.md            # System design and trade-offs
├── ASSIGNMENT.md          # Original assignment details
└── images/                # Architecture diagrams

results/                   # Collected run data (timestamped)
└── run_YYYYMMDD_HHMMSS/
    ├── imu_data.csv       # IMU sensor data
    ├── gps_data.csv       # GPS measurements
    ├── state_data.csv     # Localized state estimates
    ├── reference_data.csv # Reference trajectory
    └── motor_data.csv     # Motor controller diagnostics
```

## Testing

### Quick Test

```bash
./run.sh
```

Expected behavior:
- System initializes from first GPS reading
- Pure Pursuit controller follows Lemniscate of Gerono path
- Runs for 20 seconds with real-time control updates
- Score: L2 distance metric displayed at completion
- Typical L2 error: 0.15-0.30m mean tracking error

### Statistical Validation

To validate control parameter robustness across different noise realizations:

```bash
./run.sh 10    # Collect 10 datasets for statistical analysis
```

This is useful for:
- Computing mean and variance of tracking performance
- Testing parameter sensitivity across multiple runs
- Establishing baseline performance metrics

### Configuration Tuning

All control parameters are centralized in `wagon_control/config.py`:

```python
# Localization parameters
LOCALIZER_VELOCITY_CORRECTION_GAIN = 0.45
LOCALIZER_GYRO_BIAS = 0.015
LOCALIZER_ACCEL_X_BIAS = 0.096

# Path following parameters
FOLLOWER_BASE_LOOKAHEAD = 0.8
FOLLOWER_LOOKAHEAD_TIME = 0.7

# Motor controller gains
MOTOR_KP_V = 0.5
MOTOR_KI_V = 0.08
MOTOR_KP_OMEGA = 0.5
MOTOR_KI_OMEGA = 0.06
```

See `config.py` for detailed parameter documentation and tuning rationale.

## Development

### Code Quality Tools

This project uses modern Python tooling for code quality:

- **black**: Code formatting
- **ruff**: Fast linting
- **mypy**: Static type checking

Install development dependencies:

```bash
pip install -r requirements-dev.txt
```

Run code quality checks:

```bash
# Format code
black wagon_control/

# Lint code
ruff check wagon_control/

# Type check
mypy wagon_control/
```

### Project Configuration

Code quality settings are defined in `pyproject.toml`.
