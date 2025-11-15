# Wagon Path Following Control System

A differential-drive wagon control system for path following using IMU and GPS sensor data.

## Overview

This project implements a simulated control system for a differential-drive wagon. The system connects to a websocket server, receives noisy sensor data (accelerometer, gyroscope, GPS), and aims to control the wagon to follow a specified path.

## Features

- **Inverse Kinematics**: Converts linear velocity (`v_cmd`) and angular velocity (`omega_cmd`) into wheel velocities using differential drive model (wheelbase = 0.5m)
- **WebSocket Control**: Real-time communication with wagon server
- **Sensor Data Collection**: IMU (accelerometer, gyroscope) and GPS data
- **Live Visualization**: Real-time trajectory and sensor plots

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

#### Live Visualization

The default mode launches a real-time plot window that updates as data is collected:
- **GPS Trajectory**: Shows the wagon's path with a color gradient indicating time progression
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
wagon_control/          # Main Python package
├── model.py           # Inverse kinematics and robot parameters
├── client.py          # WebSocket client and data collection
├── visualization.py   # Plotting utilities for sensor data
├── live_plot.py       # Real-time visualization
└── plot_results.py    # Post-processing visualization

docs/                  # Documentation
├── PLAN.md            # Development roadmap
├── ASSIGNMENT.md      # Assignment details
└── images/            # Assignment diagrams

results/               # Collected run data (timestamped)
└── run_YYYYMMDD_HHMMSS/
    ├── imu_data.csv
    └── gps_data.csv
```

## Testing

### Quick Test

```bash
./run.sh
```

Expected output with current test parameters (`v_cmd=0.75 m/s`, `omega_cmd=0.5 rad/s`):
- Console: `Sending velocities: v_cmd=0.75 m/s, omega_cmd=0.5 rad/s → v_left=0.625 m/s, v_right=0.875 m/s`
- Trajectory: Curved path turning left (counter-clockwise)
- Score: L2 distance metric after ~20 seconds

### Testing Different Motions

Edit `wagon_control/client.py` lines 84-85:

```python
# Straight motion
TEST_V_CMD: float = 1.0
TEST_OMEGA_CMD: float = 0.0

# Sharp turn
TEST_V_CMD: float = 0.5
TEST_OMEGA_CMD: float = 1.0

# Spin in place
TEST_V_CMD: float = 0.0
TEST_OMEGA_CMD: float = -1.0
```

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
