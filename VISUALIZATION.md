# Visualization System Documentation

This document explains the visualization scripts used in the wagon control system and how to use them.

## Overview

The visualization system provides real-time and post-run analysis of the wagon's path-following performance. It displays trajectory tracking, sensor data, control commands, and error metrics.

## Active Visualization Scripts

### 1. Live Plot (`wagon_control/live_plot.py`)

**Purpose**: Real-time visualization during data collection runs.

**Features**:
- **GPS Trajectory Plot**: Shows planned reference path (REF), sparse GPS measurements (GPS), and localized trajectory (LOC)
- **Velocity Plot**: Compares commanded velocity (REF), motor controller output (CMD), and actual velocity (LOC)
- **Angular Velocity Plot**: Compares commanded angular velocity (REF), motor controller output (CMD), and actual angular velocity (LOC)
- **Position Errors Plot**: X and Y position errors over time
- **Heading Plot**: Heading comparison between reference and localized values
- **Tracking Metrics Overlay**: Displays L2 error, average error, and difference metrics

**How to Use**:
```bash
# Live plot is enabled by default when running a single test
./run.sh

# Disable live plotting (useful for batch runs)
./run.sh --no-live

# Run multiple tests without live plotting
./run.sh 10
```

**Output Files**:
- `results/run_YYYYMMDD_HHMMSS/live_plot_snapshot.png` - Saved screenshot of the final live plot state

**Data Sources**:
- `state_data.csv` - Localized position and velocity
- `reference_data.csv` - Time-based reference trajectory
- `gps_data.csv` - GPS measurements (1 Hz)
- `motor_data.csv` - Motor controller commands
- `tracking_metrics.csv` - Position errors
- `score.txt` - Final L2 error score

### 2. Post-Run Visualization (`wagon_control/visualization.py`)

**Purpose**: Generate static plots after data collection completes.

**Features**:
- **IMU Data Plot**: Raw accelerometer (x_dot, y_dot) and gyroscope (theta_dot) data

**How to Use**:
```bash
# View latest run's plots
python -m wagon_control.plot_results

# View specific run
python -m wagon_control.plot_results --run run_20251114_184704

# Save plots without displaying
python -m wagon_control.plot_results --save --no-show

# List all available runs
python -m wagon_control.plot_results --list
```

**Output Files**:
- `results/run_YYYYMMDD_HHMMSS/imu_data.png` - Raw IMU sensor data plot

**Note**: When running multiple tests (e.g., `./run.sh 10`), IMU plots are saved but not displayed to avoid blocking execution.

### 3. Plot Results Script (`wagon_control/plot_results.py`)

**Purpose**: Command-line interface to the post-run visualization system.

**Key Options**:
- `--run <name>` - Specify which run to visualize
- `--save` - Save plots to PNG files
- `--no-show` - Don't display plots (useful for batch processing)
- `--list` - List all available run directories

## Deprecated/Diagnostic Scripts

The following scripts are available but not part of the main workflow:

### `wagon_control/diagnostic_plots.py`
**Purpose**: Detailed diagnostic analysis with 4 separate plots:
1. XY trajectory comparison (REF vs GPS vs LOC)
2. Position error over time
3. Heading comparison and error
4. Velocity magnitude comparison

**Usage**: Not called automatically. Can be imported and used for detailed analysis.

### `wagon_control/plot_styles.py`
**Purpose**: Shared styling utilities and helper functions for all visualization scripts.

**Key Features**:
- Monumental brand color scheme (dark mode)
- Custom colormap (orange → blue gradient)
- Consistent axis styling
- CSV data loading utilities

## Data Flow Diagram

```
run.sh
  │
  ├─> [LIVE MODE] (single run)
  │     ├─> Launch live_plot.py (background)
  │     │     └─> Updates every 500ms with new data
  │     │
  │     ├─> Run client.py (data collection)
  │     │     └─> Logs to CSV files in results/run_YYYYMMDD_HHMMSS/
  │     │         └─> Terminal output shows simplified final results
  │     │
  │     └─> After plot window closed:
  │           └─> plot_results.py --save --no-show
  │                 └─> Generates imu_data.png (saved, not displayed)
  │
  └─> [NO-LIVE MODE] (multiple runs or --no-live)
        ├─> Run client.py (data collection)
        │     └─> Terminal output shows simplified final results
        │
        └─> plot_results.py (if --plot-after enabled)
              └─> Generates imu_data.png
```

## Understanding the Metrics

### Live Plot Metrics Overlay

The live plot displays tracking metrics in a simple format:
```
L2: 6.792m  Avg: 339.6mm
LOC L2: 93.120m  LOC Avg: 233.4mm  Diff: 106.2mm
```

- **L2**: Cumulative L2 error based on localization
- **Avg**: Average error per sample (in millimeters)
- **LOC L2**: System score from ground truth (comparable 20s value)
- **LOC Avg**: Average error per sample from ground truth
- **Diff**: Difference between localization estimate and ground truth

### Terminal Final Results

After each run completes, the terminal displays the same simplified format:
```
L2: 6.792m  Avg: 339.6mm
LOC L2: 93.120m  LOC Avg: 233.4mm  Diff: 106.2mm
```

## Legend Labels

All plots use consistent, simplified legend labels:

| Label | Meaning |
|-------|---------|
| REF   | Reference/planned trajectory or command from path follower |
| GPS   | GPS measurements (1 Hz sampling) |
| LOC   | Localized trajectory from Extended Kalman Filter (20 Hz) |
| CMD   | Motor controller output |

## File Locations

- **Live plotting**: `wagon_control/live_plot.py`
- **Post-run plots**: `wagon_control/visualization.py`
- **CLI tool**: `wagon_control/plot_results.py`
- **Plot styles**: `wagon_control/plot_styles.py`
- **Diagnostics**: `wagon_control/diagnostic_plots.py` (not actively used)

## Customization

### Changing Plot Colors

Colors are defined in `wagon_control/config.py` using the Monumental brand palette:
- `MONUMENTAL_BLUE` - Reference/planned paths
- `MONUMENTAL_ORANGE` - GPS and errors
- `MONUMENTAL_YELLOW_ORANGE` - Motor commands
- `#00ff00` (green) - Localized trajectory

### Adjusting Update Rate

The live plot updates every 500ms by default. To change this, modify the `interval` parameter in `live_plot.py:862`:
```python
ani = animation.FuncAnimation(self.fig, self._update, interval=500, blit=False)
```

### Modifying Metrics Display

The metrics overlay is generated in `live_plot.py:592-612`. To customize the format, edit the `metrics_lines` list construction.

## Troubleshooting

### Live plot window doesn't close automatically
- **Expected behavior**: You must manually close the plot window after data collection completes
- **Why**: Allows you to inspect the final trajectory before the window closes

### IMU plots appear after multi-run tests
- **Fixed**: IMU plots are now saved but not displayed when running multiple tests
- **Location**: Find saved plots in `results/run_YYYYMMDD_HHMMSS/imu_data.png`

### Plots appear blank or incomplete
- **Cause**: CSV files may be empty or corrupted
- **Solution**: Check that the wagon is properly connected and transmitting data

### Missing data in plots
- **Cause**: Some CSV files may not exist if that data source failed
- **Solution**: Check logs for sensor connection issues
