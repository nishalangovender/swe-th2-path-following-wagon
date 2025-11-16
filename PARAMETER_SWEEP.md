# Parameter Sweep Testing Guide

This guide explains how to use the parameter sweep testing tool to find optimal control parameters for the wagon control system.

## Overview

The parameter sweep tool systematically tests different parameter configurations to find settings that provide:
1. **Good average performance** (low mean error)
2. **High consistency** (low standard deviation)
3. **No catastrophic failures** (all runs < 50m error)

## Quick Start

### 1. Test the Script (Recommended First Step)

Before running the full sweep, test with minimal runs:

```bash
./test_sweep.sh
```

This runs 2 trials per configuration to verify everything works.

### 2. Run Full Parameter Sweep

Run the complete sweep with 10 trials per configuration:

```bash
python parameter_sweep.py --num-runs 10
```

**Expected Duration:** This will test ~32 configurations × 10 runs = ~320 total test runs. At ~25 seconds per run, expect **2-3 hours** for completion.

### 3. Custom Number of Runs

Adjust the number of trials per configuration:

```bash
# Quick sweep (less reliable)
python parameter_sweep.py --num-runs 5

# Thorough sweep (more reliable, takes longer)
python parameter_sweep.py --num-runs 15
```

### 4. Verbose Output

See detailed logs for debugging:

```bash
python parameter_sweep.py --num-runs 10 --verbose
```

## What Gets Tested

The script tests these parameters independently (one-at-a-time):

### 1. Localizer Velocity Correction Gain
- **Current:** 0.45
- **Test Range:** [0.35, 0.40, 0.45, 0.50, 0.55, 0.60, 0.65]
- **Impact:** Controls how aggressively GPS corrects velocity drift

### 2. Follower Base Lookahead
- **Current:** 0.8m
- **Test Range:** [0.6, 0.7, 0.8, 0.9, 1.0]
- **Impact:** Distance ahead on path to target (when not adaptive)

### 3. Follower Lookahead Time
- **Current:** 0.7s
- **Test Range:** [0.6, 0.65, 0.7, 0.75, 0.8, 0.85, 0.9]
- **Impact:** Velocity-dependent lookahead multiplier

### 4. Motor KI_V (Linear Velocity Integral Gain)
- **Current:** 0.08
- **Test Range:** [0.05, 0.06, 0.08, 0.10, 0.12, 0.15]
- **Impact:** How fast integral action eliminates steady-state velocity error

### 5. Motor KI_OMEGA (Angular Velocity Integral Gain)
- **Current:** 0.06
- **Test Range:** [0.04, 0.06, 0.08, 0.10, 0.12]
- **Impact:** How fast integral action eliminates steady-state heading error

### Reference Configurations
- **Baseline (ce8b3ef):** 7-15m variance baseline configuration
- **Current Config:** Your current configuration for comparison

## Output Files

Results are saved in the `results/` directory with timestamps:

### 1. CSV File: `parameter_sweep_YYYYMMDD_HHMMSS.csv`

Detailed data for all configurations with columns:
- `config_name`: Configuration description
- `mean`, `std_dev`, `min`, `max`: Statistical measures
- `num_failures`: Number of failed runs
- `is_valid`: Whether configuration meets criteria (no runs > 50m)
- `consistency_score`: Ranking metric (mean + 2×std_dev, lower is better)
- `all_scores`: Individual scores from each run

### 2. Summary File: `parameter_sweep_YYYYMMDD_HHMMSS_summary.txt`

Human-readable report containing:
- **Top 10 Configurations:** Ranked by consistency score
- **All Valid Configurations:** Sorted by mean error
- **Invalid Configurations:** Failed configs with outliers or errors

## Understanding Results

### Consistency Score

The primary ranking metric is: **Consistency Score = Mean + 2×StdDev**

This balances:
- **Mean error:** Lower is better (closer to path)
- **Standard deviation:** Lower is better (more consistent)
- **Weighting:** 2× weight on std_dev prioritizes consistency

### Valid vs Invalid Configurations

A configuration is marked **INVALID** if:
1. Any single run has error > 50m (catastrophic failure)
2. Any runs failed to complete

Valid configurations are candidates for optimal settings.

### Reading the Summary

Example output:
```
1. VelCorrectionGain=0.5      | Mean:   8.45m | Std:  1.23m | Range: [ 6.89, 10.12] | Failures:  0/10 | VALID
   Consistency Score: 10.91
   Parameters: {'LOCALIZER_VELOCITY_CORRECTION_GAIN': 0.5}
```

- **Mean:** Average error across 10 runs
- **Std:** Standard deviation (consistency measure)
- **Range:** [best run, worst run]
- **Consistency Score:** Lower is better

## Safety Features

### Automatic Backup & Restore
- Original `config.py` is backed up before testing
- Automatically restored after completion
- Restored even if interrupted (Ctrl+C)

### Outlier Detection
- Configurations with any run > 50m are flagged as invalid
- Prevents recommending unstable configurations

### Timeout Protection
- Each test run has 60-second timeout
- Failed runs don't block the sweep

## Tips for Optimal Results

### 1. Run During Stable Conditions
- Ensure consistent network connection
- Avoid running during server maintenance

### 2. Use Sufficient Runs
- Minimum 10 runs recommended for statistical significance
- 15-20 runs for more reliable results

### 3. Check Individual Scores
- Look at the `all_scores` column in CSV
- Identify if failures are random or systematic

### 4. Compare to Baseline
- The sweep tests both baseline and current config
- Use these as reference points

## Interpreting Top Results

After the sweep completes:

1. **Check the top 5 configurations** in the summary
2. **Look for patterns:**
   - Are top configs clustered around certain values?
   - Does current config rank well?
   - Is baseline still competitive?

3. **Verify consistency:**
   - Top config should have low std_dev (< 2m ideally)
   - Check that min/max range is reasonable

4. **Consider practical constraints:**
   - Very low lookahead may be unstable in practice
   - Very high integral gains may cause oscillations

## Applying Best Configuration

To apply the best configuration found:

1. Open `wagon_control/config.py`
2. Update the parameter(s) from the top result
3. Test manually with `./run.sh` to verify
4. Run multiple times to confirm consistency

## Troubleshooting

### Script fails immediately
- Check that `wagon_control/config.py` exists
- Verify virtual environment is activated
- Ensure dependencies are installed: `pip install -r requirements.txt`

### All configurations marked invalid
- Server may be down or unreachable
- Check network connection
- Try running `./run.sh` manually first

### Inconsistent results
- Increase number of runs (--num-runs 15 or higher)
- Check for external factors (network stability)
- Verify server is not under heavy load

## Advanced Usage

### Testing Custom Parameter Combinations

Edit `parameter_sweep.py` to add custom configurations:

```python
# In run_parameter_sweep() method, add:
self.results.append(
    self.test_configuration(
        "Custom Config",
        {
            "LOCALIZER_VELOCITY_CORRECTION_GAIN": 0.48,
            "FOLLOWER_BASE_LOOKAHEAD": 0.75,
            "MOTOR_KI_V": 0.09,
        },
    )
)
```

### Adjusting Test Ranges

Modify the ranges in `run_parameter_sweep()`:

```python
# Example: Test finer granularity around a good value
for gain in [0.48, 0.49, 0.50, 0.51, 0.52]:
    self.results.append(...)
```

### Changing Outlier Threshold

Currently set to 50m. To adjust, modify in the `test_configuration()` method:

```python
# Change from:
is_valid = max_score <= 50.0 and failures == 0

# To (example):
is_valid = max_score <= 30.0 and failures == 0
```

## Example Workflow

Complete workflow for finding optimal parameters:

```bash
# 1. Test the script works
./test_sweep.sh

# 2. Run full sweep (this takes 2-3 hours)
python parameter_sweep.py --num-runs 10

# 3. Review results
cat results/parameter_sweep_*_summary.txt | less

# 4. Apply best configuration manually to config.py

# 5. Verify with multiple test runs
./run.sh 5

# 6. Commit if satisfied
git add wagon_control/config.py
git commit -m "Optimize parameters based on sweep testing"
```

## Notes

- The script tests parameters **independently** (one at a time)
- For testing parameter **combinations**, you'd need to modify the script
- Results are deterministic if the simulator is deterministic
- Network latency and server load may affect consistency

## Support

If you encounter issues:
1. Check the verbose logs: `python parameter_sweep.py --verbose`
2. Verify manual runs work: `./run.sh`
3. Review the backup config: `wagon_control/config.py.backup`
