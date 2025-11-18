# Approach

## System Overview

This system implements a **four-layer control architecture** for autonomous path following of a differential-drive wagon. The control pipeline processes noisy sensor data (GPS at 1 Hz, IMU at 20 Hz) and generates wheel velocity commands to follow a Lemniscate of Gerono reference path over 20 seconds.

![Control System](images/control-system.png)

*Figure: Four-layer control architecture*

## Architecture & Components

The control system consists of four hierarchical layers:

### 1. **State Estimation (Localizer)** - `localizer.py`
- **Purpose**: Fuse noisy GPS and IMU data to estimate wagon state (position, heading, velocity)
- **Algorithm**: Extended Kalman Filter (EKF) with curvature-adaptive sensor fusion
  - **State vector** (5 elements): [px, py, θ, v, b_g] - position, heading, velocity, gyro bias
  - **GPS updates** (1 Hz): Position measurements with Mahalanobis outlier rejection
  - **IMU prediction** (20 Hz): Continuous state propagation via gyro/accelerometer integration
  - **Covariance propagation**: Principled uncertainty quantification through Jacobian linearization
  - **Curvature adaptation**: Dynamically adjusts GPS/IMU trust based on path curvature
- **Key Parameters**:
  - EKF process noise: Q (5×5) scaled by configurable factors
  - GPS measurement noise: R (2×2) = diag([0.25, 0.25]) m²
  - Mahalanobis threshold: 9.0 (base, relaxed during high-curvature turns)
  - Curvature heading scale: 7.0 (optimized - increases heading uncertainty 8× at κ=1.0 rad/m)
  - Curvature threshold scale: 1.5 (optimized - relaxes GPS rejection 2.5× at κ=1.0 rad/m)
  - Curvature bias scale: 2.0 (increases gyro bias uncertainty during turns)

### 2. **Path Following (Pure Pursuit)** - `follower.py`
- **Purpose**: Generate desired velocity commands to track the reference trajectory
- **Algorithm**: Time-based Pure Pursuit with adaptive lookahead
  - Tracks reference point based on elapsed time (ensures 20s completion)
  - Lookahead distance adapts to velocity: `L = 0.9 * |v| + 0.3` (clamped to [0.5m, 2.0m])
  - Computes curvature to lookahead point: `κ = 2*sin(α)/L`
- **Key Parameters**:
  - Base lookahead: 0.8m (good preview without cutting corners)
  - Lookahead time: 0.9s (optimized via parameter sweep for smoothest tracking)
  - Min/max lookahead: [0.5m, 2.0m] (stability bounds)

### 3. **Motor Controller (PI Feedback)** - `motor_controller.py`
- **Purpose**: Correct velocity tracking errors from model mismatch and disturbances
- **Algorithm**: PI feedback on linear and angular velocities
  - Proportional term: Immediate error correction
  - Integral term: Eliminates steady-state errors
  - Anti-windup: Clamps integral at ±0.5 to prevent overshoot
- **Key Parameters**:
  - P gains: Kv = Kω = 0.5 (moderate correction, prioritizes stability)
  - I gains: Kiv = 0.05, Kiω = 0.04 (optimized via parameter sweep for best consistency)

### 4. **Inverse Kinematics** - `model.py`
- **Purpose**: Convert (v, ω) commands to individual wheel velocities
- **Algorithm**: Differential drive kinematics with velocity clamping
  - `v_left = v - (L/2)*ω`, `v_right = v + (L/2)*ω`
  - Wheelbase L = 0.5m
  - Clamps to hardware limits: velocity ∈ [-2.0, 2.0] m/s

## Design Trade-Offs

### Extended Kalman Filter (EKF) for Sensor Fusion
**Choice**: Extended Kalman Filter with online gyro bias estimation
**Rationale**:
- **Pro:** Optimal sensor fusion with principled uncertainty quantification
- **Pro:** Online bias estimation eliminates need for manual calibration
- **Pro:** Covariance propagation enables adaptive fusion (curvature-based trust)
- **Pro:** Mahalanobis distance provides statistically-grounded outlier rejection
- **Con:** More complex than complementary filter (5×5 matrices, Jacobian linearization)
- **Con:** Requires tuning of Q and R noise covariance matrices

**Impact**: Achieves 9.26m ± 4.74m with curvature-adaptive fusion (25.8% better than baseline). The principled uncertainty framework enables dynamic GPS/IMU trust balancing based on path geometry.

### Pure Pursuit vs. Model Predictive Control (MPC)
**Choice**: Pure Pursuit with PI feedback
**Rationale**:
- **Pro:** Geometric intuition (follow carrot on path)
- **Pro:** Fast, no optimization required
- **Pro:** Well-suited for smooth, pre-defined paths
- **Con:** No explicit constraint handling (relies on lookahead tuning)
- **Con:** Less robust to obstacles

**Impact**: Successfully tracks Lemniscate with adaptive lookahead. MPC would enable explicit velocity constraints but requires real-time optimization (overkill for this task).

### PI vs. PID Control
**Choice**: PI feedback (no derivative term)
**Rationale**:
- **Pro:** P-term corrects errors, I-term eliminates steady-state drift
- **Pro:** Derivative amplifies sensor noise (especially with 20 Hz IMU)
- **Pro:** Low-pass velocity filter (α=0.3) provides implicit smoothing

**Impact**: Stable tracking without noise amplification. D-term would require additional filtering, adding complexity without clear benefit.

### Time-Based vs. Position-Based Tracking
**Choice**: Time-based reference tracking
**Rationale**:
- **Pro:** Guarantees path completion in 20 seconds (assignment requirement)
- **Pro:** Handles initial positioning errors gracefully
- **Con:** May cut corners if falling behind schedule

**Impact**: Robust to initialization errors, ensures timely completion. Position-based would track path shape more precisely but risk timeout.

## Parameter Tuning Methodology

### Initial Tuning (Baseline System)

1. **Gyro/Accel Bias**: Empirical measurement from stationary wagon data
   - Gyro bias: 0.015 rad/s (from -16.68° drift over 20s)
   - Accel X bias: 0.096 m/s² (from stationary measurements)

2. **Control Parameters**: Conservative manual tuning
   - Started with low gains, incrementally increased
   - Backed off from oscillation threshold
   - **Baseline result**: 23.14m ± 21.04m (high variance, unstable)

### Systematic Optimization (Parameter Sweep)

To optimize beyond manual tuning, a **systematic parameter sweep** was performed:

**Methodology:**
- **32 configurations tested** across 5 key parameters
- **10 runs per configuration** (320 total test runs)
- **Statistical ranking** by consistency score (mean + 2×std_dev)
- **Outlier rejection**: Configurations with any run >50m marked invalid

**Parameters tested:**
- Velocity correction gain: 0.35-0.65 (7 values)
- Base lookahead: 0.6-1.0m (5 values)
- Lookahead time: 0.6-0.9s (7 values)
- Linear velocity integral gain (KI_V): 0.05-0.15 (6 values)
- Angular velocity integral gain (KI_OMEGA): 0.04-0.12 (5 values)

**Results:**
- **26 valid** configurations (no outliers >50m)
- **6 invalid** configurations (including original baseline!)
- **Best configuration** identified by tightest error distribution

**Optimized Parameters:**
- `FOLLOWER_LOOKAHEAD_TIME = 0.9` (was 0.7s)
- `MOTOR_KI_V = 0.05` (was 0.08) - BEST consistency, zero poor runs
- `MOTOR_KI_OMEGA = 0.04` (was 0.06) - 70% good runs, minimal poor runs
- `LOCALIZER_GPS_OUTLIER_THRESHOLD = 2.5` (was 5.0m) - eliminates catastrophic failures
- `LOCALIZER_VELOCITY_CORRECTION_GAIN = 0.45` (kept - lower values failed)

**Performance Improvement:**
- **Before optimization**: 14.55m ± 10.89m (GPS threshold: 5.0m, 20 runs validated)
- **After optimization**: 12.45m ± 4.96m (GPS threshold: 2.5m, 20 runs validated)
- **Improvement**:
  - 14.4% better mean error
  - 54.5% better consistency (variance reduction)
  - 55% reduction in max error (53.35m → 24.23m)
  - Eliminated catastrophic failures: 1/20 (5%) → 0/20 (0%)
  - Increased excellent runs (<10m): 30% → 50%
  - Configuration now stable and valid

See `PARAMETER_SWEEP.md` for complete methodology, visualization tools, and detailed results.

### Sequential Run Analysis

The run script supports sequential experiments for statistical validation:

```bash
./run.sh 10    # Run 10 consecutive experiments
```

This feature enables:
- **Performance Statistics**: Collect multiple datasets to compute mean/variance of tracking error
- **Parameter Validation**: Test control parameter robustness across different noise realizations
- **Automated Testing**: Streamlined data collection without manual plot closing
- Each run saves to a separate timestamped directory for post-analysis

### Curvature-Adaptive Sensor Fusion (Advanced Optimization)

**Motivation**: The Lemniscate of Gerono path contains regions of high curvature (tight turns) and low curvature (straights). During tight turns, IMU gyro integration errors accumulate faster, while GPS provides more reliable absolute position. The key insight: **trust GPS more for absolute position over long distances, but trust IMU more for short-term motion tracking on straight sections**.

**Implementation**:

1. **Path Curvature Computation** (`path.py:reference_curvature()`)
   - Computes curvature κ from parametric path derivatives
   - Formula: κ = |x'y'' - y'x''| / (x'² + y'²)^(3/2)
   - Lemniscate curvature ranges: 0.0 (straights) to ~1.5 rad/m (tight turns)

2. **Adaptive Process Noise Scaling** (`localizer.py:set_path_curvature()`)
   - **Heading uncertainty**: Q[2,2] = Q_base[2,2] × (1 + 7.0 × |κ|)
     - At κ=1.0 rad/m: heading noise increases 8×
     - Tells EKF to trust IMU heading less during turns
   - **Gyro bias uncertainty**: Q[4,4] = Q_base[4,4] × (1 + 2.0 × |κ|)
     - Bias estimation becomes harder during fast angular rates

3. **Adaptive GPS Outlier Threshold** (`localizer.py:set_path_curvature()`)
   - Threshold = 9.0 × (1 + 1.5 × |κ|)
   - At κ=1.0 rad/m: threshold relaxed 2.5×
   - Prevents valid GPS from being rejected when IMU drifts in turns

**Parameter Optimization via Systematic Sweep**:

Created `curvature_sweep.py` to test 20 configurations (200 total runs):
- **Heading scale**: [3.0, 4.0, 5.0, 6.0, 7.0]
- **Threshold scale**: [0.5, 1.0, 1.5, 2.0]
- **Bias scale**: 2.0 (fixed)
- **Runs per config**: 10 (for statistical validity)

**Key Findings**:
- **Low heading scale (3.0-5.0)**: 10-30% catastrophic failures (>30m) due to IMU drift
- **Optimal configuration**: H7.0_T1.5_B2.0
  - Heading scale 7.0: Strong distrust of IMU during turns
  - Threshold scale 1.5: Moderate GPS acceptance relaxation
  - **Zero catastrophic failures** in all tested configurations with heading≥6.0

**Performance Results**:
- **Before curvature adaptation** (EKF baseline): 12.48m ± 9.62m (1 catastrophic failure)
- **After curvature adaptation** (H7.0_T1.5): 9.26m ± 4.74m (0 catastrophic failures)
- **Improvement**: 25.8% better mean, 50.7% better consistency
- **Validation**: 20 runs, 70% scored <10m

**Integration** (`client.py:376-384`):
```python
# Compute path curvature for adaptive sensor fusion
elapsed_time_for_curvature = current_time - self.follower.start_time
path_curvature = path.reference_curvature(elapsed_time_for_curvature)
self.localizer.set_path_curvature(path_curvature)
```

Called before each GPS update to dynamically adjust EKF trust parameters based on current path geometry.

## Known Limitations & Simplifications

### Simplifying Assumptions

1. **Single-Axis Gyro Bias Estimation**
   - **Assumption**: Only z-axis gyro bias estimated (yaw rate), x/y accelerometer biases assumed negligible
   - **Reality**: All IMU axes have biases that vary with temperature and time
   - **Impact**: X-axis accelerometer drift can affect velocity estimates
   - **Mitigation**: EKF estimates gyro bias online; accelerometer biases could be added to state vector
   - **Current approach**: Works well for short 20s missions; longer missions would benefit from full 6-axis bias estimation

2. **Constant Process/Measurement Noise**
   - **Assumption**: Q and R matrices constant (except curvature-adaptive scaling)
   - **Reality**: Sensor noise varies with speed, temperature, and environmental conditions
   - **Impact**: EKF may be over/under-confident in certain scenarios
   - **Mitigation**: Curvature-adaptive Q scaling addresses primary source of variation (path geometry)

3. **GPS Rate Limitations**
   - **Assumption**: 1 Hz GPS sufficient for path following at 2 m/s
   - **Reality**: Higher GPS rates (5-10 Hz) would reduce drift between updates
   - **Trade-off**: Accepted for this application; critical for faster robots

4. **No Wheel Slip Modeling**
   - **Assumption**: Commanded wheel velocities perfectly executed
   - **Reality**: Slippage occurs on smooth surfaces, during acceleration, or on inclines
   - **Impact**: Unmodeled in simulation; would cause errors in real hardware

## Future Improvements

### For Simulation/Algorithm Improvements

1. **✅ IMPLEMENTED: Extended Kalman Filter (EKF)** with curvature-adaptive sensor fusion
   - ✅ 5-state EKF with online gyro bias estimation
   - ✅ Mahalanobis distance GPS outlier rejection
   - ✅ Curvature-based Q matrix scaling for path-dependent fusion
   - **Further improvements**: Extend to 8-state for full accelerometer bias estimation

2. **Curvature-Based Control Gains**: Vary PI gains based on path curvature
   - Schedule gains based on reference trajectory curvature
   - Higher gains for tight turns, lower for straight sections
   - Reduces overshoot while maintaining responsiveness
   - **Note**: Curvature-adaptive *fusion* already implemented; control gains could be next

3. **Predictive Lookahead**: Use path curvature to adjust lookahead dynamically
   - Longer lookahead on straights for smoother tracking
   - Shorter lookahead on curves for tighter following
   - Could reduce corner-cutting behavior
   - **Current**: Fixed adaptive lookahead based on velocity only

4. **Velocity Profiling**: Pre-compute optimal velocity profile for path
   - Slow down before sharp turns (respecting acceleration limits)
   - Maximize velocity on straight sections
   - Ensures smoother, more natural motion

5. **Multi-Rate Sensor Fusion**: Handle sensors at different rates more explicitly
   - Asynchronous measurement updates
   - Proper uncertainty propagation between GPS updates
   - Better handling of delayed or dropped measurements

### For Real Hardware Deployment

6. **Wheel Encoders Integration**
   - **Critical for real robots**: Provides direct velocity measurement
   - **Benefits**:
     - More accurate than IMU integration between GPS updates
     - Immune to accelerometer drift
     - Enables detection of wheel slip
   - **Implementation**: Add encoder velocities to complementary filter
   - **Fusion approach**: Weight encoders heavily, use IMU for heading

7. **Improved Sensor Calibration**
   - **Multi-Point Calibration**: Measure biases at multiple orientations
   - **Temperature Compensation**: Characterize bias vs temperature relationship
   - **Online Estimation**: Use EKF to estimate time-varying biases
   - **Allan Variance Analysis**: Characterize noise characteristics for proper filter tuning
   - **Magnetometer Fusion**: Add magnetometer for absolute heading (reduce gyro drift)

8. **Wheel Slip Detection & Compensation**
   - **Detection**: Compare encoder velocity to IMU-predicted velocity
   - **Compensation**: Reduce trust in encoders when slip detected
   - **Control**: Limit acceleration during slip conditions
   - **Critical for**: Outdoor robots, slippery surfaces, high-acceleration maneuvers

9. **Environmental Robustness**
   - **GPS Multipath Mitigation**: Use carrier-phase measurements, multiple constellations (GPS+GLONASS+Galileo)
   - **IMU Grade**: Consider tactical-grade IMU for longer missions (lower drift)
   - **Differential GPS**: RTK-GPS for cm-level accuracy (eliminates GPS noise issues)
   - **Vibration Isolation**: Mount IMU on damped platform to reduce noise

10. **Real-Time Performance Optimization**
    - **Embedded Implementation**: Port to embedded C/C++ for real-time guarantees
    - **RTOS Integration**: Use real-time OS for deterministic control loop timing
    - **Computational Budget**: Profile and optimize for resource-constrained hardware
    - **Watchdog Timers**: Detect and recover from control loop failures

### Transition from Simulation to Hardware

**Key Considerations:**

1. **Hardware-in-the-Loop (HIL) Testing**: Test control algorithms with real sensors before full deployment
2. **Graduated Testing**: Start with low speeds, simple paths, then increase complexity
3. **Safety Mechanisms**: E-stop, velocity limits, workspace boundaries
4. **Sensor Validation**: Cross-check redundant sensors (GPS vs encoders vs IMU)
5. **Failure Modes**: Graceful degradation when sensors fail or provide bad data

## Generalization to Other Paths

**Does this approach generalize?**

**Yes, with minor modifications:**

**Works for**: Any smooth, differentiable path with known time parameterization
- Smooth curves (circles, ellipses, splines)
- Paths with moderate curvature changes
- Trajectories with reasonable velocity profiles (< 2 m/s)

**Requires tuning for**:
- Sharp corners (reduce lookahead distance, add velocity constraints)
- High-speed paths (increase lookahead time for better preview)
- Stop-and-go trajectories (add path velocity profiling)

**Key requirement**: Path must be representable as `(x(t), y(t), θ(t))` with time parameter `t`. The `path.py` module can be easily modified to support different path primitives (lines, arcs, Bézier curves, etc.).

## Performance Summary

### Final Optimized System (EKF + Curvature-Adaptive Fusion)

- **L2 Error**: 9.26m ± 4.74m (20 runs validated, H7.0_T1.5_B2.0 configuration)
  - Range: 3.89m - 19.75m
  - **25.8% improvement** in mean error vs EKF baseline (12.48m)
  - **50.7% improvement** in consistency (std dev: 4.74m vs 9.62m)
  - **Zero catastrophic failures** (>30m) - eliminated via curvature-adaptive GPS acceptance
  - **70% excellent runs** (<10m) vs 45% with baseline EKF
  - **Interpretation**: L2 error represents time-averaged cumulative tracking error
    - Average error per sample: 9.26m ÷ 20s = **463 mm/sample**
    - Average instantaneous deviation: **~400-500mm from reference path**
    - Excellent tracking performance for noisy 1 Hz GPS on high-curvature figure-eight path

### Evolution of Performance

1. **Initial Baseline** (Complementary Filter): 23.14m ± 21.04m
   - High variance, unstable
   - Frequent catastrophic failures

2. **After Parameter Sweep** (Optimized Complementary): 12.45m ± 4.96m
   - 46.2% mean improvement
   - 76.4% variance reduction
   - GPS threshold: 2.5m (down from 5.0m)

3. **EKF Implementation** (No Curvature Adaptation): 12.48m ± 9.62m
   - Online bias estimation
   - Principled uncertainty quantification
   - Still occasional catastrophic failures in tight turns

4. **Final System** (EKF + Curvature Adaptation): 9.26m ± 4.74m
   - 25.8% improvement over EKF baseline
   - Zero catastrophic failures
   - Robust to high-curvature sections

### System Characteristics

- **Completion Time**: Consistently 20.0s ± 0.1s
- **Control Frequency**: 20 Hz (50ms update cycle)
- **Sensor Fusion**: Extended Kalman Filter with 5-state vector [px, py, θ, v, b_g]
- **Adaptive Fusion**: GPS/IMU trust dynamically adjusted based on path curvature
- **Robustness**: Curvature-scaled Mahalanobis outlier rejection (9.0 base, 22.5 peak at κ=1.5 rad/m)

### Key Innovation: Curvature-Adaptive Trust

The breakthrough improvement came from recognizing that sensor reliability is **path-dependent**:
- **Straight sections** (κ≈0): IMU gyro integration is accurate → trust IMU for heading
- **Tight turns** (κ>1.0 rad/m): IMU drifts rapidly → trust GPS for position, relax outlier threshold

This adaptive approach eliminated all catastrophic failures by preventing GPS rejection during the critical high-curvature phases where IMU is least reliable.
