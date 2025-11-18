"""Configuration parameters for wagon control system.

This module centralizes all configuration parameters including:
- Physical robot parameters
- Control system gains
- Sensor fusion parameters
- Visualization settings
- WebSocket connection parameters

All parameters are documented with their purpose, valid ranges, and tuning rationale.
"""

import numpy as np

# ============================================================================
# Physical Robot Parameters
# ============================================================================

WHEELBASE = 0.5
"""Distance between left and right wheels (meters).
Fixed by robot hardware design."""

# Velocity constraints (actuator limits)
V_MIN = -2.0
"""Minimum wheel velocity (m/s). Hardware limit."""

V_MAX = 2.0
"""Maximum wheel velocity (m/s). Hardware limit."""

# Acceleration constraints (actuator limits)
A_MIN = -1.0
"""Minimum wheel acceleration (m/s²). Hardware limit."""

A_MAX = 1.0
"""Maximum wheel acceleration (m/s²). Hardware limit."""


# ============================================================================
# Localization Parameters (Complementary Filter)
# ============================================================================

# Velocity correction gain (GPS fusion)
LOCALIZER_VELOCITY_CORRECTION_GAIN = 0.45
"""Gain for velocity correction when GPS arrives (range: [0, 1]).

Higher values = more aggressive correction of drift.
Lower values = smoother transitions but more accumulated error.

Tuning rationale:
- Reduced from initial 0.65 to 0.45 based on analysis showing GPS spikes
- 0.45 provides good balance: corrects drift without overcorrecting to noisy GPS
- Prevents velocity jumps that destabilize the controller
"""

# Velocity filtering
LOCALIZER_VELOCITY_FILTER_ALPHA = 0.3
"""Low-pass filter alpha for velocity smoothing (range: [0, 1]).

Exponential moving average: v_filtered = alpha * v_new + (1-alpha) * v_old

Higher alpha = faster response but more noise
Lower alpha = smoother but slower response

Tuning rationale:
- Reverted from 0.4 to 0.3 after Option A caused catastrophic instability
- Option A (alpha=0.4) introduced velocity noise that was amplified by KD_V
- 0.3 (30% new, 70% old) provides stable noise rejection
- Works with Phase 1 Revised baseline (9/10 good runs)
"""

# IMU bias compensation
LOCALIZER_GYRO_BIAS = 0.015
"""Gyroscope bias compensation (rad/s).

Origin: Estimated from stationary phase analysis
- Observed -16.68° drift over 20 seconds with zero input
- Drift rate = -16.68° / 20s = -0.835°/s = -0.0146 rad/s
- Rounded to 0.015 rad/s for positive compensation
"""

LOCALIZER_ACCEL_X_BIAS = 0.096
"""Accelerometer X-axis (forward) bias (m/s²).

Origin: Measured from stationary phase before movement
- Stationary wagon should read 0 m/s² in X direction
- Observed consistent positive offset of ~0.096 m/s²
- This bias causes velocity drift if not compensated
"""

# GPS outlier rejection
LOCALIZER_GPS_OUTLIER_THRESHOLD = 2.5
"""GPS outlier rejection threshold (meters).

Reject GPS updates that are more than this distance from predicted position.
Protects against multipath errors, atmospheric interference, and GPS glitches.

Tuning rationale:
- 2.5m threshold is aggressive but reasonable (1.25× max movement between updates)
- At 2m/s max velocity and 1Hz GPS, wagon moves ≤2m between GPS updates
- Rejects GPS outliers more aggressively to prevent catastrophic failures
- Tightened from 5m after observing rare 50m+ outlier failures
"""


# ============================================================================
# Extended Kalman Filter (EKF) Parameters
# ============================================================================

# EKF state vector: [px, py, theta, v, b_g]
#   px, py: position (m)
#   theta: heading (rad)
#   v: forward velocity (m/s)
#   b_g: gyro bias (rad/s)

# Process noise covariance Q (5×5)
EKF_Q_DIAG = [
    0.01,    # px process noise (m²)
    0.01,    # py process noise (m²)
    1e-4,    # theta process noise (rad²)
    0.15,    # v process noise (m²/s²) - PHASE 1 REVISED: Conservative increase
    1e-6     # b_g process noise (rad²/s²)
]
"""Process noise covariance diagonal for EKF.

Tuning rationale:
- Position noise (px, py): Small values (0.01) since position accumulates from velocity
- Heading noise (theta): Very small (1e-4) for gyro integration with bias estimation
- Velocity noise (v): PHASE 1 REVISED - Increased from 0.1 to 0.15 (+50%)
  * Conservative increase vs initial 0.25 which caused regression
  * Analysis showed velocity oscillations and covariance growth
  * Moderately higher Q[3,3] accounts for accelerometer noise
  * Balances smoother estimates with maintaining GPS/IMU fusion quality
- Gyro bias noise (b_g): Very small (1e-6) models slow random walk of bias over time
"""

EKF_Q = np.diag(EKF_Q_DIAG)

# GPS measurement noise covariance R (2×2)
EKF_R_GPS_SIGMA = 3.0  # GPS position uncertainty (meters, 1-sigma)
"""GPS measurement noise standard deviation (meters).

Based on typical consumer GPS horizontal accuracy:
- Open sky: 2-5m (95% confidence)
- Urban canyon: 5-10m (95% confidence)
- Conservative value: 3m (1-sigma) ≈ 6m (95% confidence)
"""

EKF_R_GPS = np.diag([EKF_R_GPS_SIGMA**2, EKF_R_GPS_SIGMA**2])

# Initial state covariance P0 (5×5)
EKF_P0_DIAG = [
    10.0,    # px initial uncertainty (m²)
    10.0,    # py initial uncertainty (m²)
    0.5,     # theta initial uncertainty (rad²)
    1.0,     # v initial uncertainty (m²/s²)
    0.1      # b_g initial uncertainty (rad²/s²)
]
"""Initial state covariance diagonal for EKF.

Rationale:
- Large position uncertainty (10m) since initial GPS may be noisy
- Moderate heading uncertainty (0.5 rad ≈ 40°) before gyro integration stabilizes
- Moderate velocity uncertainty (1 m/s) until acceleration integration stabilizes
- Small bias uncertainty (0.1) allows quick convergence during initial maneuvers
"""

EKF_P0 = np.diag(EKF_P0_DIAG)

# Mahalanobis distance threshold for GPS outlier rejection
EKF_MAHALANOBIS_THRESHOLD = 9.21
"""Chi-square threshold for GPS outlier rejection (2 DOF, 99% confidence).

Using innovation-based outlier detection: d² = innovation.T @ S_inv @ innovation
where S is the innovation covariance (H @ P @ H.T + R).

Threshold values (chi-square distribution, 2 degrees of freedom):
- 95% confidence: 5.99
- 99% confidence: 9.21
- 99.9% confidence: 13.82

Tuning rationale:
- 99% threshold rejects clear outliers while accepting legitimate GPS variations
- More principled than fixed distance threshold
- Adapts to EKF uncertainty estimate
"""

# Scaling factors for tuning (multiply base Q and R matrices)
EKF_Q_SCALE = 1.0
"""Global scaling factor for process noise covariance Q.

Larger values (>1) = trust model less, trust measurements more
Smaller values (<1) = trust model more, trust measurements less

Tuning rationale:
- 1.0 is baseline using manually chosen Q values
- Use parameter sweep to optimize this factor
"""

EKF_R_SCALE = 1.25
"""Global scaling factor for GPS measurement noise covariance R.

Larger values (>1) = trust GPS less, trust model more
Smaller values (<1) = trust GPS more, trust model less

Tuning rationale:
- PHASE 1 REVISED: Reverted to 1.25 (Stage 1 value)
- Stage 1 successfully eliminated catastrophic failures with R_SCALE=1.25
- Previous Phase 1 reduction to 1.15 increased variability (std 3.00m)
- Effective GPS sigma: 3.0m × 1.25 = 3.75m (conservative)
- Works with CURVATURE_THRESHOLD_SCALE=1.2 for balanced outlier rejection
- Balances with moderate Q[3,3]=0.15 velocity noise increase
"""

# Curvature-Adaptive Sensor Fusion Parameters
CURVATURE_HEADING_SCALE = 7.0
"""Heading process noise scaling factor based on path curvature.

Formula: heading_noise = base_noise * (1.0 + CURVATURE_HEADING_SCALE * |κ|)

Controls how much to distrust IMU heading integration during curved paths:
- Higher values (7-8): Trust GPS more during turns, prevents IMU drift
- Current (7.0): Optimized from parameter sweep (20 configs × 10 runs)
- Lower values (3-5): Trust IMU more during turns, but higher failure rate

Tuning rationale:
- At κ=1.0 rad/m (tight turn): heading noise increases 8× with current value
- Gyro integration errors accumulate faster during fast rotations
- Higher scaling prevents GPS rejection when IMU drifts in turns
- Parameter sweep showed 7.0 provides best consistency (8.82m ± 2.40m)
- Lower values (3.0-5.0) caused catastrophic failures in 10-30% of runs
"""

CURVATURE_BIAS_SCALE = 2.0
"""Gyro bias process noise scaling factor based on path curvature.

Formula: bias_noise = base_noise * (1.0 + CURVATURE_BIAS_SCALE * |κ|)

Controls gyro bias estimation uncertainty during curved paths:
- Higher values (3): More bias variation allowed in turns
- Current (2.0): Moderate bias adaptation
- Lower values (1): Assume stable bias even in turns

Tuning rationale:
- Bias estimation is harder during fast angular rates
- At κ=1.0 rad/m: bias uncertainty increases 3× with current value
- Prevents bias estimate from getting stuck during dynamic maneuvers
"""

CURVATURE_THRESHOLD_SCALE = 1.2
"""GPS outlier threshold scaling factor based on path curvature.

Formula: threshold = base_threshold * (1.0 + CURVATURE_THRESHOLD_SCALE * |κ|)

Controls GPS outlier rejection relaxation during curved paths:
- Higher values (2.0+): Too relaxed, accepts bad GPS outliers
- Current (1.2): Stage 1 - More conservative to prevent catastrophic failures
- Lower values (0.5-1.0): Too strict, rejects valid GPS in turns

Tuning rationale:
- Stage 1: Reduced from 1.5 to 1.2 to eliminate catastrophic failures
- More conservative GPS acceptance during turns prevents 10% failure rate
- At κ=1.0 rad/m: Mahalanobis threshold increases 2.2× (9.21 → 20.26) vs 2.5× before
- Tighter threshold rejects GPS outliers that could cause divergence
- Balances between rejecting outliers and accepting valid GPS in turns
- Works with EKF_R_SCALE=1.25 and CURVATURE_HEADING_SCALE=7.0
"""


# ============================================================================
# Path Following Parameters (Pure Pursuit)
# ============================================================================

# Lookahead distance parameters
FOLLOWER_BASE_LOOKAHEAD = 0.8
"""Base lookahead distance for pure pursuit (meters).

Distance ahead on path to target when adaptive lookahead is disabled.

Tuning rationale:
- 0.8m provides good preview without overshooting curves
- Too small (<0.5m) = oscillations and unstable tracking
- Too large (>1.5m) = cuts corners, poor path following
"""

FOLLOWER_LOOKAHEAD_TIME = 0.8
"""Time-based lookahead gain for adaptive mode (seconds).

Lookahead distance = FOLLOWER_LOOKAHEAD_TIME * |v| + FOLLOWER_LOOKAHEAD_OFFSET

Tuning rationale:
- Reverted from 0.85s to 0.8s after Option A caused catastrophic instability
- Combined with velocity filter and derivative changes, 0.85s contributed to divergence
- 0.8s provides balanced responsiveness from Stage 1 baseline
- At typical speeds (0.5-1.0 m/s), lookahead is 0.7-1.1m
- Works with Phase 1 Revised stable configuration
"""

FOLLOWER_LOOKAHEAD_OFFSET = 0.3
"""Minimum base lookahead offset (meters).

Ensures minimum lookahead even at zero velocity.

Tuning rationale:
- 0.3m minimum prevents excessive oscillation at low speeds
- Small enough to track tight curves accurately
"""

FOLLOWER_MIN_LOOKAHEAD = 0.5
"""Minimum adaptive lookahead distance (meters).

Safety bound for adaptive lookahead calculation.

Tuning rationale:
- 0.5m is minimum for stable pure pursuit control
- Below this, controller becomes too reactive and oscillates
"""

FOLLOWER_MAX_LOOKAHEAD = 2.0
"""Maximum adaptive lookahead distance (meters).

Safety bound for adaptive lookahead calculation.

Tuning rationale:
- 2.0m prevents looking too far ahead on curved paths
- Keeps control responsive to path changes
"""

FOLLOWER_K_TEMPORAL = 0.10
"""Temporal error feedback gain (dimensionless, range: [0, 0.5]).

Adds velocity adjustment to catch up when behind schedule or slow down when ahead.

Velocity adjustment = K_TEMPORAL * temporal_error
where temporal_error is in seconds (positive = behind, negative = ahead).

Tuning rationale:
- Reverted to 0.10 after Stage 2 regression (8.27m → 12.76m)
- Stage 2 showed aggressive control amplifies noisy EKF state estimates
- Conservative gain works well with Stage 1 results (mean 8.27m, std 1.54m)
- Will re-evaluate after EKF noise tuning produces cleaner estimates
"""

FOLLOWER_MAX_TEMPORAL_ADJUSTMENT = 0.4
"""Maximum velocity adjustment from temporal error feedback (m/s).

Safety limit to prevent excessive velocity corrections.

Tuning rationale:
- Reduced from 0.5 to 0.4 m/s for more conservative corrections
- Lower limit prevents temporal feedback from causing large velocity swings
- With K_TEMPORAL=0.10, typical adjustments are ~0.1-0.2 m/s
- Safety margin prevents rare large errors from destabilizing the system
- Prioritizes stability over aggressive temporal tracking
"""


# ============================================================================
# Motor Controller Parameters (PI Feedback)
# ============================================================================

# Proportional gains
MOTOR_KP_V = 1.0
"""Proportional gain for linear velocity feedback (range: [0, 2]).

Higher = more aggressive velocity correction.

Tuning rationale:
- Reverted to 1.0 after Stage 2 regression (8.27m → 12.76m)
- Higher gains amplified noisy velocity estimates from EKF
- Works well with Stage 1 EKF parameters (mean 8.27m, std 1.54m)
- Will re-tune after EKF produces smoother velocity estimates
- Balanced with KI_V=0.15, KD_V=0.05, KF_V=1.0
"""

MOTOR_KP_OMEGA = 0.5
"""Proportional gain for angular velocity feedback (range: [0, 2]).

Higher = more aggressive heading correction.

Tuning rationale:
- 0.5 matches linear velocity gain for balanced control
- Prevents coupling between linear and angular control loops
"""

# Integral gains
MOTOR_KI_V = 0.15
"""Integral gain for linear velocity (range: [0, 0.5]).

Eliminates steady-state velocity tracking error.

Tuning rationale:
- Reverted to 0.15 after Stage 2 regression (8.27m → 12.76m)
- Aggressive integral amplified velocity estimate noise from EKF
- Works well with Stage 1 parameters (mean 8.27m, std 1.54m)
- Will re-tune after EKF noise parameters improve state estimates
- Balanced with KP_V=1.0, KD_V=0.05, K_TEMPORAL=0.10
"""

MOTOR_KI_OMEGA = 0.04
"""Integral gain for angular velocity (range: [0, 0.5]).

Eliminates steady-state heading tracking error.

Tuning rationale:
- Optimized to 0.04 based on parameter sweep (32 configs × 10 runs)
- Sweep result: Mean 10.11m, Std 3.20m (2nd best consistency score: 16.52)
- Very conservative integral action provides excellent stability
- Prevents heading oscillations and integral windup during turns
"""

# Feedforward gains
MOTOR_KF_V = 1.0
"""Feedforward gain for linear velocity acceleration (dimensionless, range: [0, 1]).

Anticipates required accelerations from path to reduce tracking lag.

Feedforward term: v_ff = Kf * a_ref
where a_ref is the reference acceleration computed from velocity profile.

Tuning rationale:
- Initial 0.3 was too conservative - wagon achieved only 77% of commanded velocity
- Increased to 0.7 (70% feedforward authority) but still had 160ms lag
- Increased to 1.0 (100% feedforward authority) for maximum anticipation
- At max path acceleration (0.256 m/s²), contributes 0.256 m/s anticipation
- Works with PID (KP=1.0, KI=0.12, KD=0.1) for full predictive + reactive control
- Derivative term (KD=0.1) prevents overshoot from aggressive feedforward
"""

MOTOR_KF_OMEGA = 0.0
"""Feedforward gain for angular acceleration (dimensionless, range: [0, 1]).

Currently disabled (0.0) - focusing on linear velocity feedforward first.

Tuning rationale:
- Angular control is already performing well with PI alone
- Linear velocity tracking is the primary bottleneck
- Can enable later if angular tracking needs improvement
"""

# Derivative gains
MOTOR_KD_V = 0.05
"""Derivative gain for linear velocity (dimensionless, range: [0, 0.5]).

Provides damping and reduces transient response lag.

Derivative term: v_d = -Kd * d(error)/dt

Tuning rationale:
- Reverted from 0.07 to 0.05 after Option A caused catastrophic instability
- KD_V=0.07 amplified velocity filter noise (alpha=0.4), causing wild oscillations
- 6/10 runs in Option A showed velocity runaway and GPS rejection
- Conservative 0.05 provides stable damping without noise amplification
- Works with Phase 1 Revised baseline (9/10 good runs: 7.56m ± 2.48m)
"""

MOTOR_KD_OMEGA = 0.0
"""Derivative gain for angular velocity (dimensionless, range: [0, 0.5]).

Currently disabled (0.0) - caused instability when enabled.

Tuning rationale:
- Disabled after KD_OMEGA=0.08 caused angular instability (runs 11.02m, 27.80m)
- Derivative term amplified gyro noise → wild angular oscillations → heading loss
- PI control alone (KP=0.5, KI=0.04) provides sufficient angular tracking
- Angular control doesn't need derivative damping - it creates more problems
- System stable with linear derivative only (KD_V=0.05)
"""

# Anti-windup limit
MOTOR_INTEGRAL_LIMIT = 0.5
"""Anti-windup limit for integral terms (meters/second or radians/second).

Clamps integral accumulation to prevent large overshoots.

Tuning rationale:
- 0.5 limit prevents excessive integral buildup
- Allows reasonable correction without destabilizing control
"""

# Velocity compensation factors
MOTOR_VELOCITY_COMPENSATION = 1.08
"""Velocity compensation factor for actuator response lag (dimensionless, range: [1.0, 1.2]).

Compensates for systematic velocity tracking deficit.

Compensation: v_cmd_final = v_cmd_pid * compensation_factor

Tuning rationale:
- Reverted to 1.08 after Stage 2 regression (8.27m → 12.76m)
- Higher compensation with noisy estimates caused instability
- 8% boost works well with Stage 1 parameters (mean 8.27m, std 1.54m)
- Will re-tune after EKF provides cleaner velocity estimates
- Balanced with KI_V=0.15, KP_V=1.0, K_TEMPORAL=0.10
"""

MOTOR_OMEGA_COMPENSATION = 1.0
"""Angular velocity compensation factor (dimensionless, range: [1.0, 1.2]).

Currently disabled (1.0) - angular tracking is adequate.

Tuning rationale:
- Angular velocity tracking performs well without compensation
- Can increase if analysis shows omega tracking deficit
"""


# ============================================================================
# Visualization Colors (Monumental Branding)
# ============================================================================

# Brand colors (hex codes for matplotlib)
MONUMENTAL_ORANGE = "#f74823"
"""Primary brand color - used for main data, measurements, actual trajectory."""

MONUMENTAL_BLUE = "#2374f7"
"""Secondary brand color - used for reference, predictions, ideal paths."""

MONUMENTAL_CREAM = "#fffdee"
"""Light color for text and labels on dark backgrounds."""

MONUMENTAL_TAUPE = "#686a5f"
"""Neutral color for guides, grids, and secondary elements."""

MONUMENTAL_YELLOW_ORANGE = "#ffa726"
"""Accent color (Atrium style) for highlights and warnings."""

MONUMENTAL_DARK_BLUE = "#0d1b2a"
"""Dark background color for live plots and dark mode displays."""

# Terminal color codes (ANSI escape sequences)
TERM_ORANGE = "\033[38;2;247;72;35m"
"""Terminal color code for Monumental orange (RGB: 247, 72, 35)."""

TERM_BLUE = "\033[38;2;35;116;247m"
"""Terminal color code for complementary blue (RGB: 35, 116, 247)."""

TERM_RESET = "\033[0m"
"""Terminal color reset code."""


# ============================================================================
# WebSocket Configuration
# ============================================================================

WS_URI = "ws://91.99.103.188:8765"
"""WebSocket server URI for wagon simulation."""

WS_RETRY_DELAY_SECONDS = 1
"""Initial retry delay for failed WebSocket connections (seconds)."""

WS_MAX_RETRY_DELAY_SECONDS = 60
"""Maximum retry delay with exponential backoff (seconds)."""

WS_TIMEOUT_SECONDS = 5.0
"""Timeout for WebSocket message reception (seconds)."""


# ============================================================================
# Path Planning Configuration
# ============================================================================

PATH_DURATION = 20.0
"""Total duration for reference path trajectory (seconds).
Must match assignment requirement (20-second Lemniscate)."""

PATH_DT = 0.1
"""Time step for path discretization (seconds).
Results in 200 waypoints for 20-second trajectory."""
