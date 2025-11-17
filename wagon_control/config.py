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
- 0.3 (30% new, 70% old) provides good noise rejection
- Smooths accelerometer integration errors
- Maintains responsiveness for feedback control
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
    0.1,     # v process noise (m²/s²)
    1e-6     # b_g process noise (rad²/s²)
]
"""Process noise covariance diagonal for EKF.

Tuning rationale:
- Position noise (px, py): Small values (0.01) since position accumulates from velocity
- Heading noise (theta): Very small (1e-4) for gyro integration with bias estimation
- Velocity noise (v): Moderate (0.1) to account for accelerometer noise and model mismatch
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

EKF_R_SCALE = 1.0
"""Global scaling factor for GPS measurement noise covariance R.

Larger values (>1) = trust GPS less, trust model more
Smaller values (<1) = trust GPS more, trust model less

Tuning rationale:
- 1.0 is baseline using 3m GPS sigma
- Use parameter sweep to optimize this factor
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

FOLLOWER_LOOKAHEAD_TIME = 0.9
"""Time-based lookahead gain for adaptive mode (seconds).

Lookahead distance = FOLLOWER_LOOKAHEAD_TIME * |v| + FOLLOWER_LOOKAHEAD_OFFSET

Tuning rationale:
- Optimized to 0.9s based on parameter sweep (32 configs × 10 runs)
- Sweep result: Mean 11.35m, Std 5.29m (ranked 5th overall)
- Longer lookahead provides smoother path following with better consistency
- Provides velocity-proportional lookahead: faster → look further ahead
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


# ============================================================================
# Motor Controller Parameters (PI Feedback)
# ============================================================================

# Proportional gains
MOTOR_KP_V = 1.2
"""Proportional gain for linear velocity feedback (range: [0, 2]).

Higher = more aggressive velocity correction.

Tuning rationale:
- Iterative tuning: 0.5 → 0.9 → 1.1 (slightly slow) → 1.3 (too aggressive)
- 1.3 caused degraded performance (overshoot/oscillations)
- 1.2 is the sweet spot between 1.1 (good but slow) and 1.3 (too much)
"""

MOTOR_KP_OMEGA = 0.5
"""Proportional gain for angular velocity feedback (range: [0, 2]).

Higher = more aggressive heading correction.

Tuning rationale:
- 0.5 matches linear velocity gain for balanced control
- Prevents coupling between linear and angular control loops
"""

# Integral gains
MOTOR_KI_V = 0.18
"""Integral gain for linear velocity (range: [0, 0.5]).

Eliminates steady-state velocity tracking error.

Tuning rationale:
- Iterative tuning: 0.05 → 0.12 → 0.16 (good) → 0.20 (too high, caused windup)
- 0.18 balances fast error elimination without integral windup
- Works with KP_V=1.2 for optimal velocity tracking
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

# Anti-windup limit
MOTOR_INTEGRAL_LIMIT = 0.5
"""Anti-windup limit for integral terms (meters/second or radians/second).

Clamps integral accumulation to prevent large overshoots.

Tuning rationale:
- 0.5 limit prevents excessive integral buildup
- Allows reasonable correction without destabilizing control
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
