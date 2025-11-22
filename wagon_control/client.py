#!/usr/bin/env python3
"""
WebSocket Client for Wagon Control and Sensor Data Collection

This module provides a WebSocket client that connects to a wagon control server,
sends velocity commands, and collects simulated IMU and GPS sensor data. After
sending velocity commands, it waits for the score message (~20s), then terminates
and saves collected data to CSV files.
"""

import argparse
import asyncio
import json
import logging
import math
import signal
import sys
import time
from typing import Any, Dict, List, Optional, Tuple, Union

import websockets

from wagon_control import path
from wagon_control.config import (
    FOLLOWER_BASE_LOOKAHEAD,
    FOLLOWER_K_TEMPORAL,
    FOLLOWER_LOOKAHEAD_TIME,
    FOLLOWER_MAX_TEMPORAL_ADJUSTMENT,
    # Control parameters
    LOCALIZER_VELOCITY_CORRECTION_GAIN,
    MOTOR_KD_OMEGA,
    MOTOR_KD_V,
    MOTOR_KF_OMEGA,
    MOTOR_KF_V,
    MOTOR_KI_OMEGA,
    MOTOR_KI_V,
    MOTOR_KP_OMEGA,
    MOTOR_KP_V,
    MOTOR_OMEGA_COMPENSATION,
    MOTOR_VELOCITY_COMPENSATION,
    PATH_DT,
    # Path configuration
    PATH_DURATION,
    TERM_BLUE,
    # Terminal colors
    TERM_RESET,
    WS_MAX_RETRY_DELAY_SECONDS,
    WS_RETRY_DELAY_SECONDS,
    WS_TIMEOUT_SECONDS,
    # WebSocket configuration
    WS_URI,
)
from wagon_control.component_modes import ComponentMode, parse_component_flags
from wagon_control.data_collector import DataCollector
from wagon_control.follower import PurePursuitFollower
from wagon_control.localizer import WagonLocalizer
from wagon_control.model import inverse_kinematics
from wagon_control.motor_controller import MotorController


class CustomFormatter(logging.Formatter):
    """Custom logging formatter that removes timestamps from INFO messages.

    This formatter provides clean console output by showing INFO messages without
    timestamps while preserving full context for WARNING, ERROR, and DEBUG messages.
    """

    def format(self, record: logging.LogRecord) -> str:
        """Format a log record based on its level.

        Args:
            record: The log record to format.

        Returns:
            Formatted log message string.
        """
        if record.levelno == logging.INFO:
            # INFO messages: just the message without timestamp
            return record.getMessage()
        else:
            # WARNING, ERROR, etc.: include timestamp and level
            return f"{self.formatTime(record, self.datefmt)} - {record.levelname} - {record.getMessage()}"


def setup_logging(verbose: bool = False) -> None:
    """Configure logging based on verbosity level.

    Args:
        verbose: If True, show all levels with timestamps. If False, show INFO
                 without timestamps and WARNING/ERROR with timestamps.
    """
    if verbose:
        # Verbose mode: show all levels with timestamps
        logging.basicConfig(
            level=logging.DEBUG,
            format="%(asctime)s - %(levelname)s - %(message)s",
            datefmt="%Y-%m-%d %H:%M:%S",
        )
    else:
        # Normal mode: INFO without timestamps, WARNING/ERROR with timestamps
        handler = logging.StreamHandler()
        formatter = CustomFormatter(datefmt="%Y-%m-%d %H:%M:%S")
        handler.setFormatter(formatter)
        logger = logging.getLogger()
        logger.setLevel(logging.INFO)
        logger.addHandler(handler)


class WagonController:
    """Wagon control system with WebSocket communication and data logging.

    This class manages the complete wagon control pipeline:
    - WebSocket connection to simulation server
    - Sensor data collection (IMU, GPS)
    - State estimation (localization)
    - Path following control (Pure Pursuit)
    - Motor feedback control (PID + acceleration feedforward)
    - Data logging to CSV files

    Attributes:
        uri: WebSocket URI to connect to.
        data_collector: Handles CSV file logging.
        localizer: State estimator (complementary filter).
        follower: Path follower (Pure Pursuit).
        motor_controller: Velocity feedback controller (PID + feedforward).
        should_stop: Flag indicating whether to stop control loop.
    """

    def __init__(self, uri: str, output_dir: str = ".", component_mode: Optional[ComponentMode] = None) -> None:
        """Initialize the wagon controller.

        Args:
            uri: WebSocket URI to connect to (must start with ws:// or wss://).
            output_dir: Base directory for output files (default: current directory).
            component_mode: ComponentMode configuration for component isolation testing.

        Raises:
            ValueError: If URI format is invalid.
        """
        # Validate URI format
        if not uri or not uri.startswith(("ws://", "wss://")):
            raise ValueError(f"Invalid WebSocket URI: {uri}. Must start with 'ws://' or 'wss://'")

        self.uri: str = uri
        self.should_stop: bool = False

        # Store component mode configuration
        if component_mode is None:
            component_mode = ComponentMode()  # Default: all components enabled
        self.component_mode = component_mode

        # Log active component configuration
        logging.info(f"{TERM_BLUE}Component Configuration: {component_mode}{TERM_RESET}")

        # Initialize data collector
        self.data_collector = DataCollector(output_dir=output_dir)

        # Initialize control system components (using config parameters + component modes)
        self.localizer = WagonLocalizer(
            velocity_correction_gain=LOCALIZER_VELOCITY_CORRECTION_GAIN,
            bypass_mode=not component_mode.use_ekf
        )
        self.follower = PurePursuitFollower(
            lookahead_distance=FOLLOWER_BASE_LOOKAHEAD,
            lookahead_time=FOLLOWER_LOOKAHEAD_TIME,
            k_temporal=FOLLOWER_K_TEMPORAL,
            max_temporal_adjustment=FOLLOWER_MAX_TEMPORAL_ADJUSTMENT,
            bypass_mode=not component_mode.use_pure_pursuit,
            disable_temporal=not component_mode.use_temporal_feedback,
        )
        self.motor_controller = MotorController(
            k_v=MOTOR_KP_V,
            k_omega=MOTOR_KP_OMEGA,
            k_i_v=MOTOR_KI_V,
            k_i_omega=MOTOR_KI_OMEGA,
            k_d_v=MOTOR_KD_V,
            k_d_omega=MOTOR_KD_OMEGA,
            k_f_v=MOTOR_KF_V,
            k_f_omega=MOTOR_KF_OMEGA,
            velocity_compensation=MOTOR_VELOCITY_COMPENSATION,
            omega_compensation=MOTOR_OMEGA_COMPENSATION,
            bypass_mode=not component_mode.use_pid,
            disable_feedforward=not component_mode.use_feedforward,
            disable_integral=not component_mode.use_integral,
            disable_derivative=not component_mode.use_derivative,
        )

        # Latest sensor readings (for IMU integration)
        self.current_ax: float = 0.0  # Body-frame x-acceleration
        self.current_ay: float = 0.0  # Body-frame y-acceleration
        self.current_theta_dot: float = 0.0  # Angular velocity
        self.last_update_time: Optional[float] = None  # For dt computation

        # Initialization tracking (wait for first fresh GPS before starting control)
        self.initialized: bool = False  # Set to True after first valid GPS
        self.last_gps_position: Optional[Tuple[float, float]] = None  # Track stale GPS readings

        # Pre-compute reference path
        path_data = path.path_trajectory(t_max=PATH_DURATION, dt=PATH_DT)
        self.path_t = path_data["t"]
        self.path_x = path_data["x"]
        self.path_y = path_data["y"]

        # Tracking metrics for localization quality analysis
        self.cumulative_l2_error: float = 0.0  # Running sum of L2 position errors
        self.sample_count: int = 0  # Number of tracking error samples logged

    async def send_velocity_command(
        self, websocket: Any, v_left: float, v_right: float, log: bool = False
    ) -> None:
        """Send velocity command to the WebSocket server.

        Args:
            websocket: Active WebSocket connection.
            v_left: Left wheel velocity in m/s (range: -2.0 to 2.0).
            v_right: Right wheel velocity in m/s (range: -2.0 to 2.0).
            log: Whether to log the command (default: False for quiet operation).
        """
        command = {"v_left": v_left, "v_right": v_right}
        message = json.dumps(command)
        await websocket.send(message)
        if log:
            logging.debug(f"Sent command: v_left={v_left:.3f}, v_right={v_right:.3f}")

    def process_sensor_message(self, data: Dict[str, Any]) -> None:
        """Process sensor data message and log to CSV files.

        Extracts IMU (accelerometer, gyroscope) and GPS data from the sensor
        message and passes them to the data collector for logging.

        Args:
            data: Parsed JSON message containing sensor data.
        """
        sensors = data.get("sensors", [])

        # Validate that sensors is a list
        if not isinstance(sensors, list):
            logging.warning(f"Invalid sensors data type: expected list, got {type(sensors)}")
            return

        # Extract sensor data
        imu_timestamp: Optional[float] = None
        x_dot: Optional[float] = None
        y_dot: Optional[float] = None
        theta_dot: Optional[float] = None

        gps_timestamp: Optional[float] = None
        x: Optional[float] = None
        y: Optional[float] = None

        for sensor in sensors:
            sensor_name = sensor.get("name")
            sensor_data: List[float] = sensor.get("data", [])
            sensor_timestamp = sensor.get("timestamp")

            if sensor_name == "accelerometer" and len(sensor_data) >= 2:
                x_dot = sensor_data[0]
                y_dot = sensor_data[1]
                imu_timestamp = sensor_timestamp

                # Store for IMU integration
                self.current_ax = x_dot
                self.current_ay = y_dot

            elif sensor_name == "gyro" and len(sensor_data) >= 1:
                theta_dot = sensor_data[0]
                if not imu_timestamp:  # Use gyro timestamp if accel not available
                    imu_timestamp = sensor_timestamp

            elif sensor_name == "gps" and len(sensor_data) >= 2:
                x = sensor_data[0]
                y = sensor_data[1]
                gps_timestamp = sensor_timestamp

        # Log IMU data
        if imu_timestamp and (x_dot is not None or y_dot is not None or theta_dot is not None):
            self.data_collector.log_imu(imu_timestamp, x_dot, y_dot, theta_dot)

        # Log GPS data and update localizer
        if gps_timestamp and x is not None and y is not None:
            self.data_collector.log_gps(gps_timestamp, x, y)

            # Update localizer with GPS position
            current_gps = (x, y)

            # Check if this is a fresh GPS reading (not stale/repeated)
            is_fresh_gps = self.last_gps_position is None or current_gps != self.last_gps_position

            if not self.initialized and is_fresh_gps:
                # Initialize localizer at origin (0, 0) as per problem specification
                # The wagon always starts at (0, 0), so we ignore the first GPS reading
                # which may be noisy and cause initialization errors
                print(f"Initializing at origin (0.0, 0.0) - First GPS was: ({x:.4f}, {y:.4f})")
                current_time = time.time()
                self.localizer.update_gps(0.0, 0.0, timestamp=current_time)
                self.initialized = True
            elif self.initialized and is_fresh_gps:
                # Normal GPS update after initialization
                current_time = time.time()
                self.localizer.update_gps(x, y, timestamp=current_time)

            self.last_gps_position = current_gps

        # Update current gyro data for heading estimation
        if theta_dot is not None:
            self.current_theta_dot = theta_dot

    def process_score_message(self, data: Dict[str, Any]) -> None:
        """Process score message and trigger shutdown.

        Args:
            data: Parsed JSON message potentially containing score information.
        """
        message_type = data.get("message_type")

        if message_type == "score":
            score = data.get("score")

            # Log the final score to file
            self.data_collector.log_final_score(score)

            # Calculate system's average distance per sample for better intuition
            # L2_error = Σ(distances) / 20s, so Σ(distances) = L2_error × 20s
            # At 20 Hz for 20s: ~400 samples
            # avg_distance = Σ(distances) / 400 = (L2_error × 20) / 400 = L2_error / 20
            system_avg_mm = (score / 20.0) * 1000.0

            # Calculate our estimated metrics based on localization
            if self.sample_count > 0:
                est_avg_mm = (self.cumulative_l2_error / self.sample_count) * 1000.0
                est_l2_error = self.cumulative_l2_error
                # LOC L2 comparable 20s value (divide cumulative by 20)
                est_l2_20s = est_l2_error / 20.0
                diff_mm = abs(est_avg_mm - system_avg_mm)

                # Display simplified results
                logging.info(f"{TERM_BLUE}\033[1m→ L2: {score:.3f}m  Avg: {system_avg_mm:.1f}mm{TERM_RESET}")
                logging.info(f"{TERM_BLUE}\033[1m→ LOC L2: {est_l2_20s:.3f}m  LOC Avg: {est_avg_mm:.1f}mm  Diff: {diff_mm:.1f}mm{TERM_RESET}")
            else:
                logging.info(f"{TERM_BLUE}\033[1m→ L2: {score:.3f}m  Avg: {system_avg_mm:.1f}mm{TERM_RESET}")

            self.should_stop = True
        else:
            # Unknown message type - log for debugging
            logging.debug(f"\nReceived unknown message: {json.dumps(data, indent=2)}\n")

    def parse_and_route_message(self, message: Union[str, bytes]) -> None:
        """Parse incoming message and route to appropriate handler.

        Args:
            message: Raw JSON message string or bytes from WebSocket.
        """
        try:
            # Handle both str and bytes
            if isinstance(message, bytes):
                message = message.decode("utf-8")
            data = json.loads(message)

            # Route message based on type
            message_type = data.get("message_type")

            if message_type == "sensors":
                self.process_sensor_message(data)
            else:
                # Any other message type is likely the score response
                self.process_score_message(data)

        except json.JSONDecodeError as e:
            logging.error(f"Error parsing JSON: {e}")
        except (KeyError, TypeError, ValueError) as e:
            logging.error(f"Error processing message data: {e}")
        except Exception as e:
            logging.error(f"Unexpected error processing message: {e}", exc_info=True)

    async def run_control_loop(self) -> None:
        """Connect to WebSocket and run the control loop.

        Maintains a connection to the WebSocket server with automatic retry logic
        and exponential backoff. Continues running until should_stop flag is set
        (typically by receiving a score message).
        """
        retry_delay = WS_RETRY_DELAY_SECONDS
        max_retry_delay = WS_MAX_RETRY_DELAY_SECONDS

        while not self.should_stop:
            try:
                async with websockets.connect(self.uri) as websocket:
                    logging.info(f"{TERM_BLUE}✓ Connected to server{TERM_RESET}")
                    retry_delay = (
                        WS_RETRY_DELAY_SECONDS  # Reset retry delay on successful connection
                    )

                    # Control loop: continuously update and send commands
                    control_started = False
                    while not self.should_stop:
                        try:
                            message = await asyncio.wait_for(
                                websocket.recv(), timeout=WS_TIMEOUT_SECONDS
                            )

                            # Parse sensor message and update state
                            message_data = (
                                message.decode("utf-8") if isinstance(message, bytes) else message
                            )
                            parsed_data = json.loads(message_data)
                            message_type = parsed_data.get("message_type")

                            # Always parse and route message first
                            self.parse_and_route_message(message)

                            # Update localizer and compute control (for sensor messages)
                            if message_type == "sensors":
                                current_time = time.time()

                                # Compute path curvature for adaptive sensor fusion
                                if self.follower.start_time is not None:
                                    elapsed_time_for_curvature = current_time - self.follower.start_time
                                else:
                                    elapsed_time_for_curvature = 0.0

                                # Get current path curvature and set in localizer
                                path_curvature = path.reference_curvature(elapsed_time_for_curvature)
                                self.localizer.set_path_curvature(path_curvature)

                                # Update localizer with full IMU (accelerometer + gyro)
                                if self.last_update_time is not None:
                                    dt = current_time - self.last_update_time
                                    if dt > 0:
                                        self.localizer.update_imu(
                                            self.current_ax,
                                            self.current_ay,
                                            self.current_theta_dot,
                                            dt,
                                        )

                                self.last_update_time = current_time

                                # Get localized state
                                state = self.localizer.get_state()

                                # Compute elapsed time for reference trajectory
                                if self.follower.start_time is not None:
                                    elapsed_time = current_time - self.follower.start_time
                                else:
                                    elapsed_time = 0.0

                                # Get reference state at current time
                                ref_state = path.reference_state(elapsed_time)

                                # Log localized state
                                self.data_collector.log_state(
                                    current_time,
                                    state["x"],
                                    state["y"],
                                    state["theta"],
                                    state["v_x"],
                                    state["v_y"],
                                )

                                # Log EKF diagnostics
                                ekf_diagnostics = self.localizer.get_diagnostics()
                                self.data_collector.log_ekf_diagnostics(current_time, ekf_diagnostics)

                                # Log reference state
                                self.data_collector.log_reference(
                                    current_time,
                                    elapsed_time,
                                    ref_state["x"],
                                    ref_state["y"],
                                    ref_state["theta"],
                                )

                                # Calculate and log tracking error metrics
                                error_x = ref_state["x"] - state["x"]
                                error_y = ref_state["y"] - state["y"]
                                error_l2 = math.sqrt(error_x**2 + error_y**2)

                                # Update cumulative tracking metrics
                                self.cumulative_l2_error += error_l2
                                self.sample_count += 1
                                avg_sample_error_mm = (self.cumulative_l2_error / self.sample_count) * 1000.0

                                # Log tracking metrics
                                self.data_collector.log_tracking_metrics(
                                    current_time,
                                    error_x,
                                    error_y,
                                    error_l2,
                                    self.cumulative_l2_error,
                                    self.sample_count,
                                    avg_sample_error_mm,
                                )

                                # Only run control after initialization from first GPS
                                if not self.initialized:
                                    # Wait for first fresh GPS - send zero velocity
                                    v_left, v_right = 0.0, 0.0
                                    await self.send_velocity_command(
                                        websocket, v_left=v_left, v_right=v_right
                                    )
                                else:
                                    # Compute reference commands using pure pursuit with acceleration feedforward
                                    v_ref, omega_ref, a_ref, alpha_ref = self.follower.compute_control(
                                        state, self.path_x, self.path_y, self.path_t, current_time
                                    )

                                    # Compute localized velocities for motor controller feedback
                                    v_loc = self.motor_controller.transform_velocity_to_body_frame(
                                        state["v_x"], state["v_y"], state["theta"]
                                    )
                                    omega_loc = self.current_theta_dot

                                    # Compute dt for motor controller
                                    if self.last_update_time is not None:
                                        dt_motor = current_time - self.last_update_time
                                    else:
                                        dt_motor = 0.05  # Default 50ms if first update

                                    # Apply PI + feedforward motor controller
                                    v_cmd, omega_cmd = self.motor_controller.compute_control(
                                        v_ref, omega_ref, v_loc, omega_loc, dt_motor, a_ref, alpha_ref
                                    )

                                    # Log motor controller data
                                    diagnostics = self.motor_controller.get_diagnostics(
                                        v_ref, omega_ref, v_loc, omega_loc, v_cmd, omega_cmd
                                    )
                                    self.data_collector.log_motor_diagnostics(
                                        current_time, diagnostics
                                    )

                                    # Convert to wheel velocities
                                    v_left, v_right = inverse_kinematics(v_cmd, omega_cmd)

                                    # Send velocity command
                                    await self.send_velocity_command(
                                        websocket, v_left=v_left, v_right=v_right
                                    )

                                # Log first control command
                                if not control_started:
                                    logging.info(
                                        f"{TERM_BLUE}✓ Running pure pursuit path following{TERM_RESET}"
                                    )
                                    control_started = True

                        except asyncio.TimeoutError:
                            # No message received in timeout period, continue
                            continue
                        except websockets.exceptions.ConnectionClosed:
                            logging.warning("Connection closed by server")
                            break

            except Exception as e:
                if self.should_stop:
                    break
                logging.error(f"Connection error: {e}")
                logging.info(f"Retrying in {retry_delay} seconds...")
                await asyncio.sleep(retry_delay)
                retry_delay = min(retry_delay * 2, max_retry_delay)

    def stop(self) -> None:
        """Signal the controller to stop."""
        self.should_stop = True

    def __enter__(self) -> "WagonController":
        """Context manager entry point.

        Returns:
            Self reference for use in with statement.
        """
        self.data_collector.setup()
        return self

    def __exit__(self, exc_type: Any, exc_val: Any, exc_tb: Any) -> None:
        """Context manager exit point - ensures cleanup is called.

        Args:
            exc_type: Exception type if an exception occurred.
            exc_val: Exception value if an exception occurred.
            exc_tb: Exception traceback if an exception occurred.
        """
        self.data_collector.cleanup()


async def main(component_mode: Optional[ComponentMode] = None) -> None:
    """Main entry point for the WebSocket client.

    Creates a WagonController instance, sets up signal handlers for graceful
    shutdown, and starts the control loop.

    Args:
        component_mode: ComponentMode configuration for component isolation testing.
    """
    # Create controller and use as context manager for automatic cleanup
    with WagonController(WS_URI, component_mode=component_mode) as controller:
        # Setup signal handlers for graceful shutdown
        loop = asyncio.get_event_loop()

        def signal_handler() -> None:
            """Handle shutdown signals (SIGINT, SIGTERM)."""
            logging.info("\nShutdown signal received...")
            controller.stop()

        for sig in (signal.SIGINT, signal.SIGTERM):
            loop.add_signal_handler(sig, signal_handler)

        # Start control loop
        await controller.run_control_loop()


if __name__ == "__main__":
    # Parse component isolation flags first
    component_mode, remaining_args = parse_component_flags()

    # Parse remaining command-line arguments
    parser = argparse.ArgumentParser(
        description="WebSocket client for wagon control and sensor data collection"
    )
    parser.add_argument(
        "-v", "--verbose", action="store_true", help="Enable verbose logging with timestamps"
    )
    args = parser.parse_args(remaining_args)

    # Setup logging based on verbose flag
    setup_logging(args.verbose)

    try:
        asyncio.run(main(component_mode=component_mode))
    except KeyboardInterrupt:
        logging.info("\nExiting...")
        sys.exit(0)
