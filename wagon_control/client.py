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
import csv
import json
import logging
import os
import signal
import sys
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, List, Optional, TextIO, Tuple, Union
import time

import websockets

from wagon_control.model import inverse_kinematics
from wagon_control.follower import PurePursuitFollower
from wagon_control.localizer import WagonLocalizer
from wagon_control.motor_controller import MotorController
from wagon_control import path

# Terminal color codes - Monumental branding
TERM_ORANGE = "\033[38;2;247;72;35m"  # Monumental orange
TERM_BLUE = "\033[38;2;35;116;247m"  # Complementary blue
TERM_RESET = "\033[0m"  # Reset color


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


# Configuration Constants
WS_URI: str = "ws://91.99.103.188:8765"
# Test velocity commands (to be replaced by path follower)
TEST_V_CMD: float = 0.5  # Linear velocity (m/s)
TEST_OMEGA_CMD: float = 1.0  # Angular velocity (rad/s)
RETRY_DELAY_SECONDS: int = 1
MAX_RETRY_DELAY_SECONDS: int = 60
WEBSOCKET_TIMEOUT_SECONDS: float = 5.0


class SensorDataCollector:
    """Collects sensor data from WebSocket server and writes to separate CSV files.

    This class manages a WebSocket connection to collect IMU (accelerometer, gyroscope)
    and GPS data from a wagon simulation server. Data is written to timestamped CSV
    files in separate directories for each run.

    The class can be used as a context manager to ensure proper cleanup of resources.

    Attributes:
        uri: WebSocket URI to connect to.
        imu_csv_file: File handle for IMU data CSV.
        imu_csv_writer: CSV writer for IMU data.
        gps_csv_file: File handle for GPS data CSV.
        gps_csv_writer: CSV writer for GPS data.
        should_stop: Flag indicating whether to stop data collection.
        run_dir: Directory path for this run's output files.
        imu_output_path: Path to IMU CSV file.
        gps_output_path: Path to GPS CSV file.
    """

    def __init__(self, uri: str, output_dir: str = ".") -> None:
        """Initialize the sensor data collector.

        Args:
            uri: WebSocket URI to connect to (must start with ws:// or wss://).
            output_dir: Base directory for output files (default: current directory).

        Raises:
            ValueError: If URI format is invalid or output_dir is not a directory.
        """
        # Validate URI format
        if not uri or not uri.startswith(("ws://", "wss://")):
            raise ValueError(f"Invalid WebSocket URI: {uri}. Must start with 'ws://' or 'wss://'")

        # Validate output directory
        output_path = Path(output_dir)
        if output_path.exists() and not output_path.is_dir():
            raise ValueError(f"Output path exists but is not a directory: {output_dir}")

        self.uri: str = uri
        self.imu_csv_file: Optional[TextIO] = None
        self.imu_csv_writer: Any = None
        self.gps_csv_file: Optional[TextIO] = None
        self.gps_csv_writer: Any = None
        self.state_csv_file: Optional[TextIO] = None
        self.state_csv_writer: Any = None
        self.reference_csv_file: Optional[TextIO] = None
        self.reference_csv_writer: Any = None
        self.motor_csv_file: Optional[TextIO] = None
        self.motor_csv_writer: Any = None
        self.should_stop: bool = False

        # State estimation and control
        self.localizer = WagonLocalizer()
        self.follower = PurePursuitFollower(lookahead_distance=1.0)  # Optimized through testing (16-20m error)
        self.motor_controller = MotorController(
            k_v=0.5, k_omega=0.5,      # Proportional gains (stable baseline)
            k_i_v=0.1, k_i_omega=0.1   # Integral gains
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
        path_data = path.path_trajectory(t_max=20.0, dt=0.1)
        self.path_t = path_data['t']
        self.path_x = path_data['x']
        self.path_y = path_data['y']

        # Check if run directory is provided via environment variable (for live plotting)
        env_run_dir = os.environ.get("RUN_DIR")
        if env_run_dir:
            # Use provided run directory
            self.run_dir: Path = Path(env_run_dir)
            # Ensure directory exists
            self.run_dir.mkdir(parents=True, exist_ok=True)
        else:
            # Create results folder structure: results/run_YYYYMMDD_HHMMSS/
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            base_dir = output_path
            results_dir = base_dir / "results"
            self.run_dir = results_dir / f"run_{timestamp}"

            # Create directory structure
            self.run_dir.mkdir(parents=True, exist_ok=True)

        # Create output filenames
        self.imu_output_path: Path = self.run_dir / "imu_data.csv"
        self.gps_output_path: Path = self.run_dir / "gps_data.csv"
        self.state_output_path: Path = self.run_dir / "state_data.csv"
        self.reference_output_path: Path = self.run_dir / "reference_data.csv"
        self.motor_output_path: Path = self.run_dir / "motor_data.csv"

    def setup_csv(self) -> None:
        """Initialize CSV files with headers.

        Creates and opens IMU, GPS, state, and reference CSV files with appropriate column headers.
        """
        # Setup IMU CSV file
        self.imu_csv_file = open(self.imu_output_path, "w", newline="")
        self.imu_csv_writer = csv.writer(self.imu_csv_file)
        self.imu_csv_writer.writerow(["timestamp", "x_dot", "y_dot", "theta_dot"])
        self.imu_csv_file.flush()

        # Setup GPS CSV file
        self.gps_csv_file = open(self.gps_output_path, "w", newline="")
        self.gps_csv_writer = csv.writer(self.gps_csv_file)
        self.gps_csv_writer.writerow(["timestamp", "x", "y"])
        self.gps_csv_file.flush()

        # Setup State CSV file (localized estimates)
        self.state_csv_file = open(self.state_output_path, "w", newline="")
        self.state_csv_writer = csv.writer(self.state_csv_file)
        self.state_csv_writer.writerow(["timestamp", "x_loc", "y_loc", "theta_loc", "v_x", "v_y"])
        self.state_csv_file.flush()

        # Setup Reference CSV file (path follower's time-based interpretation)
        self.reference_csv_file = open(self.reference_output_path, "w", newline="")
        self.reference_csv_writer = csv.writer(self.reference_csv_file)
        self.reference_csv_writer.writerow(["timestamp", "elapsed_time", "x_ref", "y_ref", "theta_ref"])
        self.reference_csv_file.flush()

        # Setup Motor CSV file (motor controller feedback)
        self.motor_csv_file = open(self.motor_output_path, "w", newline="")
        self.motor_csv_writer = csv.writer(self.motor_csv_file)
        self.motor_csv_writer.writerow(["timestamp", "v_ref", "omega_ref", "v_loc", "omega_loc", "v_err", "omega_err", "v_cmd", "omega_cmd"])
        self.motor_csv_file.flush()

        logging.info(
            f"{TERM_BLUE}✓ Initialized data collection to results/{self.run_dir.name}/{TERM_RESET}"
        )

    def cleanup(self) -> None:
        """Close CSV files and log final output location."""
        if self.imu_csv_file:
            self.imu_csv_file.close()
        if self.gps_csv_file:
            self.gps_csv_file.close()
        if self.state_csv_file:
            self.state_csv_file.close()
        if self.reference_csv_file:
            self.reference_csv_file.close()
        if self.motor_csv_file:
            self.motor_csv_file.close()

        logging.info(f"{TERM_BLUE}✓ Saved sensor data to results/{self.run_dir.name}/{TERM_RESET}")

    async def send_velocity_command(self, websocket: Any, v_left: float, v_right: float, log: bool = False) -> None:
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
        """Process sensor data message and write to CSV files.

        Extracts IMU (accelerometer, gyroscope) and GPS data from the sensor
        message and writes them to their respective CSV files.

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

        # Write IMU data to IMU file
        if imu_timestamp and (x_dot is not None or y_dot is not None or theta_dot is not None):
            self.imu_csv_writer.writerow(
                [
                    imu_timestamp,
                    x_dot if x_dot is not None else "",
                    y_dot if y_dot is not None else "",
                    theta_dot if theta_dot is not None else "",
                ]
            )
            if self.imu_csv_file:
                self.imu_csv_file.flush()

        # Write GPS data to GPS file
        if gps_timestamp and (x is not None or y is not None):
            self.gps_csv_writer.writerow(
                [gps_timestamp, x if x is not None else "", y if y is not None else ""]
            )
            if self.gps_csv_file:
                self.gps_csv_file.flush()

            # Update localizer with GPS position
            if x is not None and y is not None:
                current_gps = (x, y)

                # Check if this is a fresh GPS reading (not stale/repeated)
                is_fresh_gps = (self.last_gps_position is None or
                               current_gps != self.last_gps_position)

                if not self.initialized and is_fresh_gps:
                    # First fresh GPS reading - initialize localizer position
                    print(f"Initializing from first GPS: ({x:.4f}, {y:.4f})")
                    self.localizer.x = x
                    self.localizer.y = y
                    self.localizer.theta = 0.0
                    self.localizer.v_x = 0.0
                    self.localizer.v_y = 0.0
                    self.initialized = True
                elif self.initialized and is_fresh_gps:
                    # Normal GPS update after initialization (only when GPS actually changes)
                    # Pass current time for GPS velocity computation
                    current_time = time.time()
                    self.localizer.update_gps(x, y, timestamp=current_time)

                self.last_gps_position = current_gps

        # Update current gyro data for heading estimation
        if theta_dot is not None:
            self.current_theta_dot = theta_dot

    def process_l2_message(self, data: Dict[str, Any]) -> None:
        """Process L2 score message and trigger shutdown.

        Args:
            data: Parsed JSON message potentially containing score information.
        """
        message_type = data.get("message_type")

        if message_type == "score":
            score = data.get("score")
            logging.info(f"{TERM_BLUE}\033[1m✓ Received score: {score:.2f}m\033[0m{TERM_RESET}")
            self.should_stop = True
        else:
            # Unknown message type - log for debugging
            logging.debug(f"\nReceived unknown message: {json.dumps(data, indent=2)}\n")

    def parse_and_write_message(self, message: Union[str, bytes]) -> None:
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
                # Any other message type is likely the L2 response
                self.process_l2_message(data)

        except json.JSONDecodeError as e:
            logging.error(f"Error parsing JSON: {e}")
        except (KeyError, TypeError, ValueError) as e:
            logging.error(f"Error processing message data: {e}")
        except Exception as e:
            logging.error(f"Unexpected error processing message: {e}", exc_info=True)

    async def collect_data(self) -> None:
        """Connect to WebSocket and collect data continuously.

        Maintains a connection to the WebSocket server with automatic retry logic
        and exponential backoff. Continues collecting data until should_stop flag
        is set (typically by receiving a score message).
        """
        retry_delay = RETRY_DELAY_SECONDS
        max_retry_delay = MAX_RETRY_DELAY_SECONDS

        while not self.should_stop:
            try:
                async with websockets.connect(self.uri) as websocket:
                    logging.info(f"{TERM_BLUE}✓ Connected to server{TERM_RESET}")
                    retry_delay = RETRY_DELAY_SECONDS  # Reset retry delay on successful connection

                    # Control loop: continuously update and send commands
                    control_started = False
                    while not self.should_stop:
                        try:
                            message = await asyncio.wait_for(
                                websocket.recv(), timeout=WEBSOCKET_TIMEOUT_SECONDS
                            )

                            # Parse sensor message and update state
                            message_data = message.decode("utf-8") if isinstance(message, bytes) else message
                            parsed_data = json.loads(message_data)
                            message_type = parsed_data.get("message_type")

                            # Always parse and write message to CSV first
                            self.parse_and_write_message(message)

                            # Update localizer and compute control (for sensor messages)
                            if message_type == "sensors":
                                current_time = time.time()

                                # Update localizer with full IMU (accelerometer + gyro)
                                if self.last_update_time is not None:
                                    dt = current_time - self.last_update_time
                                    if dt > 0:
                                        self.localizer.update_imu(
                                            self.current_ax,
                                            self.current_ay,
                                            self.current_theta_dot,
                                            dt
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
                                self.state_csv_writer.writerow([
                                    current_time,
                                    state['x'],
                                    state['y'],
                                    state['theta'],
                                    state['v_x'],
                                    state['v_y']
                                ])
                                if self.state_csv_file:
                                    self.state_csv_file.flush()

                                # Log reference state (path follower's time-based interpretation)
                                self.reference_csv_writer.writerow([
                                    current_time,
                                    elapsed_time,
                                    ref_state['x'],
                                    ref_state['y'],
                                    ref_state['theta']
                                ])
                                if self.reference_csv_file:
                                    self.reference_csv_file.flush()

                                # Only run control after initialization from first GPS
                                if not self.initialized:
                                    # Wait for first fresh GPS - send zero velocity
                                    v_left, v_right = 0.0, 0.0
                                    await self.send_velocity_command(websocket, v_left=v_left, v_right=v_right)
                                else:
                                    # Compute reference commands using pure pursuit
                                    v_ref, omega_ref = self.follower.compute_control(
                                        state,
                                        self.path_x, self.path_y, self.path_t,
                                        current_time
                                    )

                                    # Compute localized velocities for motor controller feedback
                                    v_loc = self.motor_controller.transform_velocity_to_body_frame(
                                        state['v_x'], state['v_y'], state['theta']
                                    )
                                    omega_loc = self.current_theta_dot

                                    # Compute dt for motor controller (time since last update)
                                    if self.last_update_time is not None:
                                        dt_motor = current_time - self.last_update_time
                                    else:
                                        dt_motor = 0.05  # Default 50ms if first update

                                    # Apply PI motor controller feedback
                                    v_cmd, omega_cmd = self.motor_controller.compute_control(
                                        v_ref, omega_ref, v_loc, omega_loc, dt_motor
                                    )

                                    # Log motor controller data
                                    diagnostics = self.motor_controller.get_diagnostics(
                                        v_ref, omega_ref, v_loc, omega_loc, v_cmd, omega_cmd
                                    )
                                    self.motor_csv_writer.writerow([
                                        current_time,
                                        diagnostics['v_ref'],
                                        diagnostics['omega_ref'],
                                        diagnostics['v_loc'],
                                        diagnostics['omega_loc'],
                                        diagnostics['v_err'],
                                        diagnostics['omega_err'],
                                        diagnostics['v_cmd'],
                                        diagnostics['omega_cmd']
                                    ])
                                    if self.motor_csv_file:
                                        self.motor_csv_file.flush()

                                    # Convert to wheel velocities
                                    v_left, v_right = inverse_kinematics(v_cmd, omega_cmd)

                                    # Send velocity command
                                    await self.send_velocity_command(websocket, v_left=v_left, v_right=v_right)

                                # Log first control command
                                if not control_started:
                                    logging.info(f"{TERM_BLUE}✓ Running pure pursuit path following{TERM_RESET}")
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
        """Signal the collector to stop data collection."""
        self.should_stop = True

    def __enter__(self) -> "SensorDataCollector":
        """Context manager entry point.

        Returns:
            Self reference for use in with statement.
        """
        self.setup_csv()
        return self

    def __exit__(self, exc_type: Any, exc_val: Any, exc_tb: Any) -> None:
        """Context manager exit point - ensures cleanup is called.

        Args:
            exc_type: Exception type if an exception occurred.
            exc_val: Exception value if an exception occurred.
            exc_tb: Exception traceback if an exception occurred.
        """
        self.cleanup()


async def main() -> None:
    """Main entry point for the WebSocket client.

    Creates a SensorDataCollector instance, sets up signal handlers for graceful
    shutdown, and starts the data collection process.
    """
    # Create collector and use as context manager for automatic cleanup
    with SensorDataCollector(WS_URI) as collector:
        # Setup signal handlers for graceful shutdown
        loop = asyncio.get_event_loop()

        def signal_handler() -> None:
            """Handle shutdown signals (SIGINT, SIGTERM)."""
            logging.info("\nShutdown signal received...")
            collector.stop()

        for sig in (signal.SIGINT, signal.SIGTERM):
            loop.add_signal_handler(sig, signal_handler)

        # Start collecting data
        await collector.collect_data()


if __name__ == "__main__":
    # Parse command-line arguments
    parser = argparse.ArgumentParser(
        description="WebSocket client for wagon control and sensor data collection"
    )
    parser.add_argument(
        "-v", "--verbose", action="store_true", help="Enable verbose logging with timestamps"
    )
    args = parser.parse_args()

    # Setup logging based on verbose flag
    setup_logging(args.verbose)

    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        logging.info("\nExiting...")
        sys.exit(0)
