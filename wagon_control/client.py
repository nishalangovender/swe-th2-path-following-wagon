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
from typing import Any, Dict, List, Optional, TextIO, Union

import websockets

from wagon_control.model import inverse_kinematics

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
        self.should_stop: bool = False

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

    def setup_csv(self) -> None:
        """Initialize CSV files with headers.

        Creates and opens IMU and GPS CSV files with appropriate column headers.
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

        logging.info(
            f"{TERM_BLUE}✓ Initialized data collection to results/{self.run_dir.name}/{TERM_RESET}"
        )

    def cleanup(self) -> None:
        """Close CSV files and log final output location."""
        if self.imu_csv_file:
            self.imu_csv_file.close()
        if self.gps_csv_file:
            self.gps_csv_file.close()

        logging.info(f"{TERM_BLUE}✓ Saved sensor data to results/{self.run_dir.name}/{TERM_RESET}")

    async def send_velocity_command(self, websocket: Any, v_left: float, v_right: float) -> None:
        """Send velocity command to the WebSocket server.

        Args:
            websocket: Active WebSocket connection.
            v_left: Left wheel velocity in m/s (range: -2.0 to 2.0).
            v_right: Right wheel velocity in m/s (range: -2.0 to 2.0).
        """
        command = {"v_left": v_left, "v_right": v_right}
        message = json.dumps(command)
        await websocket.send(message)
        logging.info(f"{TERM_BLUE}✓ Running path following{TERM_RESET}")

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

                    # Compute wheel velocities using inverse kinematics
                    v_left, v_right = inverse_kinematics(TEST_V_CMD, TEST_OMEGA_CMD)
                    logging.info(
                        f"Sending velocities: v_cmd={TEST_V_CMD} m/s, "
                        f"omega_cmd={TEST_OMEGA_CMD} rad/s → "
                        f"v_left={v_left:.3f} m/s, v_right={v_right:.3f} m/s"
                    )

                    # Send velocity command once after connection
                    await self.send_velocity_command(websocket, v_left=v_left, v_right=v_right)

                    while not self.should_stop:
                        try:
                            message = await asyncio.wait_for(
                                websocket.recv(), timeout=WEBSOCKET_TIMEOUT_SECONDS
                            )
                            self.parse_and_write_message(message)

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
