"""
Main entry point when running the wagon_control module with python -m.
"""

import argparse
import asyncio
import sys

from .client import main, setup_logging

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
        import logging

        logging.info("\nExiting...")
        sys.exit(0)
