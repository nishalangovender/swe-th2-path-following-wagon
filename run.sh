#!/bin/bash
# Wagon Control System - Automated Setup and Run Script
# This script creates a virtual environment, installs dependencies, and runs the wagon control system
# Usage: ./run.sh [NUM_RUNS] [OPTIONS]
#
# Arguments:
#   NUM_RUNS           Number of sequential runs (default: 1)
#                      Note: Live plots are automatically disabled for multiple runs
#
# Options:
#   -v, --verbose      Enable verbose logging
#   --live             Enable live real-time visualization (default for single run)
#   --no-live          Disable live plotting, use post-run visualization
#   --no-plot          Skip all visualization
#   --plot-only        Only visualize the most recent run (skip data collection)
#   --no-csv           Remove CSV files after run (keep only plots)
#   -h, --help         Show this help message

set -e  # Exit on error

# Color codes for output - Monumental branding
ORANGE='\033[38;2;247;72;35m'         # Monumental orange #f74823
BLUE='\033[38;2;35;116;247m'          # Complementary blue #2374f7
CREAM='\033[38;2;255;253;238m'        # Monumental cream #fffdee
NC='\033[0m' # No Color

# Parse command-line arguments
VERBOSE=""
LIVE_PLOT=true
PLOT_AFTER=true
PLOT_ONLY=false
SAVE_PLOTS="--save"
KEEP_CSV=true
NUM_RUNS=1

show_help() {
    echo "Usage: $0 [NUM_RUNS] [OPTIONS]"
    echo ""
    echo "Wagon Control System - Run data collection and visualization"
    echo ""
    echo "Arguments:"
    echo "  NUM_RUNS           Number of sequential runs (default: 1)"
    echo "                     Note: Live plots are auto-disabled for multiple runs"
    echo ""
    echo "Options:"
    echo "  -v, --verbose      Enable verbose logging with timestamps"
    echo "  --live             Enable live real-time visualization (default for single run)"
    echo "  --no-live          Disable live plotting, show plots after collection"
    echo "  --no-plot          Skip all visualization"
    echo "  --plot-only        Only visualize the most recent run (skip collection)"
    echo "  --no-csv           Remove CSV files after run (keep only plots)"
    echo "  -h, --help         Show this help message"
    echo ""
    echo "Examples:"
    echo "  ./run.sh                    # Single run with live visualization"
    echo "  ./run.sh 5                  # 5 sequential runs (live plots disabled)"
    echo "  ./run.sh 3 --verbose        # 3 runs with verbose logging"
    echo "  ./run.sh --no-live          # Single run, then show plots after completion"
    echo "  ./run.sh --no-plot          # Run data collection only, no plots"
    echo "  ./run.sh --plot-only        # Just visualize the last run"
    exit 0
}

# Check if first argument is a number (NUM_RUNS)
if [[ $# -gt 0 ]] && [[ $1 =~ ^[0-9]+$ ]]; then
    NUM_RUNS=$1
    shift

    # Auto-disable live plots for multiple runs
    if [ "$NUM_RUNS" -gt 1 ]; then
        LIVE_PLOT=false
        echo -e "${ORANGE}[Sequential Mode] Running $NUM_RUNS times (live plots disabled)${NC}"
    fi
fi

while [[ $# -gt 0 ]]; do
    case $1 in
        -v|--verbose)
            VERBOSE="--verbose"
            shift
            ;;
        --live)
            LIVE_PLOT=true
            shift
            ;;
        --no-live)
            LIVE_PLOT=false
            shift
            ;;
        --no-plot)
            PLOT_AFTER=false
            LIVE_PLOT=false
            SAVE_PLOTS=""
            shift
            ;;
        --plot-only)
            PLOT_ONLY=true
            PLOT_AFTER=true
            LIVE_PLOT=false
            shift
            ;;
        --no-csv)
            KEEP_CSV=false
            shift
            ;;
        -h|--help)
            show_help
            ;;
        *)
            echo "Unknown option: $1"
            echo "Use --help for usage information"
            exit 1
            ;;
    esac
done

# Get script directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"
echo -e "${ORANGE}==============================${NC}"
echo -e "${ORANGE}1. Wagon Control System Setup${NC}"
echo -e "${ORANGE}==============================${NC}"

# Check if Python3 is installed
if ! command -v python3 &> /dev/null; then
    echo -e "${CREAM}Error: python3 is not installed${NC}"
    echo "Please install Python 3 and try again"
    exit 1
fi

PYTHON_VERSION=$(python3 --version)
echo -e "${BLUE}✓ Found $PYTHON_VERSION${NC}"

# Create virtual environment if it doesn't exist
if [ ! -d "venv" ]; then
    python3 -m venv venv
    echo -e "${BLUE}✓ Created virtual environment${NC}"
else
    echo -e "${BLUE}✓ Using existing virtual environment${NC}"
fi

# Activate virtual environment
source venv/bin/activate

# Setup cleanup function to deactivate venv on exit
cleanup() {
    deactivate 2>/dev/null || true
}

# Trap EXIT signal to ensure cleanup runs
trap cleanup EXIT

# Upgrade pip
pip install --upgrade pip -q 2>/dev/null
echo -e "${BLUE}✓ Upgraded pip${NC}"

# Install requirements
pip install -r requirements.txt -q 2>/dev/null
echo -e "${BLUE}✓ Installed dependencies${NC}"

echo -e "${BLUE}✓ Completed setup!${NC}"

# Create run directory for live plotting if needed
RUN_DIR=""
LIVE_PLOT_PID=""
if [ "$LIVE_PLOT" = true ] && [ "$PLOT_ONLY" = false ]; then
    # Create run directory with timestamp
    TIMESTAMP=$(date +"%Y%m%d_%H%M%S")
    RUN_DIR="results/run_${TIMESTAMP}"
    mkdir -p "$RUN_DIR"

    # Create empty CSV files for live plotter to monitor
    echo "timestamp,x,y" > "$RUN_DIR/gps_data.csv"
    echo "timestamp,x_dot,y_dot,theta_dot" > "$RUN_DIR/imu_data.csv"
    echo -e "${ORANGE}================================${NC}"
    echo -e "${ORANGE}2. Starting Live Visualization"
    echo -e "${ORANGE}================================${NC}"

    # Start live plotter in background with auto-save enabled
    python -m wagon_control.live_plot "$RUN_DIR" --save &
    LIVE_PLOT_PID=$!

    # Give the plotter a moment to start
    sleep 1
fi

# Run data collection unless --plot-only is specified
if [ "$PLOT_ONLY" = false ]; then
    echo -e "${ORANGE}=================================${NC}"
    echo -e "${ORANGE}3. Starting Wagon Control System${NC}"
    echo -e "${ORANGE}=================================${NC}"
    if [ -n "$VERBOSE" ]; then
        echo -e "${CREAM}Running in verbose mode${NC}"
    fi

    # Run the wagon control system NUM_RUNS times
    for ((run=1; run<=NUM_RUNS; run++)); do
        if [ "$NUM_RUNS" -gt 1 ]; then
            echo -e "${ORANGE}--- Run $run of $NUM_RUNS ---${NC}"
        fi

        # Run the wagon control system
        if [ -n "$RUN_DIR" ]; then
            # Pass run directory to client for live plotting
            RUN_DIR="$RUN_DIR" python -m wagon_control.client $VERBOSE
        else
            python -m wagon_control.client $VERBOSE
        fi

        # Brief pause between runs (except after the last one)
        if [ "$NUM_RUNS" -gt 1 ] && [ "$run" -lt "$NUM_RUNS" ]; then
            echo -e "${CREAM}Waiting 2 seconds before next run...${NC}"
            sleep 2
        fi
    done
fi

# Run post-visualization if requested (non-live mode)
if [ "$PLOT_AFTER" = true ] && [ "$LIVE_PLOT" = false ]; then
    python -m wagon_control.plot_results $SAVE_PLOTS
fi

# Wait for live plotter if it's running
if [ -n "$LIVE_PLOT_PID" ]; then
    echo -e "${BLUE}✓ Completed data collection${NC}"

    # Check if process is still running
    if kill -0 $LIVE_PLOT_PID 2>/dev/null; then
        echo -e "${CREAM}Live plot is still running. Close the plot window to exit.${NC}"
    fi

    wait $LIVE_PLOT_PID 2>/dev/null || true

    # Save static plots after live plotting completes
    if [ -n "$RUN_DIR" ]; then
        python -m wagon_control.plot_results --run "$(basename "$RUN_DIR")" --save --no-show
    fi
fi

# Clean up CSV files if requested
if [ "$KEEP_CSV" = false ] && [ -n "$RUN_DIR" ]; then
    echo -e "${CREAM}Removing CSV files (keeping plots only)...${NC}"
    rm -f "$RUN_DIR"/*.csv
    echo -e "${BLUE}✓ CSV files removed${NC}"
fi

echo -e "${BLUE}✓ Done${NC}"