#!/bin/bash
# Analysis Framework - Automated Setup and Run Script
# This script creates a virtual environment, installs dependencies, and runs parameter sweeps
# Usage: ./analysis/test.sh [COMMAND] [OPTIONS]

set -e  # Exit on error

# Color codes for output - Monumental branding
ORANGE='\033[38;2;247;72;35m'         # Monumental orange #f74823
BLUE='\033[38;2;35;116;247m'          # Complementary blue #2374f7
CREAM='\033[38;2;255;253;238m'        # Monumental cream #fffdee
NC='\033[0m' # No Color

show_help() {
    echo -e "${ORANGE}═══════════════════════════════════════════════════════════════════════${NC}"
    echo -e "${ORANGE}  Analysis Framework - Parameter Sweep Testing${NC}"
    echo -e "${ORANGE}═══════════════════════════════════════════════════════════════════════${NC}"
    echo ""
    echo "Usage: $0 [COMMAND] [OPTIONS]"
    echo ""
    echo "Commands:"
    echo "  sweep              Run a parameter sweep"
    echo "  visualize          Visualize existing sweep results"
    echo "  stats              Generate statistical report"
    echo ""
    echo "Sweep Options:"
    echo "  --param NAME=v1,v2,...  Parameter to sweep (can specify multiple)"
    echo "  --runs N                Runs per configuration (default: 10)"
    echo "  --threshold T           Invalid score threshold in meters (default: 50.0)"
    echo "  --output FILE           Output CSV path (default: auto-generated)"
    echo "  --visualize             Generate plots after sweep"
    echo "  --report                Generate statistical report after sweep"
    echo "  --top N                 Show top N configs (default: 10)"
    echo "  -v, --verbose           Enable verbose logging"
    echo ""
    echo "Visualize Options:"
    echo "  CSV_FILE                Path to sweep results CSV"
    echo "  --output-dir DIR        Output directory for plots"
    echo ""
    echo "Stats Options:"
    echo "  CSV_FILE                Path to sweep results CSV"
    echo "  --output FILE           Output report path"
    echo "  --top N                 Show top N configs (default: 10)"
    echo ""
    echo "Examples:"
    echo "  # Sweep MOTOR_KP_V with 3 values"
    echo "  ./analysis/test.sh sweep --param MOTOR_KP_V=1.0,1.5,2.0 --runs 10"
    echo ""
    echo "  # Sweep multiple parameters"
    echo "  ./analysis/test.sh sweep \\"
    echo "    --param MOTOR_KP_V=1.0,1.5,2.0 \\"
    echo "    --param FOLLOWER_BASE_LOOKAHEAD=0.6,0.8,1.0 \\"
    echo "    --runs 5 --visualize --report"
    echo ""
    echo "  # Visualize existing results"
    echo "  ./analysis/test.sh visualize results/parameter_sweep_20250101_120000.csv"
    echo ""
    echo "  # Generate statistical report"
    echo "  ./analysis/test.sh stats results/parameter_sweep_20250101_120000.csv"
    echo ""
    exit 0
}

# Check for help flag
if [[ $# -eq 0 ]] || [[ "$1" == "-h" ]] || [[ "$1" == "--help" ]]; then
    show_help
fi

# Detect project root (script is in analysis/ subdirectory)
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"
cd "$PROJECT_ROOT"

VENV_DIR="venv"
PYTHON="python3"

echo -e "${ORANGE}═══════════════════════════════════════════════════════════════════════${NC}"
echo -e "${ORANGE}  Analysis Framework Setup${NC}"
echo -e "${ORANGE}═══════════════════════════════════════════════════════════════════════${NC}"
echo ""

# Check if virtual environment exists
if [ ! -d "$VENV_DIR" ]; then
    echo -e "${BLUE}Creating virtual environment...${NC}"
    $PYTHON -m venv "$VENV_DIR"
    echo -e "${CREAM}✓ Virtual environment created${NC}"
else
    echo -e "${CREAM}✓ Virtual environment exists${NC}"
fi

# Activate virtual environment
echo -e "${BLUE}Activating virtual environment...${NC}"
source "$VENV_DIR/bin/activate"

# Check if dependencies are installed
DEPS_INSTALLED=false
if python -c "import numpy, matplotlib" 2>/dev/null; then
    DEPS_INSTALLED=true
fi

# Install/upgrade dependencies if needed
if [ "$DEPS_INSTALLED" = false ]; then
    echo -e "${BLUE}Installing dependencies...${NC}"
    pip install -q --upgrade pip
    pip install -q -r requirements.txt
    echo -e "${CREAM}✓ Dependencies installed${NC}"
else
    echo -e "${CREAM}✓ Dependencies installed${NC}"
fi

echo ""
echo -e "${ORANGE}═══════════════════════════════════════════════════════════════════════${NC}"
echo -e "${ORANGE}  Running Analysis Framework${NC}"
echo -e "${ORANGE}═══════════════════════════════════════════════════════════════════════${NC}"
echo ""

# Run the CLI with all arguments passed through
python -m analysis.cli "$@"

EXIT_CODE=$?

echo ""
if [ $EXIT_CODE -eq 0 ]; then
    echo -e "${CREAM}✓ Analysis complete${NC}"
else
    echo -e "${ORANGE}✗ Analysis failed with exit code $EXIT_CODE${NC}"
fi

deactivate

exit $EXIT_CODE
