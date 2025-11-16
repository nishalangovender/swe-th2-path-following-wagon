#!/bin/bash
# Quick test of parameter sweep with minimal runs
# Usage: ./test_sweep.sh

set -e

echo "Running quick parameter sweep test (2 runs per config, limited params)..."
echo "This will test the script functionality before running the full sweep."
echo ""

# Activate venv if it exists
if [ -d "venv" ]; then
    source venv/bin/activate
fi

# Run with just 2 runs and verbose output for testing
python parameter_sweep.py --num-runs 2 --verbose

echo ""
echo "Test complete! If this worked, you can run the full sweep with:"
echo "  python parameter_sweep.py --num-runs 10"
