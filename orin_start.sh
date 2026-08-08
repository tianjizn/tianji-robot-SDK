#!/bin/bash
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
export LD_LIBRARY_PATH="$SCRIPT_DIR/test_examples/lib:$LD_LIBRARY_PATH"
source $SCRIPT_DIR/install/setup.bash
ros2 run test_examples example_basic_pd_orin
