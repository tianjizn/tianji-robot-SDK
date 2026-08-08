#!/bin/bash
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
source $SCRIPT_DIR/install/setup.bash
ros2 run test_examples example_basic_pd_user
