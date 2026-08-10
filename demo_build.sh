#!/bin/bash

colcon build --packages-select test_examples --parallel-workers 11 --event-handlers console_direct+
