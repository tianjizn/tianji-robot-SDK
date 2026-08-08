#!/bin/bash

colcon build --packages-select robot_data_interfaces --parallel-workers 11 --event-handlers console_direct+
