"""
@file example_basic_Drag_Interference.py
@brief Example demonstrating joint-based drag mode with real-time
       collision detection for the L1 robot (both arms).

Workflow overview:
    1. Initialize communication with the robot controller
    2. Retrieve SDK and controller versions
    3. Initialize collision detection with configuration files (The configuration
       file must be selected according to the robot model)
    4. Switch ARM0 and ARM1 into joint drag mode using stiffness and damping
       parameters(STATE_DRAG_JOINT)(Check and recover the arm state (ERROR → IDLE))
    5. Loop for approx. 30s: continuously acquire left and right arm joint
       angles -> perform collision detection (log collisions without exiting)
    6. Switch both arms to IDLE state

In STATE_DRAG_JOINT, each joint can be physically guided by hand,
while collision distances are computed at approximately 100 Hz.

@warning Ensure the robot arms are in a safe position and workspace
         before entering drag mode to avoid collisions or unexpected motion.

@warning Press Enter key at any time to stop the robot and exit.
"""

import sys
import time
import threading
from pathlib import Path

root_dir = Path(__file__).parent.parent
if str(root_dir) not in sys.path:
    sys.path.insert(0, str(root_dir))

from PYTHON_SDK.GentoRobot import GentoRobot, FXLogMask, FXObjMask, FXObjType, state_map, error_dict

def main():
    def emergency_stop_thread(robot):
        input()
        print("\n[Emergency stop] Triggered. Stopping robot...")
        robot.emergency_stop(FXObjMask.OBJ_ALL_FLAG)
        time.sleep(0.1)
        print("Exiting program.")
        sys.exit(0)

    ctrl_obj0=FXObjType.OBJ_ARM0
    ctrl_obj1=FXObjType.OBJ_ARM1
    target_state="DragJoint"
    log_mask=FXLogMask.FX_LOG_INFO_FLAG
    interf_threshold = 10
    run_seconds = 30.0
    k=[3, 3, 3, 2, 1, 1, 1]
    d=[0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2]

    config_dir = root_dir / "C_SDK" / "FXConfig" / "InterferenceCfg" / "GentoLuna-Standard"
    cord_def = str(config_dir / "CordDef.Cord")
    cal_link_def = str(config_dir / "CalLinkDef.Links")
    input_def = str(config_dir / "CalInputDef.Joints")
    convex_def = str(config_dir / "ConvexDef.Convex")
    ic_def = str(config_dir / "InterfDef.Interf")

    robot = GentoRobot()
    threading.Thread(target=emergency_stop_thread, args=(robot,), daemon=True).start()

    print(f"\n### 1/7 .Connecting...")
    ret = robot.link(6, 6, 7, 190, log_level=log_mask)
    if not robot._connected:
        print(f"Link failed. Error msg: {robot._get_operate_error_msg(ret)}")
        return

    print(f"\n### 2/7. Getting versions...")
    print(f"Controller version:{robot.get_controller_version()}")
    print(f"Sdk version:{robot.get_sdk_version()}")

    print(f"\n### 3/7. Initializing collision detection...")
    ret = robot.interf_init(cord_def, cal_link_def, input_def, convex_def, ic_def)
    if ret != 0:
        print(f"Interference init failed. Error msg: {robot._get_operate_error_msg(ret)}")
        return

    print(f"\n### 4/7. Drag joint state switching...")
    for ctrl_obj in (ctrl_obj0,ctrl_obj1):
        arm_state=robot.current_state(ctrl_obj)
        if state_map[arm_state] == "Error":
            ret, system_errorcode = robot.reset_error(ctrl_obj, 1000)
            if ret != 0:
                print(f"Reset error failed. Error msg: {error_dict[system_errorcode]}")
                return
        elif state_map[arm_state] != "IDLE":
            ret = robot.switch_to_idle(ctrl_obj, 1000)
            if ret != 0:
                print(f"Switch to IDLE failed. Error msg: {robot._get_operate_error_msg(ret)}")
                return

    for ctrl_obj in (ctrl_obj0,ctrl_obj1):
        ret=robot.switch_to_drag_joint(ctrl_obj,2000,k,d)
        if ret !=0:
            print(f"Switch to {target_state} failed. Error msg: {robot._get_operate_error_msg(ret)}")
            return
    print(f"Arm0 and Arm1 are in STATE_DRAG_JOINT state now, please press the drag button")

    print(f"\n### 5/7. Running {run_seconds}s drag + interference check loop...")
    ret, input_num = robot.interf_get_cords_num()
    if ret != 0:
        print(f"Get input number failed. Error msg: {robot._get_operate_error_msg(ret)}")
        return

    loop_start = time.monotonic()
    while True:
        elapsed = time.monotonic() - loop_start
        if elapsed >= run_seconds:
            break

        # Acquire joint states of body(6) + arm0(7) + arm1(7) + head(2), total 22
        # matching the loaded CalInputDef.Joints definition
        rt_dict = robot.get_rt_dict()
        input_pos = list(rt_dict["body"]["fb_pos"])
        input_pos += list(rt_dict["arms"][0]["fb"]["fb_pos"])
        input_pos += list(rt_dict["arms"][1]["fb"]["fb_pos"])
        input_pos += list(rt_dict["head"]["fb_pos"][:2])

        # Update poses, update convex hulls and compute collision distances
        robot.interf_update_cord(input_num, input_pos)
        robot.interf_update_convex()
        robot.interf_calc_interf_distance()

        # Check interference for each collision pair
        ret, min_spans = robot.interf_get_min_spans()
        if ret == 0:
            for i, span in enumerate(min_spans):
                if span < interf_threshold:
                    print(f"interference detected, min_span[{i}] = {span:.2f}")
        time.sleep(0.01)  # 100Hz

    print(f"loop finished")

    print(f"\n### 6/7. Switching to IDLE...")
    for ctrl_obj in (ctrl_obj0,ctrl_obj1):
        ret = robot.switch_to_idle(ctrl_obj, 2000)
        if ret != 0:
            print(f"Switch to IDLE failed. Error msg: {robot._get_operate_error_msg(ret)}")
            return

    print(f"\n### 7/7. Task finished.")

if __name__ == "__main__":
    main()
