"""
@file example_kine_LunaBody_BasicKinematics.py
@brief Example demonstrating Luna body coordinated forward and inverse kinematics.

This example does NOT connect to a real controller. It initializes the Luna body
kinematics model from input parameters and runs a forward/inverse kinematics
verification loop, mirroring the C example `example_kine_LunaBody_BasicKinematics.cpp`.

Workflow overview:
    1. Create the standalone kinematics context
    2. Initialize Luna body kinematics from input parameters
    3. Forward kinematics: body joints -> left/right shoulder poses
    4. Inverse kinematics: verify the shoulder poses recover the joints
    5. Apply a TCP offset, then recompute inverse kinematics
    6. Forward kinematics again to verify the offset joint solution
    7. Release the kinematics context

"""

import sys
from pathlib import Path

root_dir = Path(__file__).parent.parent
if str(root_dir) not in sys.path:
    sys.path.insert(0, str(root_dir))
from PYTHON_SDK.GentoRobot import GentoRobot, FXLogMask


def print_matrix(name, matrix):
    print(f"{name}:")
    for row in range(4):
        print(" ".join(f"{matrix[row][col]:10.4f}" for col in range(4)))


def print_body_joints(name, joints):
    print(f"{name} {{{', '.join(f'{v:.4f}' for v in joints)}}}")


def main():
    log_mask = FXLogMask.FX_LOG_ALL_FLAG

    # Luna body standalone kinematics parameters (same as the C example).
    flange_length = 391.5
    joint_limit_pos = [11.0, 90.0, 10.0, 55.0, 30.0, 170.0]
    joint_limit_neg = [-11.0, -10.0, -140.0, -110.0, -30.0, -170.0]
    dh_luna_body = [
        [0.0, 0.0, 0.0, 0.0],
        [-90.0, 165.0, 0.0, 0.0],
        [0.0, 300.0, 0.0, 0.0],
        [0.0, 300.0, 0.0, 0.0],
        [90.0, 0.0, 0.0, 90.0],
        [90.0, 0.0, 0.0, 0.0],
    ]

    luna_joints = [10.15, 20.22, -30.33, -40.44, 10.15, 156.0]

    print("\n### 1/7. Create kinematics context...")
    robot = GentoRobot()
    # The kinematics handle is created lazily on the first kinematics call.
    robot.kine_log_level(log_mask)

    print("\n### 2/7. Initialize Luna body kinematics from input parameters...")
    ret = robot.luna_body_init_by_input_params(dh_luna_body, flange_length,
                                               joint_limit_neg, joint_limit_pos)
    if ret != 0:
        print(f"Failed to initialize Luna body kinematics. Error code: {ret}")
        robot.cleanup()
        return
    print("Luna body kinematics initialized")

    print("\n### 3/7. Forward kinematics: body joints -> shoulder poses...")
    ret = robot.luna_body_forward_kinematics(luna_joints)
    if isinstance(ret, tuple):
        arm0_shoulder_tcp, arm1_shoulder_tcp = ret
        print_matrix("arm0_shoulder_tcp", arm0_shoulder_tcp)
        print_matrix("arm1_shoulder_tcp", arm1_shoulder_tcp)
    else:
        print(f"Failed to verify Luna body forward kinematics. Error code: {ret}")
        robot.cleanup()
        return

    print("\n### 4/7. Inverse kinematics: verify the shoulder poses recover the joints...")
    ret = robot.luna_body_inverse_kinematics(arm0_shoulder_tcp, arm1_shoulder_tcp, luna_joints)
    if isinstance(ret, list):
        verify_joints = ret
        print_body_joints("verify_joints", verify_joints)
    else:
        print(f"Failed to verify Luna body inverse kinematics. Error code: {ret}")
        robot.cleanup()
        return

    print("\n### 5/7. Apply a TCP offset and recompute inverse kinematics...")
    arm0_shoulder_tcp[0][3] += 10
    arm1_shoulder_tcp[0][3] += 10
    ret = robot.luna_body_inverse_kinematics(arm0_shoulder_tcp, arm1_shoulder_tcp, luna_joints)
    if isinstance(ret, list):
        offset_joints = ret
        print_body_joints("offset_joints", offset_joints)
    else:
        print(f"Failed to verify Luna body inverse kinematics. Error code: {ret}")
        robot.cleanup()
        return

    print("\n### 6/7. Forward kinematics on the offset joint solution...")
    ret = robot.luna_body_forward_kinematics(offset_joints)
    if isinstance(ret, tuple):
        arm0_shoulder_tcp_offset, arm1_shoulder_tcp_offset = ret
        print_matrix("arm0_shoulder_tcp_offset", arm0_shoulder_tcp_offset)
        print_matrix("arm1_shoulder_tcp_offset", arm1_shoulder_tcp_offset)
    else:
        print(f"Failed to verify Luna body forward kinematics. Error code: {ret}")
        robot.cleanup()
        return

    robot.cleanup()
    print("\n### 7/7. Task finished.")


if __name__ == "__main__":
    main()
