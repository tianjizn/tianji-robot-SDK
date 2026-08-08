# ArmSet.msg - Arm configuration and control parameters
# Corresponds to FXCommon.h ARM_SET structure

int32 ctrl_imp_type               # Impedance control type
float32 ctrl_vel_ratio            # Velocity scaling ratio
float32 ctrl_acc_ratio            # Acceleration scaling ratio
float32[7] ctrl_joint_k           # Joint stiffness
float32[7] ctrl_joint_d           # Joint damping
float32[7] ctrl_cart_k            # Cartesian stiffness
float32[7] ctrl_cart_d            # Cartesian damping
float32[6] ctrl_tool_kine         # Tool kinematics
float32[10] ctrl_tool_dyna        # Tool dynamics