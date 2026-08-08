# ArmIn.msg - Arm input command structure
# Corresponds to FXCommon.h ARM_IN structure

float32[7] cmd_joint_tor
float32[7] cmd_joint_pos
int16 cmd_ctrl_drag_type
int16 cmd_ctrl_force_type
float32[5] cmd_ctrl_force_dir
float32[5] cmd_ctrl_torque_dir
int32 cmd_tag
int16 cmd_pd_serial
int16 cmd_ctrl_ref_ori_type
float32[3] cmd_ctrl_ref_ori
