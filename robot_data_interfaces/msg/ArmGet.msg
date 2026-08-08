# ArmGet.msg - Arm extended feedback structure
# Corresponds to FXCommon.h ARM_GET structure

float32[7] fbk_joint_tor
float32[7] fbk_joint_ext_pos
int8[7] fbk_joint_temp
int8 fbk_flange_di
int8 fbk_low_spd_flag
int8 fbk_traj_state
int8 fbk_pd_cmd_quality