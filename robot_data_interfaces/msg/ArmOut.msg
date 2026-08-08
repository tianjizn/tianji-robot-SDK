# ArmOut.msg - Arm feedback output structure
# Corresponds to FXCommon.h ARM_OUT structure

float32[7] fbk_joint_pos
float32[7] fbk_joint_vel
float32[7] fbk_joint_cmd
float32[7] fbk_joint_sensor_tor
float32[7] fbk_joint_external_tor_est
float32[6] fbk_base_fn_est
float32[6] fbk_flange_ft_sensor
float32[7] fbk_joint_gravity_tor