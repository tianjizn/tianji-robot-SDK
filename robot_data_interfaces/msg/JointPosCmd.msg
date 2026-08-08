# JointPosCmd.msg - Joint position command message
# Used for runtime motion control

uint32 thread_id              # Thread ID for this call (range 1~7)
uint8 obj_type                # Object type (FXObjType: ARM0=0, ARM1=1)
float64[7] pos_cmd            # Joint position command array