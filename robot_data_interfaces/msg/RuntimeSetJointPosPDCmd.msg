# JointPosPDCmd.msg - Joint position PD command message
# Used for PD mode runtime motion control (high frequency)

std_msgs/Header header        # Header with timestamp for latency measurement
uint32 thread_id              # Thread ID for this call (range 1~7)
uint8 obj_type                # Object type (FXObjType: ARM0=0, ARM1=1)
float64[7] pos_cmd            # Joint position command array
