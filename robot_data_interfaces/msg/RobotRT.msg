# RobotRT.msg - Real-time robot data structure (1000 Hz)
# Corresponds to FXCommon.h ROBOT_RT structure

int32    rt_frame_serial
DeviceRT device
HeadRT   head
ArmRT[2] arms
BodyRT   body
LiftRT   lift
SystemRT system