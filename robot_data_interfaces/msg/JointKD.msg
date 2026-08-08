# JointKD.msg - Joint stiffness and damping coefficients
# Used for impedance control

uint32 thread_id              # Thread ID for this call (range 1~7)
uint8 obj_type                # Object type
float32[7] k                  # Joint stiffness coefficients
float32[7] d                  # Joint damping coefficients