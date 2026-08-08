# StateCtr.msg - State control structure
# Corresponds to FXCommon.h StateCtr structure

uint16 cur_state    # Current state machine state
uint16 cmd_state    # Commanded target state
uint32 err_code     # Error code if any