/**
 * @file example_basic_Emergency.cpp
 * @brief Example demonstrating how to trigger an emergency stop (E-Stop)
 *        on the L1 robot arm during motion.
 *
 * This example shows how to:
 * - Query SDK and controller versions
 * - Recover the robot arm from ERROR state
 * - Switch the arm to IDLE and then POSITION mode
 * - Command joint-space motion targets
 * - Trigger an emergency stop during motion
 * - Observe the resulting arm state after E-Stop
 *
 * The emergency stop immediately halts motion and transitions the arm
 * into a protective state.
 *
 * @warning An emergency stop may cause abrupt deceleration.
 *          Ensure the robot is in a safe configuration before testing.
 */

#include "L1Robot.h"

#ifdef _WIN32
#define SLEEP_MS(ms) Sleep(ms)
#else
#define SLEEP_MS(ms) usleep((ms) * 1000)
#endif

/**
 * @brief Convert FXStateType enumeration to human-readable string.
 *
 * @param[in] state_type Robot state type returned by the feedback interface.
 * @return const char* String representation of the state.
 */
const char* StateType2Str(FXStateType state_type)
{
    switch (state_type)
    {
        case FX_STATE_IDLE:        return "STATE_IDLE";
        case FX_STATE_POSITION:    return "STATE_POSITION";
        case FX_STATE_IMP_JOINT:   return "STATE_IMP_JOINT";
        case FX_STATE_IMP_CART:    return "STATE_IMP_CART";
        case FX_STATE_IMP_FORCE:   return "STATE_IMP_FORCE";
        case FX_STATE_DRAG_JOINT:  return "STATE_DRAG_JOINT";
        case FX_STATE_DRAG_CART_X: return "STATE_DRAG_CART_X";
        case FX_STATE_DRAG_CART_Y: return "STATE_DRAG_CART_Y";
        case FX_STATE_DRAG_CART_Z: return "STATE_DRAG_CART_Z";
        case FX_STATE_DRAG_CART_R: return "STATE_DRAG_CART_R";
        case FX_STATE_RELEASE:     return "STATE_RELEASE";
        case FX_STATE_PD:          return "STATE_PD";
        case FX_STATE_ERROR:       return "STATE_ERROR";
        case FX_STATE_TRANSFERRING:return "STATE_TRANSFERRING";
        case FX_STATE_UNKNOWN:
        default:                   return "STATE_UNKNOWN";
    }
}

/**
 * @brief Entry point of the emergency stop example.
 *
 * Workflow overview:
 * 1. Initialize communication with the robot controller
 * 2. Retrieve SDK and controller versions
 * 3. Check and recover the arm state (ERROR → IDLE)
 * 4. Switch ARM0 into POSITION mode
 * 5. Move the arm to two different joint positions
 * 6. Trigger an emergency stop during motion
 * 7. Print the resulting arm state
 *
 * @param[in] argc Argument count (unused)
 * @param[in] argv Argument vector (unused)
 * @return int Exit code (0 on normal exit)
 */
int main(int argc, char** argv)
{
    int sdk_version = 0;                  ///< SDK version number
    int controller_version = 0;           ///< Controller firmware version
    unsigned int system_errorcode = 0;    ///< Last system error code
    FXStateType obj_state = FX_STATE_UNKNOWN;
    FXStateType cur_state = FX_STATE_UNKNOWN;

    double vel_ratio = 10.0;              ///< Velocity ratio for position mode
    double acc_ratio = 10.0;              ///< Acceleration ratio for position mode

    /**
     * First target joint position (degrees).
     * Index mapping:
     * - [0..5]: Joint 1–6
     * - [6]   : Nullspace / auxiliary joint
     */
    double pos1[7] = { 0, 0, 0, 0, 0, 0, 0 };

    /**
     * Second target joint position (degrees).
     */
    double pos2[7] = { 10, 10, 10, 90, 10, 10, 10 };

    /* Get SDK version */
    sdk_version = FX_L1_System_GetSDKVersion();
    printf("SDK version is 0x%08x\n", sdk_version);

    /* Establish communication with the controller */
    if (FX_L1_System_Link(6, 6, 7, 190, FX_LOG_ALL_FLAG) < 0)
    {
        printf("Failed to link system\n");
        goto WAIT_EXIT;
    }

    /* Get controller version */
    controller_version = FX_L1_System_GetControllerVersion();
    printf("Controller version is 0x%08x\n", controller_version);

    /* Check current arm state */
    obj_state = FX_L1_Fbk_CurrentState(FX_OBJ_ARM0);
    if (obj_state == FX_STATE_ERROR)
    {
        printf("Arm0 is in STATE_ERROR state now, press any key to reset error\n");
        getchar();
        if (FX_L1_State_ResetError(FX_OBJ_ARM0, 1000, &system_errorcode) == FUNC_RET_SUCCESS)
        {
            printf("Reset arm0 error success, arm0 is now in STATE_IDLE state\n");
        }
        else
        {
            printf("Failed to reset arm0 error, errorcode = 0x%08x\n", system_errorcode);
            goto WAIT_EXIT;
        }
    }
    else if (obj_state != FX_STATE_IDLE)
    {
        printf("Arm0 is not in STATE_IDLE state now, press any key to transfer to STATE_IDLE state\n");
        getchar();
        if (FX_L1_State_SwitchToIdle(FX_OBJ_ARM0, 1000) != FUNC_RET_SUCCESS)
        {
            printf("Failed to transfer arm0 to STATE_IDLE state\n");
            goto WAIT_EXIT;
        }
    }

    /* Switch to position mode */
    printf("Arm0 is in STATE_IDLE state now, press any key to transfer to STATE_POSITION state\n");
    getchar();
    if (FX_L1_State_SwitchToPositionMode(FX_OBJ_ARM0, 2000, vel_ratio, acc_ratio) != FUNC_RET_SUCCESS)
    {
        printf("Failed to transfer arm0 to STATE_POSITION state\n");
        goto WAIT_EXIT;
    }

    /* Move to first target position */
    printf("Arm0 is in STATE_POSITION state now\n");
    printf("Press any key to let arm0 move to target position "
           "{%.4lf,%.4lf,%.4lf,%.4lf,%.4lf,%.4lf,%.4lf}\n",
           pos1[0], pos1[1], pos1[2], pos1[3], pos1[4], pos1[5], pos1[6]);
    getchar();

    if (FX_L1_Runtime_SetJointPosCmd(1, FX_OBJ_ARM0, pos1) != FUNC_RET_SUCCESS)
    {
        printf("Failed to set arm0's target position\n");
        goto WAIT_EXIT;
    }

    /* Move to second target position */
    printf("Set arm0's target position success, arm0 is moving...\n");
    printf("Press any key to let arm0 move to target position "
           "{%.4lf,%.4lf,%.4lf,%.4lf,%.4lf,%.4lf,%.4lf}\n",
           pos2[0], pos2[1], pos2[2], pos2[3], pos2[4], pos2[5], pos2[6]);
    getchar();

    if (FX_L1_Runtime_SetJointPosCmd(1, FX_OBJ_ARM0, pos2) != FUNC_RET_SUCCESS)
    {
        printf("Failed to set arm0's target position\n");
        goto WAIT_EXIT;
    }

    /* Trigger emergency stop */
    printf("Set arm0's target position success, arm0 is moving...\n");
    printf("Press any key to trigger arm0's emergency stop\n");
    getchar();

    if (FX_L1_Runtime_EmergencyStop(1, FX_OBJ_ARM0_FLAG) != FX_OBJ_ARM0_FLAG)
    {
        printf("Failed to trigger arm0's emergency stop\n");
        goto WAIT_EXIT;
    }

    printf("Arm0's emergency stop is triggered\n");
    SLEEP_MS(200);

    cur_state = FX_L1_Fbk_CurrentState(FX_OBJ_ARM0);
    printf("Arm0 is in %s state now\n", StateType2Str(cur_state));

WAIT_EXIT:
    printf("Press any key to exit\n");
    getchar();
    return 0;
}
