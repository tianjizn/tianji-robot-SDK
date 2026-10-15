/**
 * @file example_basic_DragJoint.cpp
 * @brief Example demonstrating how to switch the L1 robot arm into
 *        joint-space drag mode (STATE_DRAG_JOINT).
 *
 * This example shows how to:
 * - Query SDK and controller versions
 * - Recover the robot arm from ERROR state
 * - Switch the arm to IDLE state
 * - Enter joint drag mode using stiffness and damping parameters
 * - Exit drag mode and return to IDLE state
 *
 * In STATE_DRAG_JOINT, each joint can be physically guided by hand,
 * and the arm behaves according to per-joint impedance control.
 *
 * @warning Ensure the robot arm is in a safe position and workspace
 *          before entering drag mode to avoid collisions or unexpected motion.
 */

#include "L1Robot.h"

/**
 * @brief Entry point of the joint-space drag mode example.
 *
 * Workflow overview:
 * 1. Initialize communication with the robot controller
 * 2. Retrieve SDK and controller versions
 * 3. Check and recover the arm state (ERROR → IDLE)
 * 4. Switch ARM0 into joint drag mode (STATE_DRAG_JOINT)
 * 5. Allow manual dragging via the arm terminal button
 * 6. Return the arm to IDLE state
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

    /**
     * Joint stiffness coefficients for drag mode.
     * Index mapping:
     * - [0..5]: Joint 1–6
     * - [6]   : Nullspace stiffness
     */
    double k[7] = { 3, 3, 3, 2, 1, 1, 1 };

    /**
     * Joint damping coefficients for drag mode.
     * Index mapping matches the stiffness array.
     */
    double d[7] = { 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2 };

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

    /* Switch to joint drag mode */
    printf("Arm0 is in STATE_IDLE state now, press any key to transfer to STATE_DRAG_JOINT state\n");
    getchar();
    if (FX_L1_State_SwitchToDragJoint(FX_OBJ_ARM0, 2000, k, d) != FUNC_RET_SUCCESS)
    {
        printf("Failed to transfer arm0 to STATE_DRAG_JOINT state\n");
        goto WAIT_EXIT;
    }

    /* Manual dragging */
    printf("Arm0 is in STATE_DRAG_JOINT state now, please press the drag button "
           "on arm0's terminal to drag the arm\n");
    printf("Press any key to let arm0 transfer to STATE_IDLE state.\n");
    getchar();

    /* Return to IDLE state */
    if (FX_L1_State_SwitchToIdle(FX_OBJ_ARM0, 1000) != FUNC_RET_SUCCESS)
    {
        printf("Failed to transfer arm0 to STATE_IDLE state\n");
        goto WAIT_EXIT;
    }

WAIT_EXIT:
    printf("Press any key to exit\n");
    getchar();
    return 0;
}
