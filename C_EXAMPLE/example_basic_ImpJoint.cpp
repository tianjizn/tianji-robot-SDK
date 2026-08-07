/**
 * @file example_basic_ImpJoint.cpp
 * @brief Example demonstrating how to switch the L1 robot arm into
 *        joint-space impedance control mode (STATE_IMP_JOINT).
 *
 * This example shows how to:
 * - Query SDK and controller versions
 * - Recover the robot arm from ERROR state
 * - Switch the arm to IDLE state
 * - Enter joint impedance control mode (STATE_IMP_JOINT)
 * - Command joint target positions under impedance control
 * - Return the arm to IDLE state
 *
 * In STATE_IMP_JOINT, the arm behaves like a compliant mechanism
 * at the joint level, controlled by stiffness and damping parameters.
 *
 * @warning Ensure the robot arm is in a safe position and workspace
 *          before entering impedance control mode.
 */

#include "L1Robot.h"

/**
 * @brief Entry point of the joint impedance control example.
 *
 * Workflow overview:
 * 1. Initialize communication with the robot controller
 * 2. Retrieve SDK and controller versions
 * 3. Check and recover the arm state (ERROR → IDLE)
 * 4. Switch ARM0 into joint impedance mode (STATE_IMP_JOINT)
 * 5. Move the arm to two different joint target positions
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
     * Joint stiffness coefficients for impedance control.
     * Index mapping:
     * - [0..5]: Joint 1–6
     * - [6]   : Nullspace stiffness
     */
    double k[7] = { 3, 3, 3, 2, 1, 1, 1 };

    /**
     * Joint damping coefficients for impedance control.
     * Index mapping matches the stiffness array.
     */
    double d[7] = { 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2 };

    double vel_ratio = 10.0;              ///< Velocity ratio for impedance mode
    double acc_ratio = 10.0;              ///< Acceleration ratio for impedance mode

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

    /* Switch to joint impedance control mode */
    printf("Arm0 is in STATE_IDLE state now, press any key to transfer to STATE_IMP_JOINT state\n");
    getchar();
    if (FX_L1_State_SwitchToImpJointMode(
            FX_OBJ_ARM0,
            2000,
            vel_ratio,
            acc_ratio,
            k,
            d) != FUNC_RET_SUCCESS)
    {
        printf("Failed to transfer arm0 to STATE_IMP_JOINT state\n");
        goto WAIT_EXIT;
    }

    /* Move to first target position */
    printf("Arm0 is in STATE_IMP_JOINT state now\n");
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

    /* Return to IDLE state */
    printf("Set arm0's target position success, arm0 is moving...\n");
    printf("Press any key to transfer to STATE_IDLE state\n");
    getchar();
    if (FX_L1_State_SwitchToIdle(FX_OBJ_ARM0, 1000) != FUNC_RET_SUCCESS)
    {
        printf("Failed to transfer arm0 to STATE_IDLE state\n");
        goto WAIT_EXIT;
    }

    printf("Arm0 is in STATE_IDLE state now\n");

WAIT_EXIT:
    printf("Press any key to exit\n");
    getchar();
    return 0;
}