/**
 * @file example_basic_DragJoint2ImpCart.cpp
 * @brief Example demonstrating a transition from joint-space drag mode
 *        to Cartesian impedance control mode on the L1 robot arm.
 *
 * This example shows how to:
 * - Query SDK and controller versions
 * - Recover the robot arm from ERROR state
 * - Switch the arm to IDLE state
 * - Enter joint-space drag mode (STATE_DRAG_JOINT)
 * - Transition from joint drag mode to Cartesian impedance mode (STATE_IMP_CART)
 * - Return the arm to IDLE state
 *
 * This workflow is useful for applications that combine manual guidance
 * with compliant Cartesian interaction.
 *
 * @warning Ensure the robot arm is in a safe position and workspace
 *          before entering any drag or impedance control mode.
 */

#include "L1Robot.h"

/**
 * @brief Entry point of the joint-drag to Cartesian-impedance example.
 *
 * Workflow overview:
 * 1. Initialize communication with the robot controller
 * 2. Retrieve SDK and controller versions
 * 3. Check and recover the arm state (ERROR → IDLE)
 * 4. Switch ARM0 into joint drag mode (STATE_DRAG_JOINT)
 * 5. Manually drag the arm using the terminal button
 * 6. Switch to Cartesian impedance mode (STATE_IMP_CART)
 * 7. Return the arm to IDLE state
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
     * Joint stiffness coefficients for STATE_DRAG_JOINT.
     * Index mapping:
     * - [0..5]: Joint 1–6
     * - [6]   : Nullspace stiffness
     */
    double joint_k[7] = { 3, 3, 3, 2, 1, 1, 1 };

    /**
     * Joint damping coefficients for STATE_DRAG_JOINT.
     * Index mapping matches the stiffness array.
     */
    double joint_d[7] = { 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2 };

    /**
     * Cartesian stiffness coefficients for STATE_IMP_CART.
     * Index mapping:
     * - [0..2]: Translation X, Y, Z
     * - [3..5]: Rotation X, Y, Z
     * - [6]   : Nullspace stiffness
     */
    double cart_k[7] = { 2000, 2000, 2000, 100, 100, 100, 50 };

    /**
     * Cartesian damping coefficients for STATE_IMP_CART.
     * Index mapping matches the stiffness array.
     */
    double cart_d[7] = { 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 1.0 };

    double vel_ratio = 10.0;              ///< Velocity ratio for impedance mode
    double acc_ratio = 10.0;              ///< Acceleration ratio for impedance mode

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
    if (FX_L1_State_SwitchToDragJoint(FX_OBJ_ARM0, 2000, joint_k, joint_d) != FUNC_RET_SUCCESS)
    {
        printf("Failed to transfer arm0 to STATE_DRAG_JOINT state\n");
        goto WAIT_EXIT;
    }

    /* Manual dragging in joint mode */
    printf("Arm0 is in STATE_DRAG_JOINT state now, please press the drag button "
           "on arm0's terminal to drag the arm\n");
    printf("Press any key to let arm0 transfer to STATE_IMP_CART state.\n");
    getchar();

    /* Switch to Cartesian impedance mode */
    if (FX_L1_State_SwitchToImpCartMode(
            FX_OBJ_ARM0,
            1000,
            vel_ratio,
            acc_ratio,
            cart_k,
            cart_d) != FUNC_RET_SUCCESS)
    {
        printf("Failed to transfer arm0 to STATE_IMP_CART state\n");
        goto WAIT_EXIT;
    }

    /* Exit impedance mode */
    printf("Arm0 is in STATE_IMP_CART state now, press any key to transfer to STATE_IDLE\n");
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