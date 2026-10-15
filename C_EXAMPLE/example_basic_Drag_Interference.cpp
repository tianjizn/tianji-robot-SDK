/**
 * @file example_basic_Drag_Interference.cpp
 * @brief Example demonstrating joint-based drag mode with real-time
 *        collision detection for the L1 robot (both arms).
 *
 * This example shows how to:
 * - Query SDK and controller versions
 * - Recover the robot arms from ERROR state
 * - Switch the arms to IDLE state
 * - Initialize collision detection with configuration files
 * - Enter joint drag mode using stiffness and damping parameters
 * - Run a loop that continuously reads joint states and performs
 *   collision detection between the arms, body and head
 * - Exit drag mode and return to IDLE state
 *
 * In STATE_DRAG_JOINT, each joint can be physically guided by hand,
 * while collision distances are computed at approximately 100 Hz.
 * Collisions are logged without stopping the demo.
 *
 * The configuration files must be selected according to the robot model.
 *
 * @warning Ensure the robot arms are in a safe position and workspace
 *          before entering drag mode to avoid collisions or unexpected motion.
 */

#include "L1Robot.h"
#include <stdio.h>
#include <chrono>
#ifdef _WIN32
    #include <windows.h>
    #define SLEEP(ms) Sleep(ms)
#else
    #include <unistd.h>
    #define SLEEP(ms) usleep((ms) * 1000)
#endif

/**
 * @brief Entry point of the drag mode with collision detection example.
 *
 * Workflow overview:
 * 1. Initialize communication with the robot controller
 * 2. Retrieve SDK and controller versions
 * 3. Initialize collision detection with configuration files
 * 4. Check and recover the arm states (ERROR → IDLE)
 * 5. Switch ARM0 and ARM1 into joint drag mode (STATE_DRAG_JOINT)
 * 6. Loop for approx. 30 s: acquire joint states of both arms and
 *    perform collision detection (log collisions without exiting)
 * 7. Return both arms to IDLE state
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
    FXStateType obj_state0 = FX_STATE_UNKNOWN;
    FXStateType obj_state1 = FX_STATE_UNKNOWN;
    double interf_threshold = 10;         ///< Collision detection threshold (mm)
    FX_InterfHandle interf_handle = NULL; ///< Collision detection handle
    int interf_ret = 0;                   ///< Collision detection return code

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

    /* Initialize collision detection */
    /* The configuration file must be selected according to the robot model */
    interf_handle = FX_Interf_Create();
    interf_ret = FX_Interf_Init(interf_handle,
        (char*)"../C_SDK/FXConfig/InterferenceCfg/GentoLuna-Standard/CordDef.Cord",
        (char*)"../C_SDK/FXConfig/InterferenceCfg/GentoLuna-Standard/CalLinkDef.Links",
        (char*)"../C_SDK/FXConfig/InterferenceCfg/GentoLuna-Standard/CalInputDef.Joints",
        (char*)"../C_SDK/FXConfig/InterferenceCfg/GentoLuna-Standard/ConvexDef.Convex",
        (char*)"../C_SDK/FXConfig/InterferenceCfg/GentoLuna-Standard/InterfDef.Interf");
    if (interf_ret != FUNC_RET_SUCCESS)
    {
        printf("Failed to init collision detection, errorcode = 0x%08x\n", interf_ret);
        goto WAIT_EXIT;
    }

    /* Check current arm0 state */
    obj_state0 = FX_L1_Fbk_CurrentState(FX_OBJ_ARM0);
    if (obj_state0 == FX_STATE_ERROR)
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
    else if (obj_state0 != FX_STATE_IDLE)
    {
        printf("Arm0 is not in STATE_IDLE state now, press any key to transfer to STATE_IDLE state\n");
        getchar();
        if (FX_L1_State_SwitchToIdle(FX_OBJ_ARM0, 1000) != FUNC_RET_SUCCESS)
        {
            printf("Failed to transfer arm0 to STATE_IDLE state\n");
            goto WAIT_EXIT;
        }
    }

    /* Check current arm1 state */
    obj_state1 = FX_L1_Fbk_CurrentState(FX_OBJ_ARM1);
    if (obj_state1 == FX_STATE_ERROR)
    {
        printf("Arm1 is in STATE_ERROR state now, press any key to reset error\n");
        getchar();
        if (FX_L1_State_ResetError(FX_OBJ_ARM1, 1000, &system_errorcode) == FUNC_RET_SUCCESS)
        {
            printf("Reset arm1 error success, arm1 is now in STATE_IDLE state\n");
        }
        else
        {
            printf("Failed to reset arm1 error, errorcode = 0x%08x\n", system_errorcode);
            goto WAIT_EXIT;
        }
    }
    else if (obj_state1 != FX_STATE_IDLE)
    {
        printf("Arm1 is not in STATE_IDLE state now, press any key to transfer to STATE_IDLE state\n");
        getchar();
        if (FX_L1_State_SwitchToIdle(FX_OBJ_ARM1, 1000) != FUNC_RET_SUCCESS)
        {
            printf("Failed to transfer arm1 to STATE_IDLE state\n");
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

    printf("Arm1 is in STATE_IDLE state now, press any key to transfer to STATE_DRAG_JOINT state\n");
    getchar();
    if (FX_L1_State_SwitchToDragJoint(FX_OBJ_ARM1, 2000, k, d) != FUNC_RET_SUCCESS)
    {
        printf("Failed to transfer arm1 to STATE_DRAG_JOINT state\n");
        goto WAIT_EXIT;
    }

    /* Manual dragging with collision detection */
    printf("Arm0 and Arm1 are in STATE_DRAG_JOINT state now, please press the drag button\n");

    /**
     * Loop for approx. 30 s: continuously acquire the joint states of
     * both arms (14 joints), the body (6 joints) and the head (3 joints),
     * then update the collision model and compute collision distances.
     */
    {
        auto loop_start = std::chrono::high_resolution_clock::now();
        double run_seconds = 30.0;
        const ROBOT_RT* rt_ptr = FX_L1_Fbk_GetRT(); ///< Real-time feedback pointer

        /* Get input position number */
        long input_num = 0;
        long interf_num = 0;
        if (FX_Interf_GetInputNum(interf_handle, &input_num) != FUNC_RET_SUCCESS)
        {
            printf("Failed to get input number\n");
            goto WAIT_EXIT;
        }
        double input_pos[input_num] = {0};
        if (FX_Interf_GetInterfNum(interf_handle, &interf_num) != FUNC_RET_SUCCESS)
        {
            printf("Failed to get interference number\n");
            goto WAIT_EXIT;
        }
        double min_span[interf_num] = {0};

        while (true)
        {
            auto now_time = std::chrono::high_resolution_clock::now();
            double elapsed = std::chrono::duration<double>(now_time - loop_start).count();
            if (elapsed >= run_seconds) { break; }

            /* Acquire joint states of both arms, the body and the head */
            for (int i = 0; i < 6; i++)
            {
                input_pos[i] = rt_ptr->m_BODY.m_BODY_OUT.m_BODY_FBK_Joint_Pos[i];
            }
            for (int i = 0; i < 7; i++)
            {
                input_pos[i + 6] = rt_ptr->m_ARMS[0].m_ARM_OUT.m_ARM_FBK_Joint_Pos[i];
                input_pos[i + 13] = rt_ptr->m_ARMS[1].m_ARM_OUT.m_ARM_FBK_Joint_Pos[i];
            }
            for (int i = 0; i < 2; i++)
            {
                input_pos[20 + i] = rt_ptr->m_HEAD.m_HEAD_OUT.m_HEAD_FBK_Joint_Pos[i];
            }

            /* Update poses, update convex hulls and compute collision distances */
            FX_Interf_UpdateCord(interf_handle, input_num, input_pos);
            FX_Interf_UpdateConvex(interf_handle);
            FX_Interf_CalInterfDistance(interf_handle);

            /* Check interference */
            FX_Interf_GetMinSpan(interf_handle, min_span);
            for (int i = 0; i < interf_num; i++)
            {
                if (min_span[i] < interf_threshold)
                {
                    /* If the value hasn't updated, it might be because the distance is greater than 200 */
                    printf("interference detected, min_span[%d] = %.2f\n", i, min_span[i]);
                }
            }
            SLEEP(10);  /* 100Hz */
        }
    }

    printf("drag + interference check loop finished\n");

    /* Return to IDLE state */
    if (FX_L1_State_SwitchToIdle(FX_OBJ_ARM0, 1000) != FUNC_RET_SUCCESS)
    {
        printf("Failed to transfer arm0 to STATE_IDLE state\n");
        goto WAIT_EXIT;
    }
    if (FX_L1_State_SwitchToIdle(FX_OBJ_ARM1, 1000) != FUNC_RET_SUCCESS)
    {
        printf("Failed to transfer arm1 to STATE_IDLE state\n");
        goto WAIT_EXIT;
    }
    SLEEP(500);

WAIT_EXIT:
    printf("Press any key to exit\n");
    getchar();
    FX_L1_System_Unlink();
    return 0;
}
