/**
 * @file example_basic_MIT.cpp
 * @brief Example demonstrating how to switch the L1 robot arm into
 *        MIT control mode (STATE_MIT).
 *
 * This example shows how to:
 * - Query SDK and controller versions
 * - Recover the robot arm from ERROR state
 * - Switch the arm to IDLE state
 * - Enter MIT control mode (STATE_MIT)
 * - Send real-time MIT commands at 2ms intervals
 * - Control the arm for a total of 10 seconds
 * - Return the arm to IDLE state
 *
 * In STATE_MIT, the arm accepts joint position, velocity, and torque commands
 * with configurable PD gains. The controller expects commands at 2ms intervals.
 * If no command is received for more than 10ms, the controller will stop
 * the arm at its current position.
 *
 * @warning IMPORTANT SAFETY NOTICE — Read carefully before use
 *
 * [Overview]
 * This example performs PD position-hold control at the current pose. If no MIT
 * command is sent for 10 seconds, the internal controller will automatically
 * switch to holding the current joint positions.
 *
 * [Scope]
 * MIT mode supports only the left and right arms (FX_OBJ_ARM0 / FX_OBJ_ARM1).
 * The Body module does NOT support this feature. Do not enable MIT mode on
 * the Body object.
 *
 * [Risk Warning]
 * The control performance of MIT mode depends primarily on the external
 * controller's real-time capability and algorithm quality. In this mode, the
 * robot's internal controller provides no additional trajectory protection or
 * safety intervention. The following conditions may degrade control performance
 * and cause runaway motion, abnormal vibration, or other unpredictable behavior:
 *   - Insufficient real-time performance of the external controller (unstable
 *     command cycle, excessive network latency);
 *   - Trajectories that exceed joint range or physical limits;
 *   - Discontinuous or non-smooth trajectories (e.g., step changes, jumps);
 *   - Unreasonable PD parameters (stiffness / damping).
 *
 * [Prerequisites]
 * This feature is intended for professional users with expertise in real-time
 * robot control, impedance parameter tuning, and kinematics theory. Fully
 * understand the operating principles and potential risks of MIT mode before
 * use, and conduct thorough testing in a safe environment.
 *
 * [Disclaimer]
 * FX Robotics shall not be liable for any equipment damage, personal injury, or
 * property loss arising from the use of MIT mode. Users must independently
 * assess the risks and take necessary safety measures (including but not limited
 * to emergency-stop devices, physical barriers, and safe distances). Use of this
 * feature constitutes the user's acknowledgment and acceptance of all foregoing
 * risks.
 */

#include "L1Robot.h"
#ifdef _WIN32
#define SLEEP_MS(ms) Sleep(ms)
#else
#define SLEEP_MS(ms) usleep((ms) * 1000)
#endif

/**
 * @brief Entry point for MIT control mode example.
 * 
 * Workflow Overview:
 * 1. Initialize communication with robot controller
 * 2. Get SDK and controller version
 * 3. Check and recover robot arm state (ERROR -> IDLE)
 * 4. Switch ARM1 to MIT mode (STATE_MIT)
 * 5. Send MIT commands at 2ms intervals for 10 seconds total control time
 *    NOTE: If the host computer does not send MIT commands for more than 10ms,
 *          the controller will set current position as target position internally,
 *          stopping the robot at its current location
 * 6. Return to IDLE state
 * 
 * @param[in] argc Number of arguments (not used)
 * @param[in] argv Argument vector (not used)
 * @return int Exit code (returns 0 on normal exit)
 */
int main(int argc, char** argv)
{
    int sdk_version = 0;                  ///< SDK version number
    int controller_version = 0;           ///< Controller firmware version
    unsigned int system_errorcode = 0;    ///< Latest system error code
    FXStateType obj_state = FX_STATE_UNKNOWN;  ///< Current robot state
    double AllTime = 10;                  ///< Total MIT control time (seconds)

    double k[7] = { 5, 5, 5, 3, 2, 2, 2 };   ///< PD proportional gains for 7 joints
    double d[7] = { 0.4, 0.4, 0.4, 0.4, 0.15, 0.15, 0.15 }; ///< PD derivative gains for 7 joints

    double NowPos[7] = { 0 };             ///< Current joint positions
    double vel_cmd[7] = { 0 };            ///< Desired velocity commands
    double tor_cmd[7] = { 0 };            ///< Torque commands

    const ROBOT_SG* sg_ptr = FX_L1_Fbk_GetSG(); ///< State group feedback pointer
    const ROBOT_RT* rt_ptr = FX_L1_Fbk_GetRT(); ///< Real-time feedback pointer

    /* Query SDK version */
    sdk_version = FX_L1_System_GetSDKVersion();
    printf("SDK version is 0x%08x\n", sdk_version);

    /* Establish communication with controller */
    if (FX_L1_System_Link(6, 6, 7, 190, FX_LOG_ALL_FLAG) < 0) {
        printf("Failed to link system\n");
        goto WAIT_EXIT;
    }

    /* Get controller firmware version */
    controller_version = FX_L1_System_GetControllerVersion();
    printf("Controller version is 0x%08x\n", controller_version);

    /* Error handling: Check and recover from ERROR state */
    obj_state = FX_L1_Fbk_CurrentState(FX_OBJ_ARM1);
    if (obj_state == FX_STATE_ERROR) {
        printf("Arm1 is in STATE_ERROR, press any key to reset error\n");
        getchar();
        if (FX_L1_State_ResetError(FX_OBJ_ARM1, 1000, &system_errorcode) != FUNC_RET_SUCCESS) {
            printf("Failed to reset arm1 error, errorcode = 0x%08x\n", system_errorcode);
            goto WAIT_EXIT;
        }
    }
    else if (obj_state != FX_STATE_IDLE) {
        printf("Arm1 is not in STATE_IDLE, press any key to switch to IDLE\n");
        getchar();
        if (FX_L1_State_SwitchToIdle(FX_OBJ_ARM1, 1000) != FUNC_RET_SUCCESS) {
            printf("Failed to switch arm1 to IDLE state\n");
            goto WAIT_EXIT;
        }
    }

    /* Prompt user before entering MIT mode */
    printf("Press any key to switch arm1 to STATE_MIT\n");
    getchar();

    /* Switch to MIT control mode */
    if (FX_L1_State_SwitchToMITMode(FX_OBJ_ARM1, 2000, k, d) != FUNC_RET_SUCCESS) {
        printf("Failed to switch arm1 to STATE_MIT\n");
        goto WAIT_EXIT;
    }

    /* Record initial joint positions from real-time feedback */
    for (int i = 0; i < 7; i++) {
        NowPos[i] = rt_ptr->m_ARMS[1].m_ARM_OUT.m_ARM_FBK_Joint_Pos[i];
    }

    SLEEP_MS(2);  ///< Short delay to ensure stable feedback

    /* MIT control loop: Send commands at 2ms interval for 10 seconds */
    /* The torque command typically uses gravity compensation torque, */
    /* which can be obtained from m_ARM_FBK_Joint_GravityTor */
    while (AllTime > 0) {
        AllTime -= 0.002;  // Decrement by 2ms

        // /* Print gravity compensation torque values */
        // printf("Grav=%f %f %f %f %f %f %f\n", 
        //     rt_ptr->m_ARMS[1].m_ARM_OUT.m_ARM_FBK_Joint_GravityTor[0],
        //     rt_ptr->m_ARMS[1].m_ARM_OUT.m_ARM_FBK_Joint_GravityTor[1], 
        //     rt_ptr->m_ARMS[1].m_ARM_OUT.m_ARM_FBK_Joint_GravityTor[2],
        //     rt_ptr->m_ARMS[1].m_ARM_OUT.m_ARM_FBK_Joint_GravityTor[3], 
        //     rt_ptr->m_ARMS[1].m_ARM_OUT.m_ARM_FBK_Joint_GravityTor[4],
        //     rt_ptr->m_ARMS[1].m_ARM_OUT.m_ARM_FBK_Joint_GravityTor[5], 
        //     rt_ptr->m_ARMS[1].m_ARM_OUT.m_ARM_FBK_Joint_GravityTor[6]);

        /* Set torque command to gravity compensation and update PD gains */
        for (int i = 0; i < 7; i++) {
            tor_cmd[i] = rt_ptr->m_ARMS[1].m_ARM_OUT.m_ARM_FBK_Joint_GravityTor[i];
            k[i] = 1;   // Proportional gain
            d[i] = 0.1; // Derivative gain
        }

        /* Send MIT joint command to controller */
        if (FX_L1_Runtime_SetJointMITCmd(1, FX_OBJ_ARM1, NowPos, vel_cmd, tor_cmd, k, d) != FUNC_RET_SUCCESS) {
            goto WAIT_EXIT;
        }

        Sleep(2);  ///< 2ms delay to maintain control frequency
    }

    /* Return to IDLE state */
    printf("Press any key to switch back to IDLE state\n");
    getchar();

    /* Switch arm back to IDLE state */
    FX_L1_State_SwitchToIdle(FX_OBJ_ARM1, 1000);

WAIT_EXIT:
    printf("Press any key to exit\n");
    getchar();

    return 0;
}