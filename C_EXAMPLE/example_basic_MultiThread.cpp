/**
 * @file example_basic_MultiThread.cpp
 * @brief Example demonstrating basic multi-threaded joint position control
 *        for dual-arm L1 robot systems.
 *
 * This example shows how to:
 * - Establish communication with the L1 robot controller
 * - Switch robot arms into position control mode
 * - Control ARM0 and ARM1 simultaneously using separate threads
 * - Perform smooth joint-space interpolation
 * - Periodically monitor servo error states
 * - Safely stop threads and return to IDLE state
 *
 * Three threads are created:
 * - Thread 0: Sends joint position commands to ARM0
 * - Thread 1: Sends joint position commands to ARM1
 * - Thread 2: Monitors servo error codes
 *
 * This example is intended for users who need real-time multi-axis control
 * using multiple threads.
 */
#include "L1Robot.h"
#include <thread>

#ifdef _WIN32
#define SLEEP_MS(ms) Sleep(ms)
#else
#define SLEEP_MS(ms) usleep((ms) * 1000)
#endif

int exit_thread = 0; ///< Global flag to request all threads to exit
int run_thread = 0; ///< Global flag to start or stop motion threads
double arm0_target[7] = { 94.0, -87.0, -89.0, -94.0, 2.469, 0.0, -7.428 }; ///< Target joint positions for ARM0
double arm1_target[7] = { 78.0, 87.0, -96.0, -104.0, 0.723, 0.0, 0.0 }; ///< Target joint positions for ARM1
double zero_target[7] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 }; ///< Zero reference joint positions
int error_count[3] = { 0 }; ///< Error counters for command and state queries
int move_cycle_sec = 10; ///< Motion cycle duration in seconds
/**
 * @brief Perform smooth cubic interpolation between two joint positions.
 *
 * This function generates a smooth motion profile suitable for real-time control.
 *
 * @param[in]  start   Starting joint positions (7 joints)
 * @param[in]  target  Target joint positions (7 joints)
 * @param[in]  alpha   Normalized interpolation factor [0.0, 1.0]
 * @param[out] out     Output interpolated joint positions
 */
void InterpolateJoints(const double start[7], const double target[7], double alpha, double out[7])
{
    if (alpha < 0.0)  alpha = 0.0;
    if (alpha > 1.0)  alpha = 1.0;
    double smooth_alpha = alpha * alpha * alpha * (10.0 + alpha * (-15.0 + 6.0 * alpha));
    for (int i = 0; i < 7; ++i)
    {
        out[i] = start[i] + (target[i] - start[i]) * smooth_alpha;
    }
}
/**
 * @brief Motion control thread for ARM0.
 *
 * This thread sends interpolated joint position commands to ARM0
 * at approximately 50 Hz. The motion alternates between zero position
 * and the predefined target position.
 *
 * The thread exits when @ref exit_thread is set to 1.
 */
void thread0()
{
    double cmd_pos[7] = { 0 };
    double alpha = 0;
    auto period = std::chrono::milliseconds(20);
    auto segment_begin = std::chrono::steady_clock::now();
    auto next_tick = segment_begin;

    while (exit_thread == 0)
    {
        if (run_thread == 1)
        {
            auto now = std::chrono::steady_clock::now();
            double elapsed = std::chrono::duration<double>(now - segment_begin).count();
            int elapsed_int = elapsed;
            if((elapsed_int / move_cycle_sec) % 2 == 0) // alpha 0--->1
            {
                alpha = ((elapsed_int % (move_cycle_sec * 2)) + (elapsed - elapsed_int)) / move_cycle_sec;
            }
            else // alpha 1--->0
            {
                alpha = 1.0 - ((elapsed_int % (move_cycle_sec * 2)) - move_cycle_sec + (elapsed - elapsed_int)) / move_cycle_sec;
            }
            InterpolateJoints(zero_target, arm0_target, alpha, cmd_pos);
            
            if (FX_L1_Runtime_SetJointPosCmd(1, FX_OBJ_ARM0, cmd_pos) != FUNC_RET_SUCCESS)
            {
                error_count[0]++;
            }
        }
        next_tick += period;
        std::this_thread::sleep_until(next_tick);
    }
}
/**
 * @brief Motion control thread for ARM1.
 *
 * Similar to thread0(), this thread controls ARM1 independently
 * by sending periodic joint position commands.
 */
void thread1()
{
    double cmd_pos[7] = { 0 };
    double alpha = 0;
    auto period = std::chrono::milliseconds(20);
    auto segment_begin = std::chrono::steady_clock::now();
    auto next_tick = segment_begin;

    while (exit_thread == 0)
    {
        if (run_thread == 1)
        {
            auto now = std::chrono::steady_clock::now();
            double elapsed = std::chrono::duration<double>(now - segment_begin).count();
            int elapsed_int = elapsed;
            if ((elapsed_int / move_cycle_sec) % 2 == 0) // alpha 0--->1
            {
                alpha = ((elapsed_int % (move_cycle_sec * 2)) + (elapsed - elapsed_int)) / move_cycle_sec;
            }
            else // alpha 1--->0
            {
                alpha = 1.0 - ((elapsed_int % (move_cycle_sec * 2)) - move_cycle_sec + (elapsed - elapsed_int)) / move_cycle_sec;
            }
            InterpolateJoints(zero_target, arm1_target, alpha, cmd_pos);

            if (FX_L1_Runtime_SetJointPosCmd(2, FX_OBJ_ARM1, cmd_pos) != FUNC_RET_SUCCESS)
            {
                error_count[1]++;
            }
        }
        next_tick += period;
        std::this_thread::sleep_until(next_tick);
    }
}
/**
 * @brief Background thread for monitoring servo error states.
 *
 * This thread periodically queries servo error codes for ARM0 and ARM1.
 * It increments internal error counters when query failures occur.
 */
void thread2()
{
    unsigned int error_code[7];
    while (exit_thread == 0)
    {
        if (run_thread == 1)
        {
            if (FX_L1_State_GetServoErrorCode(FX_OBJ_ARM0, error_code) != FUNC_RET_SUCCESS)
            {
                error_count[2]++;
            }
            if (FX_L1_State_GetServoErrorCode(FX_OBJ_ARM1, error_code) != FUNC_RET_SUCCESS)
            {
                error_count[2]++;
            }
        }
        SLEEP_MS(500);
    }
}
/**
 * @brief Entry point of the multi-thread control example.
 *
 * Workflow overview:
 * 1. Retrieve SDK and controller versions
 * 2. Establish communication with the L1 controller
 * 3. Recover ARM0 and ARM1 from ERROR or non-IDLE states
 * 4. Switch both arms into POSITION control mode
 * 5. Start three control threads
 * 6. Execute cyclic motion with smooth interpolation
 * 7. Stop threads and return arms to IDLE state
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

    double vel_ratio = 10.0;              ///< Velocity ratio for position mode
    double acc_ratio = 10.0;              ///< Acceleration ratio for position mode

    std::thread arm0_thread(thread0);
    std::thread arm1_thread(thread1);
    std::thread arm2_thread(thread2);

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

    obj_state = FX_L1_Fbk_CurrentState(FX_OBJ_ARM1);
    if (obj_state == FX_STATE_ERROR)
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
    else if (obj_state != FX_STATE_IDLE)
    {
        printf("Arm1 is not in STATE_IDLE state now, press any key to transfer to STATE_IDLE state\n");
        getchar();
        if (FX_L1_State_SwitchToIdle(FX_OBJ_ARM1, 1000) != FUNC_RET_SUCCESS)
        {
            printf("Failed to transfer arm1 to STATE_IDLE state\n");
            goto WAIT_EXIT;
        }
    }

    /* Switch to position control mode */
    printf("Arm0 and Arm1 are in STATE_IDLE state now, press any key to transfer to STATE_POSITION state\n");
    getchar();
    if (FX_L1_State_SwitchToPositionMode(FX_OBJ_ARM0, 2000, vel_ratio, acc_ratio) != FUNC_RET_SUCCESS)
    {
        printf("Failed to transfer arm0 to STATE_POSITION state\n");
        goto WAIT_EXIT;
    }
    if (FX_L1_State_SwitchToPositionMode(FX_OBJ_ARM1, 2000, vel_ratio, acc_ratio) != FUNC_RET_SUCCESS)
    {
        printf("Failed to transfer arm1 to STATE_POSITION state\n");
        goto WAIT_EXIT;
    }

    /* Move to target position */
    printf("Arm0 and Arm1 are in STATE_POSITION state now\n");
    printf("Press any key to let arm0 and arm1 move to zero target position "
           "{%.4lf,%.4lf,%.4lf,%.4lf,%.4lf,%.4lf,%.4lf}\n",
        zero_target[0], zero_target[1], zero_target[2], zero_target[3], zero_target[4], zero_target[5], zero_target[6]);
    getchar();

    printf("Press any key to run two threads to send command positin to arm0 and arm1 seperately...\n");
    getchar();
    run_thread = 1;

    printf("Press any key to stop threads\n");
    getchar();
    run_thread = 0;
    printf("Threads stopped\n");
    printf("error_count = %d %d %d\n", error_count[0], error_count[1], error_count[2]);
    getchar();

    /* Return to IDLE state */
    printf("Press any key to transfer to STATE_IDLE state\n");
    getchar();
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
    printf("Arm0 and Arm1 are in STATE_IDLE state now\n");

WAIT_EXIT:
    printf("Press any key to exit\n");
    getchar();
    exit_thread = 1;
    arm0_thread.join();
    arm1_thread.join();
    arm2_thread.join();
    return 0;
}