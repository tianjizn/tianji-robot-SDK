/**
 * @file example_basic_LinkSystem.cpp
 * @brief Example demonstrating how to establish and terminate communication
 *        with the L1 robot controller system.
 *
 * This example shows how to:
 * - Query the SDK version
 * - Establish a communication link with the robot controller
 * - Check the communication link state
 * - Retrieve the controller firmware version
 * - Safely disconnect from the controller
 *
 * This is the most fundamental example and should be executed before
 * any other robot control examples.
 */

#include "L1Robot.h"
#include <thread>

#ifdef _WIN32
#define SLEEP_MS(ms) Sleep(ms)
#else
#define SLEEP_MS(ms) usleep((ms) * 1000)
#endif

int main(int argc, char** argv)
{
    int sdk_version = 0;                  ///< SDK version number
    int controller_version = 0;           ///< Controller firmware version
    unsigned int system_errorcode = 0;    ///< Last system error code
    FXStateType obj_state = FX_STATE_UNKNOWN;

    double vel_ratio = 10.0;              ///< Velocity ratio for position mode
    double acc_ratio = 10.0;              ///< Acceleration ratio for position mode
    FX_DOUBLE point[7000] = { 0 };

    /* Get SDK version */
    sdk_version = FX_L1_System_GetSDKVersion();
    printf("SDK version is 0x%08x\n", sdk_version);

    /* Establish communication with the controller */
    if (FX_L1_System_Link(6, 6, 7, 190, FX_LOG_ALL_FLAG) < 0)
    {
        printf("Failed to link system\n");
        goto WAIT_EXIT;
    }

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

    /* Switch to position control mode */
    printf("Arm0 is in STATE_IDLE state now, press any key to transfer to STATE_POSITION state\n");
    getchar();
    if (FX_L1_State_SwitchToPositionMode(FX_OBJ_ARM0, 2000, vel_ratio, acc_ratio) != FUNC_RET_SUCCESS)
    {
        printf("Failed to transfer arm0 to STATE_POSITION state\n");
        goto WAIT_EXIT;
    }

    printf("Press any key to set traj...\n");
    getchar();
    if (FX_L1_Config_SetTraj(FX_OBJ_ARM0, 1000, point) != FUNC_RET_SUCCESS)
    {
        printf("Failed to set traj\n");
    }
    else
    {
        printf("Set traj success\n");
    }

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
