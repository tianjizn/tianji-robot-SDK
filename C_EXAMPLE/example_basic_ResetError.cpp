/**
 * @file example_basic_ResetError.cpp
 * @brief Example demonstrating how to detect and reset errors on the L1 robot arm.
 *
 * This example shows how to:
 * - Query SDK and controller versions
 * - Detect whether the robot arm is in an ERROR state
 * - Optionally trigger an emergency stop to enter the ERROR state
 * - Read system-level and servo-level error codes
 * - Reset the arm error and return to IDLE state
 *
 * Error handling is a critical part of safe robot operation and should be
 * well understood before deploying user applications.
 */

#include "L1Robot.h"

#ifdef _WIN32
#define SLEEP_MS(ms) Sleep(ms)
#else
#define SLEEP_MS(ms) usleep((ms) * 1000)
#endif

/**
 * @brief Entry point of the error reset example.
 *
 * Workflow overview:
 * 1. Initialize communication with the robot controller
 * 2. Retrieve SDK and controller versions
 * 3. Check the current state of ARM1
 * 4. If not already in ERROR state, trigger an emergency stop
 * 5. Read system and servo error codes
 * 6. Attempt to reset the error and return ARM1 to IDLE state
 *
 * @param[in] argc Argument count (unused)
 * @param[in] argv Argument vector (unused)
 * @return int Exit code (0 on normal exit)
 */
int main(int argc, char** argv)
{
    int sdk_version = 0;                      ///< SDK version number
    int controller_version = 0;               ///< Controller firmware version
    FXStateType obj_state = FX_STATE_UNKNOWN; ///< Current arm state

    const ROBOT_RT* rt_ptr = NULL;            ///< Real-time robot feedback data
    unsigned int system_errorcode = 0;        ///< System-level error code
    unsigned int servo_errorcode[7] = { 0 };  ///< Per-joint servo error codes

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
    obj_state = FX_L1_Fbk_CurrentState(FX_OBJ_ARM1);
    if (obj_state == FX_STATE_UNKNOWN)
    {
        printf("Arm1 is in unknown state\n");
        goto WAIT_EXIT;
    }
    else if (obj_state == FX_STATE_ERROR)
    {
        printf("Arm1 is in error state now\n");
    }
    else
    {
        /* Trigger an emergency stop to force ARM1 into ERROR state */
        printf("Arm1 is not in error, press any key to trigger an emergency error\n");
        getchar();

        if (FX_L1_Runtime_EmergencyStop(1, FX_OBJ_ARM1_FLAG) != FX_OBJ_ARM1_FLAG)
        {
            printf("Failed to trigger an emergency error for arm1\n");
            goto WAIT_EXIT;
        }

        SLEEP_MS(100);
        if (FX_L1_Fbk_CurrentState(FX_OBJ_ARM1) != FX_STATE_ERROR)
        {
            printf("Arm1 is not in error state!\n");
            goto WAIT_EXIT;
        }
    }

    /* Read real-time feedback data */
    rt_ptr = FX_L1_Fbk_GetRT();
    if (rt_ptr == NULL)
    {
        printf("Failed to get ROBOT_RT data pointer\n");
        goto WAIT_EXIT;
    }

    /* Read and print system error code */
    system_errorcode = rt_ptr->m_ARMS[1].m_ARM_State.m_ERRCode;
    printf("Arm1's errorcode is %d\n", system_errorcode);

    /* If the error is servo-related, read detailed servo error codes */
    if (system_errorcode == ERR_Servo)
    {
        if (FX_L1_State_GetServoErrorCode(FX_OBJ_ARM1, servo_errorcode) != FUNC_RET_SUCCESS)
        {
            printf("Failed to get servo errorcode for arm1\n");
            goto WAIT_EXIT;
        }
        else
        {
            printf("Arm1 is showing a servo error, servo errorcode is "
                   "{0x%08x,0x%08x,0x%08x,0x%08x,0x%08x,0x%08x,0x%08x}\n",
                   servo_errorcode[0], servo_errorcode[1], servo_errorcode[2],
                   servo_errorcode[3], servo_errorcode[4], servo_errorcode[5],
                   servo_errorcode[6]);
        }
    }

    /* Attempt to reset the error */
    printf("Press any key to reset error for arm1\n");
    getchar();

    if (FX_L1_State_ResetError(FX_OBJ_ARM1, 1000, &system_errorcode) == FUNC_RET_SUCCESS)
    {
        printf("Reset arm1 error success, arm1 is now in STATE_IDLE state\n");
    }
    else
    {
        printf("Failed to reset arm1 error. Arm1's errorcode is %d\n", system_errorcode);

        /* Read servo error codes again if reset failed due to servo error */
        if (system_errorcode == ERR_Servo)
        {
            if (FX_L1_State_GetServoErrorCode(FX_OBJ_ARM1, servo_errorcode) != FUNC_RET_SUCCESS)
            {
                printf("Failed to get servo errorcode for arm1\n");
                goto WAIT_EXIT;
            }
            else
            {
                printf("Arm1 is showing a servo error, servo errorcode is "
                       "{0x%08x,0x%08x,0x%08x,0x%08x,0x%08x,0x%08x,0x%08x}\n",
                       servo_errorcode[0], servo_errorcode[1], servo_errorcode[2],
                       servo_errorcode[3], servo_errorcode[4], servo_errorcode[5],
                       servo_errorcode[6]);
            }
        }
    }

WAIT_EXIT:
    printf("Press any key to exit\n");
    getchar();
    return 0;
}