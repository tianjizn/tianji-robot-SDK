/**
 * @file example_basic_Brake.cpp
 * @brief Example demonstrating how to control the brakes of L1 robot arms.
 *
 * This example shows how to:
 * - Query SDK and controller versions
 * - Check and recover the robot arm state (IDLE / ERROR)
 * - Unlock and lock all brakes of a specified robot arm
 *
 * @warning Releasing the brakes may cause the arm to drop due to gravity.
 *          Ensure the arm is held securely before unlocking the brakes.
 */

#include "L1Robot.h"

/**
 * @brief Entry point of the brake control example.
 *
 * Workflow overview:
 * 1. Initialize communication with the robot controller
 * 2. Retrieve SDK and controller versions
 * 3. Verify the arm state and recover to IDLE if necessary
 * 4. Unlock all brakes of ARM0
 * 5. Lock all brakes of ARM0
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

    /*
     * Check current arm state.
     * Brake operations are only allowed in FX_STATE_IDLE.
     */
    obj_state = FX_L1_Fbk_CurrentState(FX_OBJ_ARM0);
    if (obj_state == FX_STATE_ERROR)
    {
        printf("Brake lock/unlock must be run in FX_STATE_IDLE state, try to reset error\n");
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
        if (FX_L1_State_SwitchToIdle(FX_OBJ_ARM0, 1000) != FUNC_RET_SUCCESS)
        {
            printf("Failed to transfer arm0 to STATE_IDLE state\n");
            goto WAIT_EXIT;
        }
    }

    /* Unlock all brakes of ARM0 */
    printf("Press any key to unlock all brakes of arm0, "
           "please hold the arm carefully or it will drop!\n");
    getchar();
    if (FX_L1_Config_SetBrakeUnlock(FX_OBJ_ARM0, 0xFF) != FUNC_RET_SUCCESS)
    {
        printf("Unlock all brakes of arm0 failed\n");
    }

    /* Lock all brakes of ARM0 */
    printf("Press any key to lock all brakes of arm0\n");
    getchar();
    if (FX_L1_Config_SetBrakeLock(FX_OBJ_ARM0, 0xFF) != FUNC_RET_SUCCESS)
    {
        printf("Lock all brakes of arm0 failed\n");
    }

WAIT_EXIT:
    printf("Press any key to exit\n");
    getchar();
    return 0;
}