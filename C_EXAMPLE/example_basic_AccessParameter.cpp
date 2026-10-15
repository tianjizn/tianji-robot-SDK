/**
 * @file example_basic_AccessParameter.cpp
 * @brief Example demonstrating how to access and modify basic L1 robot parameters.
 *
 * This example shows how to:
 * - Query the SDK and controller versions
 * - Read string, integer, and floating-point parameters from the robot system
 * - Modify a floating-point parameter and restore its original value
 *
 * @note This example is intended for users who need to understand the basic
 *       parameter read/write interfaces of the L1 robot SDK.
 */

#include "L1Robot.h"

/**
 * @brief Entry point of the example application.
 *
 * The workflow includes:
 * 1. Initialize communication with the robot controller
 * 2. Retrieve SDK and controller versions
 * 3. Read basic robot parameters
 * 4. Modify and verify an encoder error threshold parameter
 * 5. Restore the original parameter value
 *
 * @param[in] argc Argument count (unused)
 * @param[in] argv Argument vector (unused)
 * @return int Exit code (0 on normal exit)
 */
int main(int argc, char** argv)
{
    int sdk_version = 0;                 ///< SDK version number
    int controller_version = 0;          ///< Controller firmware version
    char robot_name[30] = { 0 };         ///< Robot name string
    int bus_freq = 0;                    ///< Communication bus frequency
    float new_enc_error_valve = 0.0f;    ///< Updated encoder error threshold
    float old_enc_error_valve = 0.0f;    ///< Original encoder error threshold

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

    /* Read robot name */
    if (FX_L1_Param_GetString("R.BASIC.Name", robot_name) != FUNC_RET_SUCCESS)
    {
        printf("Failed to get string parameter: R.BASIC.Name\n");
    }

    /* Read bus frequency */
    if (FX_L1_Param_GetInt32("R.BASIC.BusFreq", &bus_freq) != 0)
    {
        printf("Failed to get int parameter: R.BASIC.BusFreq\n");
    }

    /* Read encoder error threshold */
    if (FX_L1_Param_GetFloat("R.A0.BASIC.EncErrorValve", &old_enc_error_valve) != FUNC_RET_SUCCESS)
    {
        printf("Failed to get float parameter: R.A0.BASIC.EncErrorValve\n");
    }

    /* Modify encoder error threshold */
    printf("Press any key to set float parameter R.A0.BASIC.EncErrorValve=2.5\n");
    getchar();
    if (FX_L1_Param_SetFloat("R.A0.BASIC.EncErrorValve", 2.5f) != FUNC_RET_SUCCESS)
    {
        printf("Failed to set float parameter R.A0.BASIC.EncErrorValve\n");
    }

    /* Verify modified parameter */
    printf("Press any key to get float parameter R.A0.BASIC.EncErrorValve\n");
    getchar();
    if (FX_L1_Param_GetFloat("R.A0.BASIC.EncErrorValve", &new_enc_error_valve) != FUNC_RET_SUCCESS)
    {
        printf("Failed to get float parameter: R.A0.BASIC.EncErrorValve\n");
    }

    /* Restore original parameter value */
    printf("Press any key to set float parameter R.A0.BASIC.EncErrorValve=%.4f\n", old_enc_error_valve);
    getchar();
    if (FX_L1_Param_SetFloat("R.A0.BASIC.EncErrorValve", old_enc_error_valve) != FUNC_RET_SUCCESS)
    {
        printf("Failed to set float parameter R.A0.BASIC.EncErrorValve\n");
    }

WAIT_EXIT:
    printf("Press any key to exit\n");
    getchar();
    return 0;
}