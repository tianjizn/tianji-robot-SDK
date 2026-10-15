/**
 * @file example_basic_ResetEncoder.cpp
 * @brief Example demonstrating how to reset motor and external encoder offsets
 *        for the L1 robot arm.
 *
 * This example shows how to:
 * - Query SDK and controller versions
 * - Recover the robot arm from ERROR state
 * - Switch the arm to IDLE state
 * - Read current motor and external encoder feedback values
 * - Reset encoder offsets to zero
 *
 * @warning Before running this example, ensure that **Arm0 is at the mechanical
 *          zero position**. Otherwise, the arm may behave incorrectly in all
 *          control modes.
 *
 * @warning For each controlled object, the encoder reset operation can only be
 *          performed **once per power cycle**. Performing it multiple times may
 *          result in abnormal encoder position data after the next power-up.
 *
 * @warning After resetting the encoder, **do not execute any motion commands**.
 *          Power-cycle the system to apply the changes correctly.
 */

#include "L1Robot.h"

#ifdef _WIN32
#define SLEEP_MS(ms) Sleep(ms)
#else
#define SLEEP_MS(ms) usleep((ms) * 1000)
#endif

/**
 * @brief Read current motor and external encoder feedback values for ARM0.
 *
 * This function retrieves real-time joint position feedback from both
 * motor encoders and external encoders.
 *
 * @param[out] motor_encoder_fbk      Array of 7 motor encoder positions (degrees)
 * @param[out] external_encoder_fbk   Array of 7 external encoder positions (degrees)
 */
void ReadArm0EncoderFbk(double motor_encoder_fbk[7], double external_encoder_fbk[7])
{
    const ROBOT_RT* rt_ptr = FX_L1_Fbk_GetRT();
    const ROBOT_SG* sg_ptr = FX_L1_Fbk_GetSG();
    if (rt_ptr == NULL || sg_ptr == NULL)
    {
        return;
    }

    for (int i = 0; i < 7; i++)
    {
        motor_encoder_fbk[i] =
            rt_ptr->m_ARMS[0].m_ARM_OUT.m_ARM_FBK_Joint_Pos[i];
        external_encoder_fbk[i] =
            sg_ptr->m_ARMS[0].m_ARM_GET.m_ARM_FBK_Joint_ExtPos[i];
    }
}

/**
 * @brief Entry point of the encoder reset example.
 *
 * Workflow overview:
 * 1. Initialize communication with the robot controller
 * 2. Retrieve SDK and controller versions
 * 3. Check and recover the arm state (ERROR → IDLE)
 * 4. Read current motor and external encoder feedback
 * 5. Reset encoder offsets to zero
 * 6. Read encoder feedback again to verify the result
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
    unsigned int system_errorcode = 0;        ///< Last system error code

    double motor_encoder_fbk[7] = { 0 };      ///< Motor encoder feedback values
    double external_encoder_fbk[7] = { 0 };  ///< External encoder feedback values

    /* Get SDK version */
    sdk_version = FX_L1_System_GetSDKVersion();
    printf("SDK version is 0x%08x\n", sdk_version);

    printf("--------------------------------- IMPORTANT ----------------------------------------\n");
    printf("Before running this sample, please ensure Arm0 is at the mechanical zero position.\n");
    printf("Otherwise, Arm0 will not operate correctly in all control modes!\n");
    printf("For each controlled object, the encoder reset operation can only be performed once\n");
    printf("per power cycle. Performing this operation multiple times will result in abnormal\n");
    printf("encoder position data upon the next power-up.\n");
    printf("------------------------------------------------------------------------------------\n");

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
        printf("Reset sensor offset must be run in FX_STATE_IDLE state, try to reset error\n");
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

    /* Read encoder feedback before reset */
    ReadArm0EncoderFbk(motor_encoder_fbk, external_encoder_fbk);
    printf("Current arm0's motor encoder feedback: "
           "{%.4lf,%.4lf,%.4lf,%.4lf,%.4lf,%.4lf,%.4lf}\n",
           motor_encoder_fbk[0], motor_encoder_fbk[1], motor_encoder_fbk[2],
           motor_encoder_fbk[3], motor_encoder_fbk[4], motor_encoder_fbk[5],
           motor_encoder_fbk[6]);

    printf("Current arm0's external encoder feedback: "
           "{%.4lf,%.4lf,%.4lf,%.4lf,%.4lf,%.4lf,%.4lf}\n",
           external_encoder_fbk[0], external_encoder_fbk[1], external_encoder_fbk[2],
           external_encoder_fbk[3], external_encoder_fbk[4], external_encoder_fbk[5],
           external_encoder_fbk[6]);

    /* Reset encoder offsets */
    printf("Press any key to reset motor encoder and external encoder feedbacks to zero\n");
    getchar();

    if (FX_L1_Config_ResetEncOffset(FX_OBJ_ARM0, 0xFF) != FUNC_RET_SUCCESS)
    {
        printf("Reset arm0's motor encoder and external encoder feedbacks to zero failed\n");
        goto WAIT_EXIT;
    }

    SLEEP_MS(100);
    printf("Reset arm0's motor encoder and external encoder feedbacks to zero success\n");

    /* Read encoder feedback after reset */
    ReadArm0EncoderFbk(motor_encoder_fbk, external_encoder_fbk);
    printf("Current arm0's motor encoder feedback: "
           "{%.4lf,%.4lf,%.4lf,%.4lf,%.4lf,%.4lf,%.4lf}\n",
           motor_encoder_fbk[0], motor_encoder_fbk[1], motor_encoder_fbk[2],
           motor_encoder_fbk[3], motor_encoder_fbk[4], motor_encoder_fbk[5],
           motor_encoder_fbk[6]);

    printf("Current arm0's external encoder feedback: "
           "{%.4lf,%.4lf,%.4lf,%.4lf,%.4lf,%.4lf,%.4lf}\n",
           external_encoder_fbk[0], external_encoder_fbk[1], external_encoder_fbk[2],
           external_encoder_fbk[3], external_encoder_fbk[4], external_encoder_fbk[5],
           external_encoder_fbk[6]);

    printf("Upon successful encoder reset, the motor encoder position feedback will remain\n");
    printf("unchanged until after a reboot, whereas the external encoder position feedback\n");
    printf("updates immediately.\n");
    printf("Do not execute any motion control commands after resetting the encoder;\n");
    printf("please power cycle the system.\n");

WAIT_EXIT:
    printf("Press any key to exit\n");
    getchar();
    return 0;
}
