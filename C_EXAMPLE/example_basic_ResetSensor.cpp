/**
 * @file example_basic_ResetSensor.cpp
 * @brief Example demonstrating how to reset joint torque sensor offsets
 *        for the L1 robot arm.
 *
 * This example shows how to:
 * - Query SDK and controller versions
 * - Recover the robot arm from ERROR state
 * - Move the arm to the zero position in POSITION mode
 * - Read current joint torque sensor feedback
 * - Reset torque sensor offsets to zero
 *
 * Resetting torque sensor offsets is essential for accurate torque control
 * and collision detection.
 *
 * @warning Before running this example, ensure that **no tool is mounted**
 *          on Arm0’s end effector. Otherwise, torque control behavior may
 *          become incorrect.
 */

#include "L1Robot.h"

#ifdef _WIN32
#define SLEEP_MS(ms) Sleep(ms)
#else
#define SLEEP_MS(ms) usleep((ms) * 1000)
#endif

/**
 * @brief Read current joint torque sensor feedback for ARM0.
 *
 * This function retrieves real-time torque sensor data from the robot
 * feedback interface.
 *
 * @param[out] sensor_fbk Array of 7 torque sensor values (Nm)
 */
void ReadArm0SensorFbk(double sensor_fbk[7])
{
    const ROBOT_RT* rt_ptr = FX_L1_Fbk_GetRT();
    if (rt_ptr == NULL)
    {
        return;
    }

    for (int i = 0; i < 7; i++)
    {
        sensor_fbk[i] =
            rt_ptr->m_ARMS[0].m_ARM_OUT.m_ARM_FBK_Joint_SensorTor[i];
    }
}

/**
 * @brief Entry point of the torque sensor reset example.
 *
 * Workflow overview:
 * 1. Initialize communication with the robot controller
 * 2. Retrieve SDK and controller versions
 * 3. Check and recover the arm state (ERROR → IDLE)
 * 4. Move ARM0 to the zero position using POSITION mode
 * 5. Read current torque sensor feedback
 * 6. Reset torque sensor offsets to zero
 * 7. Read sensor feedback again to verify the result
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

    double zero_position[7] = { 0 };          ///< Zero joint position (degrees)
    double sensor_fbk[7] = { 0 };            ///< Torque sensor feedback values

    /* Get SDK version */
    sdk_version = FX_L1_System_GetSDKVersion();
    printf("SDK version is 0x%08x\n", sdk_version);

    printf("----------------------------------------- IMPORTANT ------------------------------------------\n");
    printf("Before running this example, please ensure that no tool is mounted on Arm0's end effector.\n");
    printf("Otherwise, Arm0 will not operate correctly in torque control mode!\n");
    printf("-------------------------------------------------------------------------------------------------\n");

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

    /* Switch to POSITION mode */
    printf("Press any key to transfer arm0 to FX_STATE_POSITION state\n");
    getchar();
    if (FX_L1_State_SwitchToPositionMode(FX_OBJ_ARM0, 2000, 10, 10) != FUNC_RET_SUCCESS)
    {
        printf("Transfer arm0 to FX_STATE_POSITION state failed\n");
        goto WAIT_EXIT;
    }

    /* Move arm to zero position */
    printf("Press any key to let arm0 move to zero position\n");
    getchar();
    if (FX_L1_Runtime_SetJointPosCmd(1, FX_OBJ_ARM0, zero_position) != FUNC_RET_SUCCESS)
    {
        printf("Failed to set arm0's target position\n");
        goto WAIT_EXIT;
    }

    /* Return to IDLE state */
    printf("Please wait until arm0 reaches zero position, then press any key to transfer arm0 to FX_STATE_IDLE state\n");
    getchar();
    if (FX_L1_State_SwitchToIdle(FX_OBJ_ARM0, 1000) != FUNC_RET_SUCCESS)
    {
        printf("Transfer arm0 to FX_STATE_IDLE state failed\n");
    }

    /* Read torque sensor feedback before reset */
    ReadArm0SensorFbk(sensor_fbk);
    printf("Current arm0's sensor feedback: "
           "{%.4lf,%.4lf,%.4lf,%.4lf,%.4lf,%.4lf,%.4lf}\n",
           sensor_fbk[0], sensor_fbk[1], sensor_fbk[2],
           sensor_fbk[3], sensor_fbk[4], sensor_fbk[5],
           sensor_fbk[6]);

    /* Reset torque sensor offsets */
    printf("Press any key to clear sensor offset to zero\n");
    getchar();
    if (FX_L1_Config_ResetSensorOffset(FX_OBJ_ARM0) != FUNC_RET_SUCCESS)
    {
        printf("Clear arm0's sensor offset to zero failed\n");
        goto WAIT_EXIT;
    }

    SLEEP_MS(10);

    /* Read torque sensor feedback after reset */
    ReadArm0SensorFbk(sensor_fbk);
    printf("Clear sensor offset success, current arm0's sensor feedback: "
           "{%.4lf,%.4lf,%.4lf,%.4lf,%.4lf,%.4lf,%.4lf}\n",
           sensor_fbk[0], sensor_fbk[1], sensor_fbk[2],
           sensor_fbk[3], sensor_fbk[4], sensor_fbk[5],
           sensor_fbk[6]);

WAIT_EXIT:
    printf("Press any key to exit\n");
    getchar();
    return 0;
}
