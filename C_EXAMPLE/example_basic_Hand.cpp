/**
 * @file example_basic_Hand.cpp
 * @brief Basic example demonstrating how to control the left hand of L1 robot.
 *
 * This example shows:
 * - SDK and controller version query
 * - Hand error handling and recovery
 * - Sending predefined hand gestures
 */

#include "L1Robot.h"

#ifdef _WIN32
#define SLEEP_MS(ms) Sleep(ms)
#else
#define SLEEP_MS(ms) usleep((ms) * 1000)
#endif

 /**
  * @defgroup example_hand_control Hand Control Example
  * @{
  */

  /**
   * @brief Entry point of the basic hand control example.
   *
   * The program initializes communication with the robot system,
   * checks the left hand status, recovers from errors if needed,
   * and executes several predefined hand gestures.
   *
   * @param[in] argc Argument count (unused)
   * @param[in] argv Argument vector (unused)
   * @return int Exit code (0 on success)
   */
int main(int argc, char** argv)
{
    int sdk_version = 0;                  ///< SDK version number
    int controller_version = 0;           ///< Controller firmware version

    const ROBOT_RT* rt_ptr = FX_L1_Fbk_GetRT();

    int open_palm[24] = { 0 };            ///< Fully open hand pose
    int gesture1[24] = { 0 };             ///< First demo hand pose
    int gesture2[24] = { 0 };             ///< Second demo hand pose
    int func_ret = 0;                     ///< Function return value

    /* Initialize demo gestures */
    for (int i = 0; i < 24; i++)
    {
        gesture1[i] = 30;
        gesture2[i] = 60;
    }

    /* Query SDK version */
    sdk_version = FX_L1_System_GetSDKVersion();
    printf("SDK version is 0x%08x\n", sdk_version);

    /* Establish communication with robot system */
    if (FX_L1_System_Link(6, 6, 7, 190, FX_LOG_ALL_FLAG) < 0)
    {
        printf("Failed to link system\n");
        goto WAIT_EXIT;
    }

    /* Query controller firmware version */
    controller_version = FX_L1_System_GetControllerVersion();
    printf("Controller version is 0x%08x\n", controller_version);

    /**
     * @brief Check left hand error state
     */
    if (rt_ptr->m_HANDS[0].m_HAND_OUT.m_HAND_FBK_State == FX_HAND_STATE_ERROR)
    {
        printf("Left hand has error, press any key to reset error and enable hand\n");
        getchar();

        /* Reset left hand error */
        if (FX_L1_Runtime_SetHandAction(1, FX_HAND_LEFT, FX_HAND_ACTION_RESET) != FUNC_RET_SUCCESS)
        {
            printf("Failed to reset left hand error\n");
            goto WAIT_EXIT;
        }

        SLEEP_MS(100);

        /* Enable left hand */
        if (FX_L1_Runtime_SetHandAction(1, FX_HAND_LEFT, FX_HAND_ACTION_ENABLE) != FUNC_RET_SUCCESS)
        {
            printf("Failed to enable left hand\n");
            goto WAIT_EXIT;
        }

        SLEEP_MS(100);

        /* Re-check hand state */
        if (rt_ptr->m_HANDS[0].m_HAND_OUT.m_HAND_FBK_State == FX_HAND_STATE_ERROR)
        {
            printf("Left hand is still in error state!");
            goto WAIT_EXIT;
        }
    }

    printf("Left hand is ready to work, press any key to make a gesture\n");
    getchar();

    /* Send first gesture */
    if (FX_L1_Runtime_SetHandPos(1, FX_HAND_LEFT, gesture1) != FUNC_RET_SUCCESS)
    {
        printf("Failed to make a gesture\n");
        goto WAIT_EXIT;
    }

    printf("Press any key to make another gesture\n");
    getchar();

    /* Send second gesture */
    if (FX_L1_Runtime_SetHandPos(1, FX_HAND_LEFT, gesture2) != FUNC_RET_SUCCESS)
    {
        printf("Failed to make a gesture\n");
        goto WAIT_EXIT;
    }

    printf("Press any key to make an open palm\n");
    getchar();

    /* Open palm gesture */
    if (FX_L1_Runtime_SetHandPos(1, FX_HAND_LEFT, open_palm) != FUNC_RET_SUCCESS)
    {
        printf("Failed to make an open palm\n");
        goto WAIT_EXIT;
    }

WAIT_EXIT:
    printf("Press any key to exit\n");
    getchar();
    return 0;
}

/**
 * @}
 */