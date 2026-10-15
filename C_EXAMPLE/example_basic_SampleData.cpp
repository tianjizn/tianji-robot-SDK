/**
 * @file example_basic_SampleData.cpp
 * @brief Example demonstrating how to configure and sample user-defined
 *        feedback data from the FX L1 robot controller.
 *
 * This example shows:
 * - Establishing communication with the L1 controller
 * - Configuring user-defined feedback data sets
 * - Sampling custom feedback data at runtime
 * - Accessing sampled data from a background thread
 *
 * A background thread is created to periodically retrieve user data
 * and print selected fields at 10 ms intervals.
 *
 * User-defined sample data includes:
 * - Robot real-time frame serial number
 * - Servo group frame serial number
 *
 * @note This example is intended for users who need customized feedback
 *       sampling beyond standard joint feedback.
 */
#include "L1Robot.h"
#include <thread>

#ifdef _WIN32
#define SLEEP_MS(ms) Sleep(ms)
#else
#define SLEEP_MS(ms) usleep((ms) * 1000)
#endif

int exit_thread = 0; ///< Global flag to request all threads to exit
int run_thread = 0; ///< Global flag to start or stop data sampling
/**
 * @brief User-defined structure for sampled feedback data.
 *
 * This structure must match the registered user data set
 * defined in the FX L1 feedback system.
 */
typedef struct
{
    int rt_serial; ///< RT frame serial number
    int sg_serial; ///< SG frame serial number
}UserSampleData;
/**
 * @brief Background thread for sampling user-defined feedback data.
 *
 * This thread runs at approximately 100 Hz and retrieves user data
 * using FX_L1_Fbk_GetUserData(). Selected fields are printed
 * every 100 cycles (~1 second).
 *
 * The thread exits when @ref exit_thread is set to 1.
 */
void thread0()
{
    int run_count = 0;
    UserSampleData user_data;
    while (exit_thread == 0)
    {
        if (run_thread == 1)
        {
            FX_L1_Fbk_GetUserData((void*)&user_data);
            run_count++;
            if (run_count % 100 == 0)
            {
                printf("UserSampleData: rt_serial = %d, sg_serial = %d\n", user_data.rt_serial, user_data.sg_serial);
            }
        }
        SLEEP_MS(10);
    }
}
/**
 * @brief Entry point of the user-defined sample data example.
 *
 * Workflow overview:
 * - Retrieve SDK and controller versions
 * - Establish communication with the L1 controller
 * - Reset and register user-defined feedback data sets
 * - Validate the size of the user data structure
 * - Start a background sampling thread
 * - Enable and disable sampling dynamically
 * - Safely stop the thread and release resources
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

    std::thread sample_thread(thread0);

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

    printf("Press any key to config user sample data\n");
    getchar();

    FX_L1_Fbk_ResetUserDataSet();
    if (FX_L1_Fbk_RegisterUserDataSet("ROBOT_RT.m_RT_FrameSerial", FX_int, 0, 1) != FUNC_RET_SUCCESS
        || FX_L1_Fbk_RegisterUserDataSet("ROBOT_SG.m_SG_FrameSerial", FX_int, 0, 1) != FUNC_RET_SUCCESS)
    {
        printf("Failed to register user data\n");
        goto WAIT_EXIT;
    }
    if (FX_L1_Fbk_CheckUserDataSet(sizeof(UserSampleData)) != FUNC_RET_SUCCESS)
    {
        printf("Invalid user data definition\n");
        goto WAIT_EXIT;
    }
    printf("Config user sample data success\n");
    printf("Press any key to start sample data in 100 HZ\n");
    getchar();
    run_thread = 1;

    printf("Press any key to stop sample data\n");
    getchar();
    run_thread = 0;

WAIT_EXIT:
    printf("Press any key to exit\n");
    getchar();
    exit_thread = 1;
    sample_thread.join();
    return 0;
}