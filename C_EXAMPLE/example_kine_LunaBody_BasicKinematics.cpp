/**
 * @file example_kine_LunaBody_BasicKinematics.cpp
 * @brief Example demonstrating two Luna body coordinated kinematics test functions.
 *
 * This example provides:
 * - `SkyeBody_BasicKinematics_ByIniConfig()`: keeps the original controller-linked
 *   workflow and initializes both arms with `FX_L1_Kinematics_InitSingleArm_ByIniConfig`
 * - `SkyeBody_BasicKinematics_ByInputParams()`: does not link to the controller and
 *   only runs the coordinated Luna body kinematics calculations after
 *   `FX_L1_Kinematics_InitSingleArm_ByInputParams`
 */

#include "L1Robot.h"

enum LunaBodyInitMode
{
    LUNA_BODY_INIT_BY_INI_CONFIG = 1,
    LUNA_BODY_INIT_BY_INPUT_PARAMS = 2
};

static void PrintMatrix(const char *name, double matrix[4][4])
{
    int row = 0;
    int col = 0;

    printf("%s:\n", name);
    for (row = 0; row < 4; ++row)
    {
        for (col = 0; col < 4; ++col)
        {
            printf("%10.4lf ", matrix[row][col]);
        }
        printf("\n");
    }
}

static void PrintBodyJoints(const char *name, double joints[6])
{
    int index = 0;

    printf("%s {", name);
    for (index = 0; index < 6; ++index)
    {
        printf(index == 6 ? "%.4lf" : "%.4lf, ", joints[index]);
    }
    printf("}\n");
}

static int InitLunaBodyByInputParams(FX_MotionHandle handle)
{
    int err_code = 0;

    /* Prepare the complete standalone kinematics parameter set for the specified arm model. */
    double flange_length_ = 391.5;
    double joint_limit_pos_[6] = {11.0, 90.0, 10.0, 55.0, 30.0, 170.0};
    double joint_limit_neg_[6] = {-11.0, -10.0, -140.0, -110.0, -30.0, -170.0};
    double dh_luna_body_[6][4] = {
        {0.0, 0.0, 0.0, 0.0},
        {-90.0, 165.0, 0.0, 0.0},
        {0.0, 300.0, 0.0, 0.0},
        {0.0, 300.0, 0.0, 0.0},
        {90.0, 0.0, 0.0, 90.0},
        {90.0, 0.0, 0.0, 0.0}}; ///< DH parameters which the final line is flange parameters.

    /* Import parameters into the kinematics context. */
    err_code = FX_L1_Kinematics_LunaBodyInit_ByInputParams(handle, dh_luna_body_, flange_length_, joint_limit_neg_, joint_limit_pos_);
    if (err_code != FUNC_RET_SUCCESS)
    {
        printf("Failed to initialize Luna Body kinematics from input parameters. Error code: %d\n", err_code);
        return -1;
    }
    return 0;
}

/**
 * @brief Run the Luna body forward and inverse kinematics example.
 *
 * The initialization method is selected by `init_way_`:
 * - `1`: Establish a connection to the controller for INI-based
 *        initialization.
 * - `2`: Initialize the Luna body kinematics model using input parameters.
 *
 * After initialization, the example:
 * 1. Computes the left and right shoulder poses using forward kinematics.
 * 2. Uses the resulting shoulder poses to verify inverse kinematics.
 * 3. Applies position offsets to the shoulder poses.
 * 4. Computes a new joint solution for the offset poses.
 * 5. Runs forward kinematics again to verify the new joint solution.
 *
 * @param[in] init_way_ Luna body initialization method. Use `1` for
 *                      INI-based initialization or `2` for initialization
 *                      using input parameters.
 * @return `0` on normal exit, or `-1` if a forward or inverse kinematics
 *         calculation fails.
 */
int LunaBody_BasicKinematics(int init_way_)
{
    /* Parameters declaration. */
    FX_MotionHandle handle = 0;

    int sdk_version = 0;
    int controller_version = 0;
    int system_linked = 0;
    int err_code = 0;
    int ret_value_ = 0;

    double luna_joints_[6] = {10.15, 20.22, -30.33, -40.44, 10.15, 156.0};
    double arm0_shoulder_tcp_[4][4] = {{0}};
    double arm1_shoulder_tcp_[4][4] = {{0}};

    double verify_joints_[6] = {0};

    double offset_joints_[6] = {0};
    double arm0_shoulder_tcp_offset_[4][4] = {{0}};
    double arm1_shoulder_tcp_offset_[4][4] = {{0}};

    /* Create an independent kinematics context for all following calculation or planning calls. */
    handle = FX_L1_Kinematics_Create();
    if (!handle)
    {
        printf("Failed to create kinematics context\n");
        ret_value_ = -1;
        goto WAIT_EXIT;
    }

    /* Enable verbose SDK logging so calculation failures can be diagnosed from console output. */
    FX_L1_Kinematics_SetLogLevel(FX_LOG_ALL_FLAG);

    if (init_way_ == LUNA_BODY_INIT_BY_INI_CONFIG)
    {
        /* Get SDK version. */
        sdk_version = FX_L1_System_GetSDKVersion();
        printf("SDK version is 0x%08x\n", sdk_version);

        /* Establish communication with the controller before loading ini-based models or sending control commands. */
        if (FX_L1_System_Link(6, 6, 7, 190, FX_LOG_ALL_FLAG) < 0)
        {
            printf("Failed to link system\n");
            ret_value_ = -1;
            goto WAIT_EXIT;
        }
        system_linked = 1;

        /* Get controller version. */
        controller_version = FX_L1_System_GetControllerVersion();
        printf("Controller version is 0x%08x\n", controller_version);

        /*Init Luna Body kinematics parameters by input parameters */
        if (FX_L1_Kinematics_LunaBodyInit_ByIniConfig(handle) != 0)
        {
            printf("Failed to initialize Luna body kinematics parameters.\n");
            ret_value_ = -1;
            goto WAIT_EXIT;
        }
    }
    else if (init_way_ == LUNA_BODY_INIT_BY_INPUT_PARAMS)
    {
        /* Init Luna Body kinematics parameters by input parameters. */
        if (InitLunaBodyByInputParams(handle) != 0)
        {
            printf("Failed to initialize Luna body kinematics parameters.\n");
            ret_value_ = -1;
            goto WAIT_EXIT;
        }
    }
    else
    {
        printf("Please use the reference method for parameter initialization.\n");
        ret_value_ = -1;
        goto WAIT_EXIT;
    }

    /* Calculate forward kinematics. */
    printf("Press any key to calculate forward kinematics for luna body.\n");
    getchar();
    err_code = FX_L1_Kinematics_LunaBodyForwardKinematics(handle, luna_joints_, arm0_shoulder_tcp_, arm1_shoulder_tcp_);
    if (err_code != FUNC_RET_SUCCESS)
    {
        printf("Failed to verify Luna body forward kinematics. Error code: %d\n", err_code);
        ret_value_ = -1;
        goto WAIT_EXIT;
    }
    else
    {
        PrintMatrix("arm0_shoulder_tcp_", arm0_shoulder_tcp_);
        PrintMatrix("arm1_shoulder_tcp_", arm1_shoulder_tcp_);
    }

    /* Use forward kinemtics results to verify inverse kinematics. */
    printf("Press any key to verify inverse kinematics by using forward kinemtics results.\n");
    getchar();
    err_code = FX_L1_Kinematics_LunaBodyInverseKinematics(handle, arm0_shoulder_tcp_, arm1_shoulder_tcp_, luna_joints_, verify_joints_);
    if (err_code != FUNC_RET_SUCCESS)
    {
        printf("Failed to verify Luna body inverse kinematics. Error code: %d\n", err_code);
        ret_value_ = -1;
        goto WAIT_EXIT;
    }
    else
    {
        PrintBodyJoints("verify_joints_", verify_joints_);
    }

    /* Give an offset to verify inverse kinematics and forward kinematics. */
    printf("Press any key to verify inverse kinematics and forward kinematics by giving an offset to TCP.\n");
    getchar();

    arm0_shoulder_tcp_[0][3] += 10;
    arm1_shoulder_tcp_[0][3] += 10;
    err_code = FX_L1_Kinematics_LunaBodyInverseKinematics(handle, arm0_shoulder_tcp_, arm1_shoulder_tcp_, luna_joints_, offset_joints_);
    if (err_code != FUNC_RET_SUCCESS)
    {
        printf("Failed to verify Luna body inverse kinematics. Error code: %d\n", err_code);
        ret_value_ = -1;
        goto WAIT_EXIT;
    }
    else
    {
        PrintBodyJoints("offset_joints_", offset_joints_);
    }

    err_code = FX_L1_Kinematics_LunaBodyForwardKinematics(handle, offset_joints_, arm0_shoulder_tcp_offset_, arm1_shoulder_tcp_offset_);
    if (err_code != FUNC_RET_SUCCESS)
    {
        printf("Failed to verify Luna body forward kinematics. Error code: %d\n", err_code);
        ret_value_ = -1;
        goto WAIT_EXIT;
    }
    else
    {
        PrintMatrix("arm0_shoulder_tcp_offset_", arm0_shoulder_tcp_offset_);
        PrintMatrix("arm1_shoulder_tcp_offset_", arm1_shoulder_tcp_offset_);
    }

WAIT_EXIT:
    /* Release the kinematics context before leaving the current test function. */
    if (handle)
    {
        FX_L1_Kinematics_Destroy(handle);
    }

    printf("Press any key to exit\n");
    getchar();
    return ret_value_;
}

/**
 * @brief Entry point of the Luna body kinematics example application.
 *
 * The example initializes the Luna body kinematics model using the specified
 * initialization mode and then runs the basic forward and inverse kinematics
 * tests.
 *
 * @param[in] argc Argument count (unused).
 * @param[in] argv Argument vector (unused).
 * @return Exit code returned by `LunaBody_BasicKinematics()`.
 */
int main(int argc, char **argv)
{
    (void)argc;
    (void)argv;

    return LunaBody_BasicKinematics(LUNA_BODY_INIT_BY_INPUT_PARAMS);
    // return LunaBody_BasicKinematics(LUNA_BODY_INIT_BY_INI_CONFIG);
}