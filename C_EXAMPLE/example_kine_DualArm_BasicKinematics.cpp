/**
 * @file example_kine_DualArm_BasicKinematics.cpp
 * @brief Example demonstrating two dual-arm basic kinematics test functions.
 *
 * This example provides:
 * - `DualArm_BasicKinematics_ByIniConfig()`: keeps the original controller-linked
 *   workflow and initializes both arms with `FX_L1_Kinematics_InitSingleArm_ByIniConfig`
 * - `DualArm_BasicKinematics_ByInputParams()`: does not link to the controller and
 *   only runs the core dual-arm kinematics calculations after
 *   `FX_L1_Kinematics_InitSingleArm_ByInputParams`
 */

#include "L1Robot.h"

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

static void PrintJointPosition(const char *name, double joints[7])
{
    int index = 0;

    printf("%s {", name);
    for (index = 0; index < 7; ++index)
    {
        printf(index == 6 ? "%.4lf" : "%.4lf, ", joints[index]);
    }
    printf("}\n");
}

static void SetIdentityMatrix(double matrix[4][4])
{
    int row = 0;
    int col = 0;

    for (row = 0; row < 4; ++row)
    {
        for (col = 0; col < 4; ++col)
        {
            matrix[row][col] = (row == col) ? 1.0 : 0.0;
        }
    }
}

static void FillInverseKinematicsInput(FX_InvKineSolvePara *ik_params,
                                       double ref_joints[7],
                                       double target_pose[4][4],
                                       int zsp_type = 0,
                                       const double *zsp_para = nullptr)
{
    int row = 0;
    int col = 0;
    FX_InvKineSolvePara zero_ik_params = {0};

    *ik_params = zero_ik_params;
    /* Set reference joints */
    for (row = 0; row < 7; ++row)
    {
        ik_params->m_Input_IK_RefJoint[row] = ref_joints[row];
    }

    /* Set target pose matrix*/
    for (row = 0; row < 4; ++row)
    {
        for (col = 0; col < 4; ++col)
        {
            ik_params->m_Input_IK_TargetTCP[row][col] = target_pose[row][col];
        }
    }

    /* Set ZSP parameters */
    ik_params->m_Input_IK_ZSPType = zsp_type;
    if (zsp_para != nullptr)
    {
        for (row = 0; row < 6; ++row)
        {
            ik_params->m_Input_IK_ZSPPara[row] = zsp_para[row];
        }
    }

}

static int InitSingleArmByInputParams(FX_MotionHandle handle, int robot_serial)
{
    int err_code = 0;

    /* Prepare the complete standalone kinematics parameter set for the specified arm model. */
    int robot_type = 1017;               ///< Robot type
    double gravity[3] = {0.0, 0.0, 0.0}; ///< Gravity vector
    double mass[7] = {0.0};              ///< Joint masses
    double mcp[7][3] = {0.0};            ///< Joint mass center positions
    double inertia[7][6] = {0.0};        ///< Joint inertias
    double pnva[8][4] = {
        {170.0, -170.0, 180.0, 450.0},
        {120.0, -120.0, 180.0, 450.0},
        {170.0, -170.0, 180.0, 900.0},
        {60.0, -145.0, 180.0, 900.0},
        {170.0, -170.0, 180.0, 900.0},
        {60.0, -60.0, 180.0, 900.0},
        {90.0, -90.0, 180.0, 900.0},
        {0.0, 0.0, 0.0, 0.0}}; ///< Joint positive limit, negative limit, velocity limit, and acceleration limit
    double bound[4][3] = {
        {0.0, -1.025, 110.0},
        {0.0, 1.025, 110.0},
        {0.0, -1.025, -110.0},
        {0.0, 1.025, -110.0}}; ///< Joint 67 interference curves from 1st quadrant to 4th quadrant
    double dh_ccsm6[8][4] = {
        {0.0, 0.0, 174.5, 0.0},
        {90.0, 0.0, 0.0, 0.0},
        {-90.0, 0.0, 287.0, 0.0},
        {90.0, 18.0, 0.0, 180.0},
        {90.0, 18.0, 314.0, 180.0},
        {90.0, 0.0, 0.0, 90.0},
        {90.0, 0.0, 0.0, 90.0},
        {90.0, 0.0, 95.0, 90.0}}; ///< DH parameters which the final line is flange parameters.
    double dh_ccsm3[8][4] = {
        {0.0, 0.0, 177.0, 0.0},
        {90.0, 0.0, 0.0, 0.0},
        {-90.0, 0.0, 272.0, 0.0},
        {90.0, 18.0, 0.0, 180.0},
        {90.0, 18.0, 256.0, 180.0},
        {90.0, 0.0, 0.0, 90.0},
        {90.0, 0.0, 0.0, 90.0},
        {90.0, 0.0, 87.0, 90.0}};
    (void)dh_ccsm3;///< dh_ccsm3 is defined here for different arm models but not used.

    err_code = FX_L1_Kinematics_InitSingleArm_ByInputParams(handle, robot_serial, &robot_type, dh_ccsm6, pnva, bound, gravity, mass, mcp, inertia);
    if (err_code != FUNC_RET_SUCCESS)
    {
        printf("Failed to initialize ARM%d kinematics from input parameters. Error code: %d\n", robot_serial, err_code);
        return -1;
    }
    return 0;
}

static int DoDualArm_BasicKinematics(FX_MotionHandle handle)
{
    /* Parameters declaration. */
    FX_InvKineSolvePara ik_params = {0};
    double arm0_ref_joints[7] = {17.470, -43.308, 11.804, -79.761, -10.700, -2.874, 9.134};
    double arm1_ref_joints[7] = {-17.470, -43.308, -11.804, -79.761, 10.700, -2.874, -9.134};
    double arm0_tcp_pose[4][4] = {0};
    double arm1_tcp_pose[4][4] = {0};
    double arm0_tcp_pose_with_tool[4][4] = {0};
    double arm1_tcp_pose_with_tool[4][4] = {0};
    double tool_matrix[4][4] = {0};
    double arm1_zsp_para[6] = {0, 0, -1, 0, 0, 0};
    int arm1_zsp_type = 1;
    int err_code = 0;

    /* Solve forward kinematics for both arms to get EE TCP poses. */
    err_code = FX_L1_Kinematics_ForwardKinematics(handle, 0, arm0_ref_joints, arm0_tcp_pose);
    if (err_code != FUNC_RET_SUCCESS)
    {
        printf("Failed to solve ARM0 forward kinematics. Error code: %d\n", err_code);
        return -1;
    }
    err_code = FX_L1_Kinematics_ForwardKinematics(handle, 1, arm1_ref_joints, arm1_tcp_pose);
    if (err_code != FUNC_RET_SUCCESS)
    {
        printf("Failed to solve ARM1 forward kinematics. Error code: %d\n", err_code);
        return -1;
    }
    PrintJointPosition("ARM0 reference joints", arm0_ref_joints);
    PrintMatrix("ARM0 TCP pose", arm0_tcp_pose);
    PrintJointPosition("ARM1 reference joints", arm1_ref_joints);
    PrintMatrix("ARM1 TCP pose", arm1_tcp_pose);

    /* Set a tool offset for ARM0. */
    printf("Press any key to set a 100 mm tool offset on ARM0\n");
    getchar();
    SetIdentityMatrix(tool_matrix);
    tool_matrix[2][3] = 100.0;
    err_code = FX_L1_Kinematics_SetTool(handle, 0, tool_matrix);
    if (err_code != FUNC_RET_SUCCESS)
    {
        printf("Failed to set ARM0 tool matrix. Error code: %d\n", err_code);
        return -1;
    }
    err_code = FX_L1_Kinematics_ForwardKinematics(handle, 0, arm0_ref_joints, arm0_tcp_pose_with_tool);
    if (err_code != FUNC_RET_SUCCESS)
    {
        printf("Failed to solve ARM0 forward kinematics with tool offset. Error code: %d\n", err_code);
        return -1;
    }
    PrintMatrix("ARM0 TCP pose with tool offset", arm0_tcp_pose_with_tool);

    /* Set a tool offset for ARM1. */
    printf("Press any key to set a 100 mm tool offset on ARM1\n");
    getchar();
    SetIdentityMatrix(tool_matrix);
    tool_matrix[2][3] = 100.0;
    err_code = FX_L1_Kinematics_SetTool(handle, 1, tool_matrix);
    if (err_code != FUNC_RET_SUCCESS)
    {
        printf("Failed to set ARM1 tool matrix. Error code: %d\n", err_code);
        return -1;
    }
    err_code = FX_L1_Kinematics_ForwardKinematics(handle, 1, arm1_ref_joints, arm1_tcp_pose_with_tool);
    if (err_code != FUNC_RET_SUCCESS)
    {
        printf("Failed to solve ARM1 forward kinematics with tool offset. Error code: %d\n", err_code);
        return -1;
    }
    PrintMatrix("ARM1 TCP pose with tool offset", arm1_tcp_pose_with_tool);

    /* Remove the tool offset for ARM0. */
    printf("Press any key to remove the tool offset from ARM0\n");
    getchar();
    err_code = FX_L1_Kinematics_RemoveTool(handle, 0);
    if (err_code != FUNC_RET_SUCCESS)
    {
        printf("Failed to remove ARM0 tool matrix. Error code: %d\n", err_code);
        return -1;
    }

    /* Remove the tool offset for ARM1. */
    printf("Press any key to remove the tool offset from ARM1\n");
    getchar();
    err_code = FX_L1_Kinematics_RemoveTool(handle, 1);
    if (err_code != FUNC_RET_SUCCESS)
    {
        printf("Failed to remove ARM1 tool matrix. Error code: %d\n", err_code);
        return -1;
    }

    /* Solve Inverse Kinematics for ARM0. */
    printf("Press any key to solve inverse kinematics for the original ARM0 TCP pose\n");
    getchar();
    FillInverseKinematicsInput(&ik_params, arm0_ref_joints, arm0_tcp_pose);

    printf("test: ref_j={%.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f}\n",
           ik_params.m_Input_IK_RefJoint[0], ik_params.m_Input_IK_RefJoint[1], ik_params.m_Input_IK_RefJoint[2],
           ik_params.m_Input_IK_RefJoint[3], ik_params.m_Input_IK_RefJoint[4], ik_params.m_Input_IK_RefJoint[5],
           ik_params.m_Input_IK_RefJoint[6]);
    printf("test: tcp pos=%f %f %f\n",ik_params.m_Input_IK_TargetTCP[0][3], ik_params.m_Input_IK_TargetTCP[1][3], ik_params.m_Input_IK_TargetTCP[2][3]);

    err_code = FX_L1_Kinematics_InverseKinematics(handle, 0, &ik_params);
    if (err_code != FUNC_RET_SUCCESS)
    {
        printf("Failed to solve ARM0 inverse kinematics. Error code: %d\n", err_code);
        return -1;
    }
    if (ik_params.m_Output_IsOutRange)
    {
        printf("The ARM0 inverse kinematics result is marked as out of range. Error code: %d\n", err_code);
        return -1;
    }
    PrintJointPosition("Recovered ARM0 joints", ik_params.m_Output_RetJoint);

    /* Solve Inverse Kinematics for ARM1. */
    printf("Press any key to solve inverse kinematics for the original ARM1 TCP pose\n");
    getchar();
    /* Set another ZSP Type for ARM1 to see the different results from ARM0. */
    FillInverseKinematicsInput(&ik_params, arm1_ref_joints, arm1_tcp_pose, arm1_zsp_type, arm1_zsp_para);
    err_code = FX_L1_Kinematics_InverseKinematics(handle, 1, &ik_params);
    if (err_code != FUNC_RET_SUCCESS)
    {
        printf("Failed to solve ARM1 inverse kinematics. Error code: %d\n", err_code);
        return -1;
    }
    if (ik_params.m_Output_IsOutRange)
    {
        printf("The ARM1 inverse kinematics result is marked as out of range. Error code: %d\n", err_code);
        return -1;
    }
    PrintJointPosition("Recovered ARM1 joints", ik_params.m_Output_RetJoint);

    /* Use return joints to verify the correction of Inverse Kinematics. */
    err_code = FX_L1_Kinematics_ForwardKinematics(handle, 1, ik_params.m_Output_RetJoint, arm1_tcp_pose);
    if (err_code != FUNC_RET_SUCCESS)
    {
        printf("Failed to solve ARM1 forward kinematics. Error code: %d\n", err_code);
        return -1;
    }
    PrintMatrix("Recalculate ARM1 TCP pose", arm1_tcp_pose);
    return 0;
}

/**
 * @brief Run the controller-linked dual-arm basic kinematics example.
 *
 * The workflow includes:
 * 1. Get the SDK and controller versions
 * 2. Establish communication with the robot controller
 * 3. Create the kinematics context
 * 4. Initialize ARM0 and ARM1 kinematics from ini configuration
 * 5. Execute the dual-arm forward and inverse kinematics test logic
 *
 * @return int Exit code (0 on normal exit)
 */
int DualArm_BasicKinematics_ByIniConfig()
{
    int sdk_version = 0;                ///< SDK version number
    int controller_version = 0;         ///< Controller firmware version
    int system_linked = 0;
    int err_code = 0;
    FX_MotionHandle handle = 0;

    /* Get SDK version. */
    sdk_version = FX_L1_System_GetSDKVersion();
    printf("SDK version is 0x%08x\n", sdk_version);

    /* Establish communication with the controller before loading ini-based models or sending control commands. */
    if (FX_L1_System_Link(6, 6, 7, 190, FX_LOG_ALL_FLAG) < 0)
    {
        printf("Failed to link system\n");
        goto WAIT_EXIT;
    }
    system_linked = 1;

    /* Get controller version. */
    controller_version = FX_L1_System_GetControllerVersion();
    printf("Controller version is 0x%08x\n", controller_version);

    /* Create an independent kinematics context for all following calculation. */
    handle = FX_L1_Kinematics_Create();
    if (!handle)
    {
        printf("Failed to create kinematics context\n");
        goto WAIT_EXIT;
    }
    
    /* Set SDK logging. */
    FX_L1_Kinematics_SetLogLevel(FX_LOG_ALL_FLAG);

    /* Load ARM0 kinematics parameters from the controller-side ini configuration. */
    err_code = FX_L1_Kinematics_InitSingleArm_ByIniConfig(handle, 0);
    if (err_code != FUNC_RET_SUCCESS)
    {
        printf("Failed to initialize ARM0 kinematics from ini configuration. Error code: %d\n", err_code);
        goto WAIT_EXIT;
    }
    /* Load ARM1 kinematics parameters from the controller-side ini configuration. */
    err_code = FX_L1_Kinematics_InitSingleArm_ByIniConfig(handle, 1);
    if (err_code != FUNC_RET_SUCCESS)
    {
        printf("Failed to initialize ARM1 kinematics from ini configuration. Error code: %d\n", err_code);
        goto WAIT_EXIT;
    }

    /* Run the complete dual-arm forward/inverse kinematics verification sequence. */
    if (DoDualArm_BasicKinematics(handle) != 0)
    {
        goto WAIT_EXIT;
    }

WAIT_EXIT:
    /* Release the kinematics context before leaving the current test function. */
    if (handle)
    {
        FX_L1_Kinematics_Destroy(handle);
    }
    /* Disconnect from the controller after all linked operations have completed. */
    if (system_linked)
    {
        FX_L1_System_Unlink();
    }

    printf("Press any key to exit\n");
    getchar();
    return 0;
}

/**
 * @brief Run the controller-free dual-arm basic kinematics example.
 *
 * The workflow includes:
 * 1. Get the SDK version
 * 2. Create the kinematics context
 * 3. Initialize ARM0 and ARM1 kinematics from input parameters
 * 4. Execute the dual-arm forward and inverse kinematics test logic
 *
 * @return int Exit code (0 on normal exit)
 */
int DualArm_BasicKinematics_ByInputParams()
{
    FX_MotionHandle handle = 0;

    /* Create an independent kinematics context for all following calculation or planning calls. */
    handle = FX_L1_Kinematics_Create();
    if (!handle)
    {
        printf("Failed to create kinematics context\n");
        goto WAIT_EXIT;
    }
    /* Enable verbose SDK logging so calculation failures can be diagnosed from console output. */
    FX_L1_Kinematics_SetLogLevel(FX_LOG_INFO_FLAG);

    /* Init ARM0 kinematics parameters by input parameters. */
    if (InitSingleArmByInputParams(handle, 0) != 0)
    {
        goto WAIT_EXIT;
    }
    /* Init ARM1 kinematics parameters by input parameters. */
    if (InitSingleArmByInputParams(handle, 1) != 0)
    {
        goto WAIT_EXIT;
    }

    /* Run the complete dual-arm forward/inverse kinematics verification sequence. */
    if (DoDualArm_BasicKinematics(handle) != 0)
    {
        goto WAIT_EXIT;
    }

WAIT_EXIT:
    /* Release the kinematics context before leaving the current test function. */
    if (handle)
    {
        FX_L1_Kinematics_Destroy(handle);
    }

    printf("Press any key to exit\n");
    getchar();
    return 0;
}

/**
 * @brief Entry point of the example application.
 *
 * The default workflow runs the controller-linked test function
 * `DualArm_BasicKinematics_ByIniConfig()`.
 * Replace it with `DualArm_BasicKinematics_ByInputParams()` when only the
 * controller-free kinematics function needs to be tested.
 *
 * @param[in] argc Argument count (unused)
 * @param[in] argv Argument vector (unused)
 * @return int Exit code (0 on normal exit)
 */
int main(int argc, char **argv)
{
    (void)argc;
    (void)argv;

    //return DualArm_BasicKinematics_ByInputParams();
    return DualArm_BasicKinematics_ByIniConfig();
}