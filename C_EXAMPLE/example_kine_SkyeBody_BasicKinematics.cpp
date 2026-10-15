/**
 * @file example_kine_SkyeBody_BasicKinematics.cpp
 * @brief Example demonstrating two Skye body coordinated kinematics test functions.
 *
 * This example provides:
 * - `SkyeBody_BasicKinematics_ByIniConfig()`: keeps the original controller-linked
 *   workflow and initializes both arms with `FX_L1_Kinematics_InitSingleArm_ByIniConfig`
 * - `SkyeBody_BasicKinematics_ByInputParams()`: does not link to the controller and
 *   only runs the coordinated Skye body kinematics calculations after
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

static void PrintBodyJoints(const char *name, double joints[3])
{
    printf("%s {%.4lf, %.4lf, %.4lf}\n", name, joints[0], joints[1], joints[2]);
}

static void PrintXYZABC(const char *name, double xyzabc[6])
{
    printf("%s {%.4lf, %.4lf, %.4lf, %.4lf, %.4lf, %.4lf}\n",
           name,
           xyzabc[0],
           xyzabc[1],
           xyzabc[2],
           xyzabc[3],
           xyzabc[4],
           xyzabc[5]);
}

static void CopyMatrix4x4(double src[4][4], double dst[4][4])
{
    int row = 0;
    int col = 0;

    for (row = 0; row < 4; ++row)
    {
        for (col = 0; col < 4; ++col)
        {
            dst[row][col] = src[row][col];
        }
    }
}

static void MultiplyMatrix4x4(double lhs[4][4], double rhs[4][4], double out[4][4])
{
    int row = 0;
    int col = 0;
    int inner = 0;

    for (row = 0; row < 4; ++row)
    {
        for (col = 0; col < 4; ++col)
        {
            out[row][col] = 0.0;
            for (inner = 0; inner < 4; ++inner)
            {
                out[row][col] += lhs[row][inner] * rhs[inner][col];
            }
        }
    }
}

static void InvertRigidMatrix4x4(double input[4][4], double output[4][4])
{
    int row = 0;
    int col = 0;

    for (row = 0; row < 4; ++row)
    {
        for (col = 0; col < 4; ++col)
        {
            output[row][col] = 0.0;
        }
    }

    for (row = 0; row < 3; ++row)
    {
        for (col = 0; col < 3; ++col)
        {
            output[row][col] = input[col][row];
        }
    }

    output[0][3] = -(output[0][0] * input[0][3] + output[0][1] * input[1][3] + output[0][2] * input[2][3]);
    output[1][3] = -(output[1][0] * input[0][3] + output[1][1] * input[1][3] + output[1][2] * input[2][3]);
    output[2][3] = -(output[2][0] * input[0][3] + output[2][1] * input[1][3] + output[2][2] * input[2][3]);
    output[3][3] = 1.0;
}

static void ExtractTranslation(double matrix[4][4], double position[3])
{
    position[0] = matrix[0][3];
    position[1] = matrix[1][3];
    position[2] = matrix[2][3];
}

static double CalcDistance3(double lhs[3], double rhs[3])
{
    double dx = lhs[0] - rhs[0];
    double dy = lhs[1] - rhs[1];
    double dz = lhs[2] - rhs[2];

    return FX_Sqrt(dx * dx + dy * dy + dz * dz);
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
    /* Unused in the current example but can be used to initialize a different arm model if needed. */
    double dh_ccsm3[8][4] = {
        {0.0, 0.0, 177.0, 0.0},
        {90.0, 0.0, 0.0, 0.0},
        {-90.0, 0.0, 272.0, 0.0},
        {90.0, 18.0, 0.0, 180.0},
        {90.0, 18.0, 256.0, 180.0},
        {90.0, 0.0, 0.0, 90.0},
        {90.0, 0.0, 0.0, 90.0},
        {90.0, 0.0, 87.0, 90.0}};
    (void)dh_ccsm3;

    /* Import parameters into the kinematics context. */
    err_code = FX_L1_Kinematics_InitSingleArm_ByInputParams(handle,
                                                            robot_serial,
                                                            &robot_type,
                                                            dh_ccsm6,
                                                            pnva,
                                                            bound,
                                                            gravity,
                                                            mass,
                                                            mcp,
                                                            inertia);
    if (err_code != FUNC_RET_SUCCESS)
    {
        printf("Failed to initialize ARM%d kinematics from input parameters. Error code: %d\n", robot_serial, err_code);
        return -1;
    }
    return 0;
}

static int DoSkyeBody_BasicKinematics(FX_MotionHandle handle)
{
    /* Parameters declaration. */
    int err_code = 0;
    FX_InvKineSolvePara arm0_ik = {0};
    FX_InvKineSolvePara arm1_ik = {0};
    double ref_body_joints[3] = {80.0, 5.0, 0.0};
    double arm0_ref_joints[7] = {44.04, -62.57, -8.92, -57.21, 1.45, -4.39, 2.10};
    double arm1_ref_joints[7] = {44.04, -62.57, -8.92, -57.21, 1.45, -4.39, 2.10};
    double arm0_shoulder_ref[4][4] = {0}, arm1_shoulder_ref[4][4] = {0};
    double arm0_tcp_ref_shoulder[4][4] = {0}, arm1_tcp_ref_shoulder[4][4] = {0};
    double arm0_tcp_ref_body[4][4] = {0}, arm1_tcp_ref_body[4][4] = {0};
    double arm0_target_body[4][4] = {0}, arm1_target_body[4][4] = {0};
    double arm0_target_xyzabc_body[6] = {0}, arm1_target_xyzabc_body[6] = {0};
    double arm0_shoulder_ref_pos[3] = {0}, arm1_shoulder_ref_pos[3] = {0};
    double arm0_tcp_ref_pos[3] = {0}, arm1_tcp_ref_pos[3] = {0};
    double arm0_target_pos[3] = {0}, arm1_target_pos[3] = {0};
    double body_stiffness[3] = {10.0, 10.0, 10.0};
    double solved_body_joints[3] = {0};
    double arm0_shoulder_solved[4][4] = {0}, arm1_shoulder_solved[4][4] = {0};
    double arm0_shoulder_inverse[4][4] = {0}, arm1_shoulder_inverse[4][4] = {0};
    double arm0_target_shoulder[4][4] = {0}, arm1_target_shoulder[4][4] = {0};
    double arm0_tcp_verify_shoulder[4][4] = {0}, arm1_tcp_verify_shoulder[4][4] = {0};
    double arm0_tcp_verify_body[4][4] = {0}, arm1_tcp_verify_body[4][4] = {0};
    double arm0_verify_xyzabc_body[6] = {0}, arm1_verify_xyzabc_body[6] = {0};
    double arm0_verify_pos[3] = {0}, arm1_verify_pos[3] = {0};
    
    /* Solve Skye forward kinematics for shoulder basic coordinates of both arms. */
    err_code = FX_L1_Kinematics_SkyeBodyForwardKinematics(handle, ref_body_joints, arm0_shoulder_ref, arm1_shoulder_ref);
    if (err_code != FUNC_RET_SUCCESS)
    {
        printf("Failed to solve Skye body forward kinematics. Error code: %d\n", err_code);
        return -1;
    }

    /* Solve forward kinematics for Arm0 to get EE TCP poses based on shoulder. */
    err_code = FX_L1_Kinematics_ForwardKinematics(handle, 0, arm0_ref_joints, arm0_tcp_ref_shoulder);
    if (err_code != FUNC_RET_SUCCESS)
    {
        printf("Failed to solve ARM0 forward kinematics. Error code: %d\n", err_code);
        return -1;
    }

    /* Solve forward kinematics for Arm1 to get EE TCP poses based on shoulder. */
    err_code = FX_L1_Kinematics_ForwardKinematics(handle, 1, arm1_ref_joints, arm1_tcp_ref_shoulder);
    if (err_code != FUNC_RET_SUCCESS)
    {
        printf("Failed to solve ARM1 forward kinematics. Error code: %d\n", err_code);
        return -1;
    }
    PrintBodyJoints("Reference Skye body joints", ref_body_joints);
    PrintJointPosition("Reference ARM0 joints", arm0_ref_joints);
    PrintJointPosition("Reference ARM1 joints", arm1_ref_joints);
    PrintMatrix("ARM0 shoulder pose in body frame", arm0_shoulder_ref);
    PrintMatrix("ARM1 shoulder pose in body frame", arm1_shoulder_ref);

    /* Calculate TCP of the robotic arm's EE relative to the body and set the offset. */
    MultiplyMatrix4x4(arm0_shoulder_ref, arm0_tcp_ref_shoulder, arm0_tcp_ref_body);
    MultiplyMatrix4x4(arm1_shoulder_ref, arm1_tcp_ref_shoulder, arm1_tcp_ref_body);
    CopyMatrix4x4(arm0_tcp_ref_body, arm0_target_body);
    CopyMatrix4x4(arm1_tcp_ref_body, arm1_target_body);
    arm0_target_body[0][3] += 20.0;
    arm0_target_body[2][3] += 15.0;
    arm1_target_body[0][3] += 20.0;
    arm1_target_body[2][3] += 5.0;
    FX_L1_Matrix2XYZABC(arm0_target_body, arm0_target_xyzabc_body);
    FX_L1_Matrix2XYZABC(arm1_target_body, arm1_target_xyzabc_body);
    PrintXYZABC("ARM0 target pose in body frame", arm0_target_xyzabc_body);
    PrintXYZABC("ARM1 target pose in body frame", arm1_target_xyzabc_body);
    ExtractTranslation(arm0_shoulder_ref, arm0_shoulder_ref_pos);
    ExtractTranslation(arm1_shoulder_ref, arm1_shoulder_ref_pos);
    ExtractTranslation(arm0_tcp_ref_body, arm0_tcp_ref_pos);
    ExtractTranslation(arm1_tcp_ref_body, arm1_tcp_ref_pos);
    ExtractTranslation(arm0_target_body, arm0_target_pos);
    ExtractTranslation(arm1_target_body, arm1_target_pos);
    err_code = FX_L1_Kinematics_SetSkyeBodyCondition(handle, ref_body_joints, body_stiffness, CalcDistance3(arm0_shoulder_ref_pos, arm0_tcp_ref_pos), 1.0, CalcDistance3(arm1_shoulder_ref_pos, arm1_tcp_ref_pos), 1.0);
    if (err_code != FUNC_RET_SUCCESS)
    {
        printf("Failed to set Skye body condition. Error code: %d\n", err_code);
        return -1;
    }

    /* Solve Skye body inverse kinematics for the target TCP positions. */
    printf("Press any key to solve Skye body inverse kinematics for the target TCP positions\n");
    getchar();
    err_code = FX_L1_Kinematics_SkyeBodyInverseKinematicsWithRef(handle, ref_body_joints, arm0_target_pos, arm1_target_pos, solved_body_joints);
    if (err_code != FUNC_RET_SUCCESS)
    {
        printf("Failed to solve Skye body inverse kinematics. Error code: %d\n", err_code);
        return -1;
    }
    PrintBodyJoints("Solved Skye body joints", solved_body_joints);
    err_code = FX_L1_Kinematics_SkyeBodyForwardKinematics(handle, solved_body_joints, arm0_shoulder_solved, arm1_shoulder_solved);
    if (err_code != FUNC_RET_SUCCESS)
    {
        printf("Failed to solve Skye body forward kinematics for the solved body joints. Error code: %d\n", err_code);
        return -1;
    }

    /* Solve the arm inverse kinematics in the solved shoulder frames. */
    printf("Press any key to solve the arm inverse kinematics in the solved shoulder frames\n");
    getchar();
    InvertRigidMatrix4x4(arm0_shoulder_solved, arm0_shoulder_inverse);
    InvertRigidMatrix4x4(arm1_shoulder_solved, arm1_shoulder_inverse);
    MultiplyMatrix4x4(arm0_shoulder_inverse, arm0_target_body, arm0_target_shoulder);
    MultiplyMatrix4x4(arm1_shoulder_inverse, arm1_target_body, arm1_target_shoulder);
    FillInverseKinematicsInput(&arm0_ik, arm0_ref_joints, arm0_target_shoulder);
    FillInverseKinematicsInput(&arm1_ik, arm1_ref_joints, arm1_target_shoulder);
    err_code = FX_L1_Kinematics_InverseKinematics(handle, 0, &arm0_ik);
    if (err_code != FUNC_RET_SUCCESS)
    {
        printf("Failed to solve ARM0 inverse kinematics in the solved shoulder frame. Error code: %d\n", err_code);
        return -1;
    }
    if (arm0_ik.m_Output_IsOutRange)
    {
        printf("The ARM0 inverse kinematics result is marked as out of range. Error code: %d\n", err_code);
        return -1;
    }
    err_code = FX_L1_Kinematics_InverseKinematics(handle, 1, &arm1_ik);
    if (err_code != FUNC_RET_SUCCESS)
    {
        printf("Failed to solve ARM1 inverse kinematics in the solved shoulder frame. Error code: %d\n", err_code);
        return -1;
    }
    if (arm1_ik.m_Output_IsOutRange)
    {
        printf("The ARM1 inverse kinematics result is marked as out of range. Error code: %d\n", err_code);
        return -1;
    }
    PrintJointPosition("Solved ARM0 joints", arm0_ik.m_Output_RetJoint);
    PrintJointPosition("Solved ARM1 joints", arm1_ik.m_Output_RetJoint);

    /* Verify the solved Skye body and arm joints by forward kinematics. */
    printf("IK solve complete. Press any key to verify result for both arms.\n");
    getchar();
    err_code = FX_L1_Kinematics_ForwardKinematics(handle, 0, arm0_ik.m_Output_RetJoint, arm0_tcp_verify_shoulder);
    if (err_code != FUNC_RET_SUCCESS)
    {
        printf("Failed to verify ARM0 forward kinematics. Error code: %d\n", err_code);
        return -1;
    }
    err_code = FX_L1_Kinematics_ForwardKinematics(handle, 1, arm1_ik.m_Output_RetJoint, arm1_tcp_verify_shoulder);
    if (err_code != FUNC_RET_SUCCESS)
    {
        printf("Failed to verify ARM1 forward kinematics. Error code: %d\n", err_code);
        return -1;
    }
    MultiplyMatrix4x4(arm0_shoulder_solved, arm0_tcp_verify_shoulder, arm0_tcp_verify_body);
    MultiplyMatrix4x4(arm1_shoulder_solved, arm1_tcp_verify_shoulder, arm1_tcp_verify_body);
    FX_L1_Matrix2XYZABC(arm0_tcp_verify_body, arm0_verify_xyzabc_body);
    FX_L1_Matrix2XYZABC(arm1_tcp_verify_body, arm1_verify_xyzabc_body);
    ExtractTranslation(arm0_tcp_verify_body, arm0_verify_pos);
    ExtractTranslation(arm1_tcp_verify_body, arm1_verify_pos);
    PrintXYZABC("Verified ARM0 pose in body frame", arm0_verify_xyzabc_body);
    PrintXYZABC("Verified ARM1 pose in body frame", arm1_verify_xyzabc_body);
    printf("ARM0 body-frame position error = %.6lf mm\n", CalcDistance3(arm0_verify_pos, arm0_target_pos));
    printf("ARM1 body-frame position error = %.6lf mm\n", CalcDistance3(arm1_verify_pos, arm1_target_pos));
    return 0;
}

/**
 * @brief Run the controller-linked Skye body coordinated kinematics example.
 *
 * The workflow includes:
 * 1. Query the SDK and controller versions
 * 2. Establish communication with the robot controller
 * 3. Create the kinematics context
 * 4. Initialize ARM0 and ARM1 kinematics from ini configuration
 * 5. Execute the coordinated Skye body kinematics test logic
 *
 * @return int Exit code (0 on normal exit)
 */
int SkyeBody_BasicKinematics_ByIniConfig()
{
    /* Parameters declaration. */
    int sdk_version = 0;
    int controller_version = 0;
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

    /* Create an independent kinematics context for all following calculation or planning calls. */
    handle = FX_L1_Kinematics_Create();
    if (!handle)
    {
        printf("Failed to create kinematics context\n");
        goto WAIT_EXIT;
    }

    /* Enable verbose SDK logging so calculation failures can be diagnosed from console output. */
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
    if (DoSkyeBody_BasicKinematics(handle) != 0)
    {
        goto WAIT_EXIT;
    }
WAIT_EXIT:
    if (handle)
    {
        FX_L1_Kinematics_Destroy(handle);
    }
    if (system_linked)
    {
        FX_L1_System_Unlink();
    }
    printf("Press any key to exit\n");
    getchar();
    return 0;
}

/**
 * @brief Run the controller-free Skye body coordinated kinematics example.
 *
 * The workflow includes:
 * 1. Query the SDK version
 * 2. Create the kinematics context
 * 3. Initialize ARM0 and ARM1 kinematics from input parameters
 * 4. Execute the coordinated Skye body kinematics test logic
 *
 * @return int Exit code (0 on normal exit)
 */
int SkyeBody_BasicKinematics_ByInputParams()
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
    FX_L1_Kinematics_SetLogLevel(FX_LOG_ALL_FLAG);

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

    /* Run the coordinated Skye body and dual-arm kinematics verification sequence. */
    if (DoSkyeBody_BasicKinematics(handle) != 0)
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
 * `SkyeBody_BasicKinematics_ByIniConfig()`.
 * Replace it with `SkyeBody_BasicKinematics_ByInputParams()` when only the
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

    return SkyeBody_BasicKinematics_ByIniConfig();
}