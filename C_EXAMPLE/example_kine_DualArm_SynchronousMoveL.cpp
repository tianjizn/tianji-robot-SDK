/**
 * @file example_kine_DualArm_SynchronousMoveL.cpp
 * @brief Example demonstrating two dual-arm synchronous MoveL test functions.
 *
 * This example provides:
 * - `DualArm_SynchronousMoveL_ByIniConfig()`: keeps the original controller-linked
 *   workflow and initializes both arms with `FX_L1_Kinematics_InitSingleArm_ByIniConfig`
 * - `DualArm_SynchronousMoveL_ByInputParams()`: does not link to the controller and
 *   only performs the dual-arm synchronous MoveL planning function after
 *   `FX_L1_Kinematics_InitSingleArm_ByInputParams`
 *
 * @warning The controller-linked function executes real trajectories on both arms.
 */

#include "L1Robot.h"

#ifdef _WIN32
#define SLEEP_MS(ms) Sleep(ms)
#else
#define SLEEP_MS(ms) usleep((ms) * 1000)
#endif

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
    (void)dh_ccsm3; ///< dh_ccsm3 is defined here for different arm models but not used.

    err_code = FX_L1_Kinematics_InitSingleArm_ByInputParams(handle, robot_serial, &robot_type, dh_ccsm6, pnva, bound, gravity, mass, mcp, inertia);
    if (err_code != FUNC_RET_SUCCESS)
    {
        printf("Failed to initialize ARM%d kinematics from input parameters. Error code: %d\n", robot_serial, err_code);
        return -1;
    }
    return 0;
}

static int PrepareArmPositionMode(FXObjType obj_type,
                                  const char *arm_name,
                                  double vel_ratio,
                                  double acc_ratio)
{
    /* Parameters declaration. */
    int err_code = 0;
    unsigned int system_errorcode = 0;
    FXStateType obj_state = FX_STATE_UNKNOWN;

    /*
     * Check current arm state.
     * Brake operations are only allowed in FX_STATE_IDLE.
     */
    obj_state = FX_L1_Fbk_CurrentState(obj_type);
    if (obj_state == FX_STATE_ERROR)
    {
        printf("%s is in STATE_ERROR state now, press any key to reset error\n", arm_name);
        getchar();
        err_code = FX_L1_State_ResetError(obj_type, 1000, &system_errorcode);
        if (err_code != FUNC_RET_SUCCESS)
        {
            printf("Failed to reset %s error. Error code: %d, system error code = 0x%08x\n", arm_name, err_code, system_errorcode);
            return -1;
        }
        printf("Reset %s error success, %s is now in STATE_IDLE state\n", arm_name, arm_name);
    }
    else if (obj_state != FX_STATE_IDLE)
    {
        printf("%s is not in STATE_IDLE state now, press any key to transfer to STATE_IDLE state\n", arm_name);
        getchar();
        err_code = FX_L1_State_SwitchToIdle(obj_type, 1000);
        if (err_code != FUNC_RET_SUCCESS)
        {
            printf("Failed to transfer %s to STATE_IDLE state. Error code: %d\n", arm_name, err_code);
            return -1;
        }
    }

    /* Switch to position mode. */
    printf("%s is in STATE_IDLE state now, press any key to transfer to STATE_POSITION state\n", arm_name);
    getchar();
    err_code = FX_L1_State_SwitchToPositionMode(obj_type, 2000, vel_ratio, acc_ratio);
    if (err_code != FUNC_RET_SUCCESS)
    {
        printf("Failed to transfer %s to STATE_POSITION state. Error code: %d\n", arm_name, err_code);
        return -1;
    }
    return 0;
}

static int StartTrajectory(unsigned int obj_mask)
{
    unsigned int started_mask = 0;

    started_mask = FX_L1_Runtime_RunTraj(1, obj_mask);
    if ((started_mask & obj_mask) != obj_mask)
    {
        printf("Failed to start synchronous trajectory, returned mask = 0x%08x\n", started_mask);
        return -1;
    }
    return 0;
}

/**
 * @brief Run the controller-linked dual-arm synchronous MoveL example.
 *
 * The workflow includes:
 * 1. Get the SDK and controller versions
 * 2. Establish communication with the robot controller
 * 3. Create the kinematics context
 * 4. Initialize ARM0 and ARM1 kinematics from ini configuration
 * 5. Plan the synchronous MoveL trajectories and execute them on the controller
 *
 * @return int Exit code (0 on normal exit)
 */
int DualArm_SynchronousMoveL_ByIniConfig()
{
    /* Parameters declaration. */
    int sdk_version = 0;
    int controller_version = 0;
    int point_num = 0;
    int system_linked = 0;
    double global_vel_ = 100.0;
    double global_acc_ = 100.0;
    const ROBOT_SG *sg_ptr = FX_L1_Fbk_GetSG();

    int err_code = 0;
    int index = 0;

    FX_MotionHandle handle = 0;
    ArmsSynchronousPlanningParams planning_params = {0};
    double arm0_ref_joints[7] = {0.876, -25.548, -0.000, -87.472, -18.026, -7.201, -18.925};
    double arm1_ref_joints[7] = {-0.876, -25.548, 0.000, -87.472, 18.026, -7.201, 18.925};
    double arm0_start_xyzabc[6] = {509.733, 33.610, 265.953, -169.144, 55.012, -146.752};
    double arm1_start_xyzabc[6] = {509.733, -33.610, 265.953, 169.144, 55.012, 146.752};

    static double arm0_points[FX_MOTION_MAX_POINT_NUM * 7] = {0};
    static double arm1_points[FX_MOTION_MAX_POINT_NUM * 7] = {0};

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

    /* Prepare the arms for position control and set initial velocity and acceleration ratios. */
    if (PrepareArmPositionMode(FX_OBJ_ARM0, "Arm0", 10.0, 10.0) != 0 || PrepareArmPositionMode(FX_OBJ_ARM1, "Arm1", 10.0, 10.0) != 0)
    {
        printf("Failed to prepare arms for position mode\n");
        goto WAIT_EXIT;
    }

    /* Move to first target position. */
    printf("Arm0 is in STATE_POSITION state now\n");
    PrintJointPosition("Press any key to let arm0 move to target position", arm0_ref_joints);
    getchar();

    if (FX_L1_Runtime_SetJointPosCmd(1, FX_OBJ_ARM0, arm0_ref_joints) != FUNC_RET_SUCCESS)
    {
        printf("Failed to set Arm0's target position\n");
        goto WAIT_EXIT;
    }

    PrintJointPosition("Press any key to let arm1 move to target position", arm1_ref_joints);
    getchar();

    if (FX_L1_Runtime_SetJointPosCmd(1, FX_OBJ_ARM1, arm1_ref_joints) != FUNC_RET_SUCCESS)
    {
        printf("Failed to set Arm1's target position\n");
        goto WAIT_EXIT;
    }

    /* Set global velocity ratio and acceleration ratio for motion planning. */
    err_code = FX_L1_Runtime_SetSpeedRatio(1, FX_OBJ_ARM0, global_vel_, global_acc_);
    if (err_code != FUNC_RET_SUCCESS)
    {
        printf("Failed to set Arm0 velocity ratio. Error code: %d\n", err_code);
        goto WAIT_EXIT;
    }
    err_code = FX_L1_Runtime_SetSpeedRatio(1, FX_OBJ_ARM1, global_vel_, global_acc_);
    if (err_code != FUNC_RET_SUCCESS)
    {
        printf("Failed to set Arm1 velocity ratio. Error code: %d\n", err_code);
        goto WAIT_EXIT;
    }
    SLEEP_MS(20);
    
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

    /* Plan the synchronous MoveL trajectories for two arms. */
    printf("Arm0 and Arm1 are in STATE_POSITION state now\n");
    printf("Press any key to let Arm0 and Arm1 move to the synchronous MoveL planning start position\n");
    getchar();

    planning_params.World_Co_Flag = 0;
    planning_params.Sync_Type = 0;
    planning_params.Freq = 50;
    planning_params.Vel = 100.0;
    planning_params.Acc = 300.0;
    planning_params.Arm0_ZSP_Type = 0;
    planning_params.Arm1_ZSP_Type = 0;
    planning_params.Arm0_ZSP_Para[2] = -1.0;
    planning_params.Arm1_ZSP_Para[2] = -1.0;

    for (index = 0; index < 6; ++index)
    {
        planning_params.Arm0_Start_XYZABC[index] = arm0_start_xyzabc[index];
        planning_params.Arm1_Start_XYZABC[index] = arm1_start_xyzabc[index];
        planning_params.Arm0_End_XYZABC[index] = planning_params.Arm0_Start_XYZABC[index];
        planning_params.Arm1_End_XYZABC[index] = planning_params.Arm1_Start_XYZABC[index];
    }
    for (index = 0; index < 7; ++index)
    {
        planning_params.Arm0_Ref_Joints[index] = arm0_ref_joints[index];
        planning_params.Arm1_Ref_Joints[index] = arm1_ref_joints[index];
    }
    planning_params.Arm0_End_XYZABC[2] += 200.0;
    planning_params.Arm1_End_XYZABC[2] += 100.0;
    PrintXYZABC("Arm0 synchronous MoveL target pose", planning_params.Arm0_End_XYZABC);
    PrintXYZABC("Arm1 synchronous MoveL target pose", planning_params.Arm1_End_XYZABC);

    err_code = FX_L1_Kinematics_ArmsSynchronousPlanning(handle, &planning_params, arm0_points, arm1_points, &point_num);
    if (err_code != FUNC_RET_SUCCESS)
    {
        printf("Failed to plan dual-arm synchronous MoveL trajectory. Error code: %d\n", err_code);
        return -1;
    }
    printf("Plan dual-arm synchronous MoveL trajectory success, points = %d\n", point_num);

    /* Set trajectory. */
    printf("Press any key to set the planned synchronous MoveL trajectories\n");
    getchar();
    err_code = FX_L1_Config_SetTraj(FX_OBJ_ARM0, (unsigned int)point_num, arm0_points);
    if (err_code != FUNC_RET_SUCCESS)
    {
        printf("Failed to set ARM0 synchronous MoveL trajectory. Error code: %d\n", err_code);
        goto WAIT_EXIT;
    }
    err_code = FX_L1_Config_SetTraj(FX_OBJ_ARM1, (unsigned int)point_num, arm1_points);
    if (err_code != FUNC_RET_SUCCESS)
    {
        printf("Failed to set ARM1 synchronous MoveL trajectory. Error code: %d\n", err_code);
        goto WAIT_EXIT;
    }

    /* Run trajectory. */
    printf("Press any key to start the planned synchronous MoveL trajectories\n");
    getchar();
    if (StartTrajectory(FX_OBJ_ARM0_FLAG | FX_OBJ_ARM1_FLAG) != 0)
    {
        goto WAIT_EXIT;
    }
    printf("Dual-arm synchronous MoveL trajectories started successfully\n");

    /* Wait for the trajectory to complete. */
    do {
        SLEEP_MS(2);    
    } while (sg_ptr->m_ARMS[0].m_ARM_GET.m_ARM_FBK_TrajState != 0);
    printf("Trajectory move done!\n");

    /* Switch state to STATE_IDLE. */
    printf("After both arms finish the synchronous MoveL trajectories, press any key to transfer both arms to STATE_IDLE state\n");
    getchar();
    err_code = FX_L1_State_SwitchToIdle(FX_OBJ_ARM0, 1000);
    if (err_code != FUNC_RET_SUCCESS)
    {
        printf("Failed to transfer Arm0 to STATE_IDLE state. Error code: %d\n", err_code);
        goto WAIT_EXIT;
    }
    err_code = FX_L1_State_SwitchToIdle(FX_OBJ_ARM1, 1000);
    if (err_code != FUNC_RET_SUCCESS)
    {
        printf("Failed to transfer Arm1 to STATE_IDLE state. Error code: %d\n", err_code);
        goto WAIT_EXIT;
    }
    printf("Arm0 and Arm1 are in STATE_IDLE state now\n");
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
 * @brief Run the controller-free dual-arm synchronous MoveL example.
 *
 * The workflow includes:
 * 1. Query the SDK version
 * 2. Create the kinematics context
 * 3. Initialize ARM0 and ARM1 kinematics from input parameters
 * 4. Plan the synchronous MoveL trajectories without executing them on the controller
 *
 * @return int Exit code (0 on normal exit)
 */
int DualArm_SynchronousMoveL_ByInputParams()
{
    /* Parameters declaration. */
    int point_num = 0;
    int err_code = 0;
    int index = 0;
    int show_result_ = 1;  ///< Set to 1 to print the planned trajectory points

    FX_MotionHandle handle = 0;
    ArmsSynchronousPlanningParams planning_params = {0};
    double arm0_ref_joints[7] = {0.876, -25.548, -0.000, -87.472, -18.026, -7.201, -18.925};
    double arm1_ref_joints[7] = {-0.876, -25.548, 0.000, -87.472, 18.026, -7.201, 18.925};
    double arm0_start_xyzabc[6] = {509.733, 33.610, 265.953, -169.144, 55.012, -146.752};
    double arm1_start_xyzabc[6] = {509.733, -33.610, 265.953, 169.144, 55.012, 146.752};

    static double arm0_points[FX_MOTION_MAX_POINT_NUM * 7] = {0};
    static double arm1_points[FX_MOTION_MAX_POINT_NUM * 7] = {0};

    /* Create an independent kinematics context for all following calculation. */
    handle = FX_L1_Kinematics_Create();
    if (!handle)
    {
        printf("Failed to create kinematics context\n");
        goto WAIT_EXIT;
    }

    /* Set SDK logging. */
    FX_L1_Kinematics_SetLogLevel(FX_LOG_INFO_FLAG);

    /* Build the standalone ARM0 kinematics model directly from parameters. */
    if (InitSingleArmByInputParams(handle, 0) != 0)
    {
        goto WAIT_EXIT;
    }
    /* Build the standalone ARM1 kinematics model directly from parameters. */
    if (InitSingleArmByInputParams(handle, 1) != 0)
    {
        goto WAIT_EXIT;
    }

    /* Plan the synchronous MoveL trajectories for two arms. */
    printf("Press any key to let Arm0 and Arm1 move to the synchronous MoveL planning start position\n");
    getchar();

    planning_params.World_Co_Flag = 0;
    planning_params.Sync_Type = 0;
    planning_params.Freq = 50;
    planning_params.Vel = 100.0;
    planning_params.Acc = 300.0;
    planning_params.Arm0_ZSP_Type = 0;
    planning_params.Arm1_ZSP_Type = 0;
    planning_params.Arm0_ZSP_Para[2] = -1.0;
    planning_params.Arm1_ZSP_Para[2] = -1.0;

    for (index = 0; index < 6; ++index)
    {
        planning_params.Arm0_Start_XYZABC[index] = arm0_start_xyzabc[index];
        planning_params.Arm1_Start_XYZABC[index] = arm1_start_xyzabc[index];
        planning_params.Arm0_End_XYZABC[index] = planning_params.Arm0_Start_XYZABC[index];
        planning_params.Arm1_End_XYZABC[index] = planning_params.Arm1_Start_XYZABC[index];
    }
    for (index = 0; index < 7; ++index)
    {
        planning_params.Arm0_Ref_Joints[index] = arm0_ref_joints[index];
        planning_params.Arm1_Ref_Joints[index] = arm1_ref_joints[index];
    }
    planning_params.Arm0_End_XYZABC[2] += 200.0;
    planning_params.Arm1_End_XYZABC[2] += 100.0;
    PrintXYZABC("Arm0 synchronous MoveL target pose", planning_params.Arm0_End_XYZABC);
    PrintXYZABC("Arm1 synchronous MoveL target pose", planning_params.Arm1_End_XYZABC);

    err_code = FX_L1_Kinematics_ArmsSynchronousPlanning(handle, &planning_params, arm0_points, arm1_points, &point_num);
    if (err_code != FUNC_RET_SUCCESS)
    {
        printf("Failed to plan dual-arm synchronous MoveL trajectory. Error code: %d\n", err_code);
        return -1;
    }
    else
    {
        if (show_result_)
        {
            for (index = 0; index < point_num; ++index)
            {
                PrintJointPosition("Arm0 planned joint position", &arm0_points[index * 7]);
                PrintJointPosition("Arm1 planned joint position", &arm1_points[index * 7]);
            }
        }
    }
    printf("Plan dual-arm synchronous MoveL trajectory success, points = %d\n", point_num);

WAIT_EXIT:
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
 * `DualArm_SynchronousMoveL_ByIniConfig()`.
 * Replace it with `DualArm_SynchronousMoveL_ByInputParams()` when only the
 * controller-free planning function needs to be tested.
 *
 * @param[in] argc Argument count (unused)
 * @param[in] argv Argument vector (unused)
 * @return int Exit code (0 on normal exit)
 */
int main(int argc, char **argv)
{
    (void)argc;
    (void)argv;

    //return DualArm_SynchronousMoveL_ByInputParams();
    return DualArm_SynchronousMoveL_ByIniConfig();
}