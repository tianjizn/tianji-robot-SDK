/**
 * @file example_basic_PD.cpp
 * @brief Basic example demonstrating PD (Position–Direct) control mode of L1
 * robot arm.
 *
 * This example shows how to:
 * - Initialize communication with the L1 robot system
 * - Switch the arm into PD control mode
 * - Send joint position commands cyclically
 * - Record command, feedback position, and velocity data
 * - Save sampled data to a text file
 *
 * @author L1 Robot SDK Team
 * @date 2026-05-28
 * @version 1.0
 */
#include <rclcpp/executors/multi_threaded_executor.hpp>

#include "L1Robot.h"
#include "basic_orin.h"

#ifdef _WIN32
#define SLEEP_MS(ms) Sleep(ms)
#else
#define SLEEP_MS(ms) usleep((ms)*1000)
#endif

/** @defgroup example_pd_control PD Control Example
 *  @{
 */

#define TRAJ_POINT_NUM 961  ///< Total number of trajectory points

int pos_array_count = 0;  ///< Current index of trajectory execution

/**
 * @brief Predefined joint trajectory (7 joints).
 *
 * Each row represents one trajectory point.
 */
double pos_array[TRAJ_POINT_NUM][7];

/**
 * @brief Sampled motion data array.
 *
 * Layout per point:
 * - [0..6]   : Commanded joint positions
 * - [7..13]  : Feedback joint positions
 * - [14..20] : Feedback joint velocities
 */
double sample_pos_array[TRAJ_POINT_NUM][21] = {0};

/**
 * @brief Write sampled PD data to a text file.
 *
 * @return int 0 on success, -1 on failure
 */
int WriteSample2File() {
  FILE *fp = fopen("PD_sample.txt", "w");
  if (fp == NULL) {
    printf("Failed to open file\n");
    return -1;
  }

  for (int i = 0; i < TRAJ_POINT_NUM; i++) {
    fprintf(
        fp,
        "%lf %lf %lf %lf %lf %lf %lf "
        "%lf %lf %lf %lf %lf %lf %lf "
        "%lf %lf %lf %lf %lf %lf %lf\n",
        sample_pos_array[i][0], sample_pos_array[i][1], sample_pos_array[i][2],
        sample_pos_array[i][3], sample_pos_array[i][4], sample_pos_array[i][5],
        sample_pos_array[i][6], sample_pos_array[i][7], sample_pos_array[i][8],
        sample_pos_array[i][9], sample_pos_array[i][10],
        sample_pos_array[i][11], sample_pos_array[i][12],
        sample_pos_array[i][13], sample_pos_array[i][14],
        sample_pos_array[i][15], sample_pos_array[i][16],
        sample_pos_array[i][17], sample_pos_array[i][18],
        sample_pos_array[i][19], sample_pos_array[i][20]);
  }

  fclose(fp);
  return 0;
}

/**
 * @brief Main entry point of the PD control example.
 *
 * The program performs the following steps:
 * 1. Query SDK and controller versions
 * 2. Establish communication with the robot system
 * 3. Recover from error or switch to IDLE state
 * 4. Configure PD control parameters
 * 5. Execute a predefined joint-space trajectory
 * 6. Collect feedback data and save it to file
 *
 * @param[in] argc Argument count (unused)
 * @param[in] argv Argument vector (unused)
 * @return int Exit code (0 on success)
 */
int main(int argc, char **argv) {
  // int sdk_version = 0;               ///< SDK version number
  // int controller_version = 0;        ///< Controller firmware version
  // unsigned int system_errorcode = 0; ///< Latest system error code
  // FXStateType obj_state = FX_STATE_UNKNOWN;

  // int pd_cycle_time = 0; ///< PD command cycle time (ms)

  // double k[7] = {3, 3, 3, 2, 1, 1, 1};               ///< PD proportional
  // gains double d[7] = {0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2}; ///< PD derivative
  // gains

  // double vel_ratio = 100.0; ///< Velocity scaling ratio (%)
  // double acc_ratio = 100.0; ///< Acceleration scaling ratio (%)

  // double mechanical_home[7] = {0}; ///< Mechanical home position (7 joints)

  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor executor;
  auto node = std::make_shared<BasicOrin>();
  executor.add_node(node);
  executor.spin();
  // rclcpp::spin(node);
  //---------------------------------------------------------------------------------------------------------

  rclcpp::shutdown();
  return 0;
}

/** @} */
