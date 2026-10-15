/**
 * @file example_basic_FileTransfer.cpp
 * @brief Example demonstrating how to transfer files between the local PC
 *        and the L1 robot controller.
 *
 * This example shows how to:
 * - Query SDK and controller versions
 * - Establish communication with the robot controller
 * - Upload a configuration file from the controller to the local PC
 * - Download a configuration file from the local PC to the controller
 *
 * File transfer is typically used for:
 * - Backing up robot configuration files
 * - Deploying updated configuration files to the controller
 *
 * @note File paths on the controller are Linux-style paths.
 */

#include "L1Robot.h"

/**
 * @brief Entry point of the file transfer example.
 *
 * Workflow overview:
 * 1. Initialize communication with the robot controller
 * 2. Retrieve SDK and controller versions
 * 3. Upload a configuration file from the controller to the local machine
 * 4. Download the same configuration file back to the controller
 *
 * @param[in] argc Argument count (unused)
 * @param[in] argv Argument vector (unused)
 * @return int Exit code (0 on normal exit)
 */
int main(int argc, char** argv)
{
    int sdk_version = 0;                  ///< SDK version number
    int controller_version = 0;           ///< Controller firmware version

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

    /* Upload configuration file from controller to local PC */
    printf("Press any key to upload config file robot.ini to local\n");
    getchar();
    if (FX_L1_System_RecvFile(
            "./robot.ini",
            "/home/FUSION/Config/cfg/robot.ini") != FUNC_RET_SUCCESS)
    {
        printf("Failed to transfer robot.ini to local path\n");
    }
    else
    {
        printf("Transfer robot.ini to local path success\n");
    }

    /* Download configuration file from local PC to controller */
    printf("Press any key to download config file to remote path /home/FUSION/Tmp/robot.ini\n");
    getchar();
    if (FX_L1_System_SendFile(
            "./robot.ini",
            "/home/FUSION/Tmp/robot.ini") != FUNC_RET_SUCCESS)
    {
        printf("Failed to transfer robot.ini to remote path\n");
    }
    else
    {
        printf("Transfer robot.ini to remote path success\n");
    }

WAIT_EXIT:
    printf("Press any key to exit\n");
    getchar();
    return 0;
}