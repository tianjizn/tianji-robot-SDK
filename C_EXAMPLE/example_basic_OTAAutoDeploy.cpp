/**
 * @file example_basic_OTAAutoDeploy.cpp
 * @brief One-step encrypted-OTA upgrade with automatic update flag.
 *
 * This example sends an encrypted .ota package and arms the update flag
 * with a single FX_L1_System_Update() call.
 *
 * FX_L1_System_Update(update_file, ini_file) internally:
 *   1. If update_file ends with ".ota":
 *        SendFile(local -> /home/FUSION/Tmp/update_package.ota)
 *        -> the controller file server auto-decrypts it (AES-256-GCM
 *           tag verify + anti-replay) into update_package.UPDATE
 *      Else (plain package):
 *        SendFile(local -> /home/FUSION/Tmp/update_package.UPDATE)
 *   2. Optionally transfer the ini file to robot.ini.UPDATE
 *   3. Set the controller update flag (same mechanism as the 1003 SDK)
 *
 * After success, reboot the controller: run.sh detects the update flag,
 * unpacks update_package.UPDATE and runs FXAutoRun.sh to finish the upgrade.
 *
 * Prerequisites:
 *   1. Encrypt the upgrade file on the host PC before running:
 *        ./host_pack_ota update_package.UPDATE <key_hex> update_package.ota
 *   2. The controller must have the AES key provisioned at:
 *        /home/FUSION/Config/aes_key.hex
 *
 * Usage:
 *   ./example_basic_OTAAutoDeploy [local_ota_path]   (default ./update_package.ota)
 */

#include "L1Robot.h"
#include "stdio.h"

int main(int argc, char **argv)
{
    /* Resolve local .ota path (default or command-line argument) */
    const char *local_ota = "./update_package.ota";
    if (argc >= 2)
    {
        local_ota = argv[1];
    }

    /* Get SDK version */
    int sdk_version = FX_L1_System_GetSDKVersion();
    printf("SDK version is 0x%08x\n", sdk_version);

    /* Establish communication with the controller */
    if (FX_L1_System_Link(6, 6, 7, 190, FX_LOG_ALL_FLAG) < 0)
    {
        printf("Failed to link system\n");
        return -1;
    }

    /* Get controller version */
    int controller_version = FX_L1_System_GetControllerVersion();
    printf("Controller version is 0x%08x\n", controller_version);

    /* Transfer the encrypted package and set the update flag.
     * FX_L1_System_Update() detects the ".ota" suffix and uses the
     * ".ota" remote path so the controller auto-decrypts the package. */
    printf("Press any key to send OTA package [%s] and arm the update\n", local_ota);
    getchar();

    if (FX_L1_System_Update((char *)local_ota, NULL) != FUNC_RET_SUCCESS)
    {
        printf("[ERROR] FX_L1_System_Update failed\n");
        FX_L1_System_Unlink();
        return -1;
    }

    printf("\n");
    printf("[INFO] Encrypted package transferred and decrypted on the controller.\n");
    printf("[INFO] Update flag has been set on the controller.\n");
    printf("\n");
    printf("[NEXT] Reboot the controller to apply the upgrade:\n");
    printf("       1) Manually: reboot\n");
    printf("       2) Or via SDK: FX_L1_System_Reboot()\n");
    printf("       On reboot, run.sh detects the flag and applies the upgrade.\n");

    /* Release the connection */
    FX_L1_System_Unlink();

    printf("Press any key to exit\n");
    getchar();
    return 0;
}
