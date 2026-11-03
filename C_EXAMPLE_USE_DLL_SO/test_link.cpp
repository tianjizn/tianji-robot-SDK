/**
 * @file test_link.cpp
 * @brief Minimal test for FX_L1_System_Link / FX_L1_System_Unlink.
 *
 * Must be compiled as C++ because the SDK headers contain C++ classes.
 *
 * Usage:
 *   test_link [ip1 ip2 ip3 ip4] [log_level]
 *   Default IP: 192.168.1.100, log_level: 0xFF
 */

#include <stdio.h>
#include <stdlib.h>
#include "L1Robot.h"

int main(int argc, char *argv[])
{
    int ret;
    int sdk_version = 0;
    sdk_version = FX_L1_System_GetSDKVersion();
    printf("SDK version is 0x%08x\n", sdk_version);

    ret = FX_L1_System_Link(6, 6, 7, 190, FX_LOG_ALL_FLAG);
    printf("Return code: %d", ret);
    if (ret == 0)
    {
        printf(" (success)\n");
        printf("Calling FX_L1_System_Unlink...\n");
        FX_L1_System_Unlink();
        printf("Unlink done.\n");
    }
    else
    {
        printf(" (failure)\n");
    }

    printf("=== Test finished ===\n");
    return ret;
}
