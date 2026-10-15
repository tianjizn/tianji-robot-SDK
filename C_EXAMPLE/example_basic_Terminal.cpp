/**
 * @file example_basic_Terminal.cpp
 * @brief Example demonstrating how to send and receive data through the
 *        L1 robot arm terminal interface (RS485).
 *
 * This example shows how to:
 * - Query SDK and controller versions
 * - Establish communication with the robot controller
 * - Prepare the arm terminal for data transmission
 * - Send and receive RS485 frames repeatedly
 * - Collect and display communication statistics
 *
 * The test performs cyclic data exchange on the ARM0 terminal
 * using RS485 channel A and measures timing performance.
 */

#include "L1Robot.h"

#ifdef _WIN32
#define SLEEP_MS(ms) Sleep(ms)
#else
#define SLEEP_MS(ms) usleep((ms) * 1000)
#endif

#define MAX_TEST_TIMES 500 ///< Number of communication test cycles

/**
 * @brief Entry point of the terminal communication example.
 *
 * Workflow overview:
 * 1. Initialize communication with the robot controller
 * 2. Retrieve SDK and controller versions
 * 3. Clear ARM0 terminal data buffers
 * 4. Send and receive RS485 frames for a fixed number of cycles
 * 5. Collect and print communication statistics
 *
 * @param[in] argc Argument count (unused)
 * @param[in] argv Argument vector (unused)
 * @return int Exit code (0 on normal exit)
 */
int main(int argc, char **argv)
{
    int sdk_version = 0;        ///< SDK version number
    int controller_version = 0; ///< Controller firmware version

    int send_frame_count = 0;        ///< Total number of sent frames
    unsigned int sending_time = 0;   ///< Time taken to send a frame (ms)
    unsigned int receiving_time = 0; ///< Time taken to receive a frame (ms)
    FXChnType chn_type;              ///< Received channel type

    unsigned char send_data[8] = {///< Data payload for transmission
                                  0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07};
    unsigned char recv_data[64] = {0}; ///< Buffer for received data

    /* Communication statistics */
    int statistics_send_frame_count = 0;         ///< Successful send frame count
    int statistics_send_frame_delay_average = 0; ///< Average send delay (ms)
    int statistics_send_frame_delay_max = 0;     ///< Maximum send delay (ms)

    int statistics_recv_frame_count = 0;         ///< Successful receive frame count
    int statistics_recv_frame_timeout_count = 0; ///< Receive timeout count
    int statistics_recv_frame_error_count = 0;   ///< Receive channel error count
    int statistics_recv_frame_delay_average = 0; ///< Average receive delay (ms)
    int statistics_recv_frame_delay_max = 0;     ///< Maximum receive delay (ms)

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

    /* Prepare ARM0 terminal for data transfer */
    printf("Before running this example, connect a serial debugging tool to the RS485A port of Arm0,"
           "and configure the tool to automatically echo back the received packets.\n");
    printf("Press any key to get ready for transferring arm0's terminal data...\n");
    getchar();
    if (FX_L1_Terminal_ClearData(FX_TERMINAL_ARM1) != FUNC_RET_SUCCESS)
    {
        printf("Failed to get ready for transferring arm0's terminal data...\n");
        goto WAIT_EXIT;
    }

    /* Start RS485 communication test */
    printf("Get ready for transferring arm0's terminal data now\n");
    printf("Press any key to send and receive frame on RS485_A for %d times\n", MAX_TEST_TIMES);
    getchar();

    while (send_frame_count < MAX_TEST_TIMES)
    {
        /* Send RS485 frame */
        if (FX_L1_Terminal_SetData(
                FX_TERMINAL_ARM0,
                FX_CHN_485A,
                1000,
                send_data,
                8,
                &sending_time) != FUNC_RET_SUCCESS)
        {
            printf("Failed to send frame[%d] in 1000ms\n", send_frame_count);
            goto WAIT_EXIT;
        }
        else
        {
            printf("Send frame[%d]: 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x\n",
                   send_frame_count,
                   send_data[0], send_data[1], send_data[2], send_data[3],
                   send_data[4], send_data[5], send_data[6], send_data[7]);

            if (sending_time > (unsigned int)statistics_send_frame_delay_max)
            {
                statistics_send_frame_delay_max = sending_time;
            }
            statistics_send_frame_delay_average += sending_time;
            statistics_send_frame_count++;
        }

        /* Receive RS485 response frame */
        if (FX_L1_Terminal_GetData(
                FX_TERMINAL_ARM0,
                1000,
                &chn_type,
                recv_data,
                &receiving_time) <= 0)
        {
            printf("Failed to receive frame[%d] in 1000ms\n", send_frame_count);
            statistics_recv_frame_timeout_count++;
        }
        else
        {
            if (chn_type == FX_CHN_485A)
            {
                printf("Recv frame[%d]: 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x\n",
                       send_frame_count,
                       recv_data[0], recv_data[1], recv_data[2], recv_data[3],
                       recv_data[4], recv_data[5], recv_data[6], recv_data[7]);

                if (receiving_time > (unsigned int)statistics_recv_frame_delay_max)
                {
                    statistics_recv_frame_delay_max = receiving_time;
                }
                statistics_recv_frame_delay_average += receiving_time;
                statistics_recv_frame_count++;
            }
            else
            {
                printf("Invalid channel type: %u\n", chn_type);
                statistics_recv_frame_error_count++;
            }
        }

        send_frame_count++;
    }

    printf("Send and receive frame on RS485_A for %d times finish\n", MAX_TEST_TIMES);

WAIT_EXIT:
    /* Calculate average delays */
    if (statistics_send_frame_count == 0)
    {
        statistics_send_frame_delay_average = 0;
    }
    else
    {
        statistics_send_frame_delay_average /= statistics_send_frame_count;
    }

    if (statistics_recv_frame_count == 0)
    {
        statistics_recv_frame_delay_average = 0;
    }
    else
    {
        statistics_recv_frame_delay_average /= statistics_recv_frame_count;
    }

    /* Print communication statistics */
    printf("/****************************************/\n");
    printf("Successful send frame count: %d\n", statistics_send_frame_count);
    printf("Average send delay: %d ms\n", statistics_send_frame_delay_average);
    printf("Max send delay: %d ms\n", statistics_send_frame_delay_max);
    printf("/****************************************/\n");
    printf("Successful receive frame count: %d\n", statistics_recv_frame_count);
    printf("Response frame timeout count: %d\n", statistics_recv_frame_timeout_count);
    printf("Response frame error count: %d\n", statistics_recv_frame_error_count);
    printf("Average response delay: %d ms\n", statistics_recv_frame_delay_average);
    printf("Max response delay: %d ms\n", statistics_recv_frame_delay_max);
    printf("/****************************************/\n");

    printf("Press any key to exit\n");
    getchar();
    return 0;
}
