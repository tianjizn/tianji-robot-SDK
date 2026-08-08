#ifndef FXLOADIDENT_H
#define FXLOADIDENT_H
#include "DynaIdent.h"
#include "L1Robot.h"

#if defined(_WIN32) || defined(_WIN64)
#define TOOLDYN_SDK_API __declspec(dllexport)
#elif defined(__linux__)
#define TOOLDYN_SDK_API
#endif

#ifdef __cplusplus
extern "C" {
#endif



// Opaque task handle
typedef void* TaskHandle;

/**
 * @brief Start load identification task (asynchronous)
 * @param[in] arm             Arm number: 0=left arm, 1=right arm
 * @param[in] traj_file_path  Trajectory file path (.fmv format)
 * @param[in] loadcode        Load status: 0=no load, 1=with load
 * @return Task handle, or NULL on failure
 */
TOOLDYN_SDK_API TaskHandle FX_ToolDyn_Start_Sampling(int arm, int loadcode,
                        const char* traj_file_path);
/**
 * @brief Query task status
 * @param[in] handle  Task handle
 * @param[out] status  Output status
 * @param[out] progress Output progress (0~100), can be NULL
 * @param[out] Error code
 * @return 0 on success, non-zero on failure (e.g., invalid handle)
 */
TOOLDYN_SDK_API int FX_ToolDyn_Get_Sampling_Status(TaskHandle handle, ToolDynTaskStatus* status, int* progress, FXFuncReturn* error_code);

/**
 * @brief Request to stop task (asynchronous, wait until task actually stops)
 * @param[in] handle Task handle
 * @return 0 if stopped successfully, -1 if failure
 */
TOOLDYN_SDK_API int FX_ToolDyn_Stop_Sampling(TaskHandle handle);

/**
 * @brief Destroy task handle and release resources (should be called after task completion)
 * @param[in] handle Task handle
 * @return 0 on successful destruction, -1 on failure (handle is NULL or task is still running)
 */
TOOLDYN_SDK_API int FX_ToolDyn_Destroy_Sampling(TaskHandle handle);

/**
 * @brief Execute the load identification main workflow
 * @param out_dynPara  Output identification result (memory allocated by caller)
 * @param error_code   Output error code
 * @param[in] traj_file_path  Trajectory file path (.fmv format),
 * @return 0 on success, non-zero on failure
 */
TOOLDYN_SDK_API int FX_ToolDyn_Run_Load_Identification(FXFuncReturn* error_code,
                            LoadDynamicPara* out_dynPara,
                            const char* traj_file_path );

#ifdef __cplusplus
}
#endif
#endif
