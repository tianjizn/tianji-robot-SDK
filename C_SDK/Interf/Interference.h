#ifndef FX_INTERFERENCE_H
#define FX_INTERFERENCE_H
#include "FXErrorCode.h"
#include <stdint.h>

#if defined(_WIN32) || defined(_WIN64)
#define INTERF_SDK_API __declspec(dllexport)
#elif defined(__linux__)
#define INTERF_SDK_API
#endif

#ifdef __cplusplus
extern "C"
{
#endif

    /**
     * @brief Opaque handle for interference detection context.
     *
     * The handle owns FXInterference singleton object,
     * contains coordinate group, convex geometry and interference constraint data.
     */
    typedef struct FX_InterfContext *FX_InterfHandle;

    /**
     * @brief Create singleton handle for interference detection context
     *
     * Allocates memory and instantiates a unique interference detection context singleton instance.
     * This handle must be passed to all subsequent interference module APIs.
     * @see FX_InterfHandle
     */
    INTERF_SDK_API FX_InterfHandle FX_Interf_Create();

    /**
     * @brief Destroy interference detection context and release resources
     *
     * Releases all memory, geometry buffers and configuration data bound to the given handle.
     * Handle becomes invalid after calling this function, do not reuse it.
     *
     * @param[in] handle Interference context handle created via FX_Interf_Create
     */
    INTERF_SDK_API void FX_Interf_Destroy(FX_InterfHandle handle);

    /**
     * @brief Initialize interference module with multi-group configuration definitions
     *
     * Parses and loads coordinate system, link calibration, input mapping, convex collision geometry
     * and interference constraint rules from input definition strings or file paths.
     * This API must be invoked before any coordinate update or distance calculation operations.
     *
     * @param[in] handle Interference context handle
     * @param[in] CordDef Coordinate system definition string or file path
     * @param[in] CalLinkDef Link calibration parameter definition string or file path
     * @param[in] InputDef External input variable mapping definition string or file path
     * @param[in] ConvexDef Convex collision geometry definition string or file path
     * @param[in] ICDef Interference check constraint rule definition string or file path
     *
     * @return Defined in FXFuncReturn.
     */
    INTERF_SDK_API int FX_Interf_Init(FX_InterfHandle handle, char *CordDef, char *CalLinkDef, char *InputDef, char *ConvexDef, char *ICDef);

    /**
     * @brief Refresh real-time coordinate data of specified coordinate group
     *
     * Updates rigid body position & attitude vectors used for later interference distance computation.
     * Call this function periodically to sync robot joint/tool real-time poses.
     *
     * @param[in] handle Interference context handle
     * @param[in] cord_num Target coordinate group index identifier
     * @param[in] input Raw coordinate double array, array length follows loaded config definition
     *
     * @return Defined in FXFuncReturn.
     */
    INTERF_SDK_API int FX_Interf_UpdateCord(FX_InterfHandle handle, long cord_num, double *input);

    /**
     * @brief Rebuild all convex hull collision geometry models
     *
     * Reconstructs convex polyhedron collision bodies using latest updated coordinate data.
     * Must be called after FX_Interf_UpdateCord to synchronize collision mesh geometry.
     *
     * @param[in] handle Interference context handle
     *
     * @return Defined in FXFuncReturn.
     */
    INTERF_SDK_API int FX_Interf_UpdateConvex(FX_InterfHandle handle);

    /**
     * @brief Calculate minimum separation distance for all rigid body collision pairs
     *
     * Traverses every defined collision pair and computes global minimal geometric separation distance.
     * Depends on synchronized coordinate data and rebuilt convex hull geometry.
     *
     * @param[in] handle Interference context handle
     *
     * @return Defined in FXFuncReturn.
     */
    INTERF_SDK_API int FX_Interf_CalcInterfDistance(FX_InterfHandle handle);

    /**
     * @brief Retrieve weighted average center-of-mass X/Y/Z of all rigid links
     *
     * Outputs mass-weighted average mass center coordinate of all loaded rigid bodies.
     * Output array stores [X, Y, Z] 3-axis spatial values sequentially.
     *
     * @param[in] handle Interference context handle
     * @param[out] mcp_average Double array buffer for output mass center [X, Y, Z], length = 3
     *
     * @return Defined in FXFuncReturn.
     */
    INTERF_SDK_API int FX_Interf_GetCoM(FX_InterfHandle handle, double mcp_average[3]);

    /**
     * @brief Get total count of loaded coordinate groups
     *
     * Reads the total number of coordinate groups parsed during FX_Interf_Init configuration loading.
     *
     * @param[in] handle Interference context handle
     * @param[out] cords_num Long pointer to receive total coordinate group quantity
     *
     * @return Defined in FXFuncReturn.
     */
    INTERF_SDK_API int FX_Interf_GetCordsNum(FX_InterfHandle handle, long *cords_num);

    /**
     * @brief Fetch mass scalar array for all defined rigid bodies
     *
     * Output mass value array, array length matches total loaded coordinate group count.
     *
     * @param[in] handle Interference context handle
     * @param[out] quality Double buffer to receive rigid body mass scalar array
     *
     * @return Defined in FXFuncReturn.
     */
    INTERF_SDK_API int FX_Interf_GetMass(FX_InterfHandle handle, double *quality);

    /**
     * @brief Get global minimal geometric separation distance across all collision pairs
     *
     * Retrieves the smallest distance value between any two collision volumes,
     * typically used for real-time early interference warning logic.
     *
     * @param[in] handle Interference context handle
     * @param[out] MinSpan Double pointer to store global minimal separation distance value
     *
     * @return Defined in FXFuncReturn.
     */
    INTERF_SDK_API int FX_Interf_GetMinSpan(FX_InterfHandle handle, double *MinSpan);

    /**
     * @brief Query interference distance value by collision pair serial index
     *
     * Read separation distance of specified collision pair via unique serial number identifier.
     *
     * @param[in] handle Interference context handle
     * @param[in] serial Unique serial index of target collision pair
     * @param[out] MinSpan Double pointer to store queried pair separation distance
     *
     * @return Defined in FXFuncReturn.
     */
    INTERF_SDK_API int FX_Interf_GetInterfSpan(FX_InterfHandle handle, long serial, double *MinSpan);

#ifdef __cplusplus
}
#endif

#endif