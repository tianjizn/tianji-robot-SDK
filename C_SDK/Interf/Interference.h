#ifndef _INTERFERENCE_H_
#define _INTERFERENCE_H_
#include "ICBase.h"

class FXInterference
{
public:
	virtual ~FXInterference();
    /**
     * @brief Get singleton instance pointer of FXInterference module
     *
     * Global unique entry to obtain the singleton instance of interference detection module.
     * Only one FXInterference object exists during the whole program lifecycle.
     *
     * @return FXInterference* Valid pointer of interference singleton, never nullptr
     */
    static FXInterference *GetIns();

    /**
     * @brief Initialize interference module with definition configuration strings
     *
     * Load coordinate system, calibration link, input variable, convex geometry and interference constraint definitions.
     * Initialize internal data structures. Must be called before any update or interference calculation APIs.
     *
     * @param CordDef String of coordinate system definition content or file path
     * @param CalLinkDef String of link calibration parameter definition content or file path
     * @param InputDef String of external input variable mapping definition content or file path
     * @param ConvexDef String of convex collision geometry definition content or file path
     * @param ICDef String of interference check constraint rule definition content or file path
     * @return bool Return true if initialization succeeds; return false on file read failure, format error or memory allocation failure
     */
    bool FX_OnInit(char * CordDef,char * CalLinkDef,char * InputDef,char * ConvexDef,char * ICDef);

    /**
     * @brief Update coordinate input data of specified coordinate group
     *
     * Refresh real-time coordinate vector of target rigid body link for subsequent interference computation.
     *
     * @param cord_num Index number of target coordinate group to update
     * @param input Double array storing raw coordinate data, length defined by loaded configuration
     * @return bool Return true if data update succeeds; return false if index out of range or input pointer is null
     */
    bool FX_OnUpdateCord(long cord_num, double * input);

    /**
     * @brief Rebuild all convex hull collision geometry models
     *
     * Refresh all convex polyhedron collision bodies based on latest coordinate data.
     * Call this API after coordinate update to synchronize collision geometry.
     *
     * @return bool Return true if convex hull reconstruction completes normally; return false on abnormal vertex data or insufficient memory
     */
    bool FX_OnUpdateConvex();

    /**
     * @brief Calculate global interference distance calibration
     *
     * Traverse all rigid body pairs to compute minimum collision distance and interference threshold.
     * Depends on updated coordinate and convex hull data.
     *
     * @return bool Return true if calibration calculation finishes; return false if geometry data is uninitialized
     */
    bool FX_OnCalInterf();

    /**
     * @brief Get weighted average center-of-mass of all rigid bodies
     *
     * Output weighted average X/Y/Z Cartesian centroid of all links.
     *
     * @param mcp_average[3] Output double array with length 3, stores X, Y, Z centroid coordinate in sequence
     */
    void FX_OnGetCoM(double mcp_average[3]);

    /**
     * @brief Get total count of configured coordinate groups
     *
     * Read total number of coordinate systems loaded during initialization, output via reference parameter.
     *
     * @param cords_num Output reference variable receiving total coordinate group quantity
     */
    void FX_OnGetCordsNum(long &cords_num);

    /**
     * @brief Get mass parameter array of all rigid bodies
     *
     * Output scalar mass value of each link rigid body, array length equals total coordinate group count.
     *
     * @param quality Output double array storing mass value of each rigid body
     */
    void FX_OnGetM(double *quality);

    /**
     * @brief Get global minimum interference distance
     *
     * Fetch the smallest collision distance among all rigid body pairs calculated by FX_OnCalInterf,
     * used for early interference warning judgment.
     *
     * @param MinSpan Output double pointer storing global minimum geometric separation distance
     */
    void FX_OnGetMinSpan(double *MinSpan);

private:
    FXInterference();
    CICBase m_ICBase;
};

#endif
