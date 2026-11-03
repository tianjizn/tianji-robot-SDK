#include "Interference.h"

FXInterference::FXInterference()
{

}

FXInterference::~FXInterference()
{

}

FXInterference *FXInterference::GetIns()
{
    static FXInterference s_instance;
    return &s_instance;
}

bool FXInterference::FX_OnInit(char* CordDef, char* CalLinkDef, char* InputDef, char* ConvexDef, char* ICDef)
{
    return m_ICBase.FX_OnLoadCfg(CordDef, CalLinkDef, InputDef, ConvexDef, ICDef); 
}

bool FXInterference::FX_OnUpdateCord(long cord_num, double* input)
{
    return m_ICBase.FX_OnUpdateCord(cord_num, input);
}   

bool FXInterference::FX_OnUpdateConvex()
{
    return m_ICBase.FX_OnUpdateConvex();
}

bool FXInterference::FX_OnCalInterf()
{
    return m_ICBase.FX_OnCalInterf(); 
}

void FXInterference::FX_OnGetCoM(double mcp_average[3])
{
    m_ICBase.FX_OnGetCoM(mcp_average);
}

void FXInterference::FX_OnGetCordsNum(long &cords_num)
{
    m_ICBase.FX_OnGetCordsNum(cords_num);
}

void FXInterference::FX_OnGetM(double *quality)
{
    m_ICBase.FX_OnGetM(quality);
}
void FXInterference::FX_OnGetMinSpan(double *MinSpan)
{
    m_ICBase.FX_OnGetMinSpan(MinSpan);
}
