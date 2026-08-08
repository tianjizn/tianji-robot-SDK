#ifndef FX_ROBOT_RT_OFFSET_TABLE_H
#define FX_ROBOT_RT_OFFSET_TABLE_H
#include "FXCommon.h"

typedef struct
{
    char name[128];
    FXUserDataType type;
    int num;
} OffsetTableItem;

static const OffsetTableItem g_RT_OffsetTable[] =
{
    {"ROBOT_RT.m_RT_FrameSerial",                           FX_int,               1},
    {"ROBOT_RT.m_DEVICE.m_DEVICE_FBK_Base_Gyro",            FX_float,             6},
    {"ROBOT_RT.m_DEVICE.m_DEVICE_FBK_Pdo_State",            FX_char,              3},
    {"ROBOT_RT.m_DEVICE.m_DEVICE_Pad",                      FX_char,              1},
    // ===================== HEAD_RT =====================
    {"ROBOT_RT.m_HEAD.m_HEAD_State.m_CurState",             FX_unsigned_short,    1},
    {"ROBOT_RT.m_HEAD.m_HEAD_State.m_CmdState",             FX_unsigned_short,    1},
    {"ROBOT_RT.m_HEAD.m_HEAD_State.m_ERRCode",              FX_unsigned_int,      1},
    {"ROBOT_RT.m_HEAD.m_HEAD_IN.m_HEAD_CMD_Joint_Pos",      FX_float,             3},
    {"ROBOT_RT.m_HEAD.m_HEAD_IN.m_HEAD_CMD_Tag",            FX_int,               1},
    {"ROBOT_RT.m_HEAD.m_HEAD_OUT.m_HEAD_FBK_Joint_Pos",     FX_float,             3},

    // ===================== ARM0 =====================
    {"ROBOT_RT.m_ARMS[0].m_ARM_State.m_CurState",           FX_unsigned_short,    1},
    {"ROBOT_RT.m_ARMS[0].m_ARM_State.m_CmdState",           FX_unsigned_short,    1},
    {"ROBOT_RT.m_ARMS[0].m_ARM_State.m_ERRCode",            FX_unsigned_int,      1},
    {"ROBOT_RT.m_ARMS[0].m_ARM_IN.m_ARM_CMD_Joint_Tor",     FX_float,             7},
    {"ROBOT_RT.m_ARMS[0].m_ARM_IN.m_ARM_CMD_Joint_Pos",     FX_float,             7},
    {"ROBOT_RT.m_ARMS[0].m_ARM_IN.m_ARM_CMD_Ctrl_ForceDir", FX_float,             5},
    {"ROBOT_RT.m_ARMS[0].m_ARM_IN.m_ARM_CMD_Ctrl_TorqueDir",FX_float,             5},
    {"ROBOT_RT.m_ARMS[0].m_ARM_IN.m_ARM_pad",               FX_char,              2},
    {"ROBOT_RT.m_ARMS[0].m_ARM_IN.m_ARM_CMD_Ctrl_RefOriType",FX_short,            1},
    {"ROBOT_RT.m_ARMS[0].m_ARM_IN.m_ARM_CMD_Ctrl_RefOri",   FX_float,             3},
    {"ROBOT_RT.m_ARMS[0].m_ARM_IN.m_ARM_CMD_Tag",           FX_int,               1},
    {"ROBOT_RT.m_ARMS[0].m_ARM_IN.m_ARM_CMD_PD_Serial",     FX_short,             1},    
    {"ROBOT_RT.m_ARMS[0].m_ARM_IN.m_ARM_CMD_Ctrl_DragType", FX_short,             1},  
    {"ROBOT_RT.m_ARMS[0].m_ARM_OUT.m_ARM_FBK_Joint_Pos",    FX_float,             7},
    {"ROBOT_RT.m_ARMS[0].m_ARM_OUT.m_ARM_FBK_Joint_Vel",    FX_float,             7},
    {"ROBOT_RT.m_ARMS[0].m_ARM_OUT.m_ARM_FBK_Joint_Cmd",    FX_float,             7},
    {"ROBOT_RT.m_ARMS[0].m_ARM_OUT.m_ARM_FBK_Joint_SensorTor", FX_float,          7},
    {"ROBOT_RT.m_ARMS[0].m_ARM_OUT.m_ARM_FBK_Joint_ExternalTorEst", FX_float,     7},
    {"ROBOT_RT.m_ARMS[0].m_ARM_OUT.m_ARM_FBK_Joint_GravityTor", FX_float,         7},    
    {"ROBOT_RT.m_ARMS[0].m_ARM_OUT.m_ARM_FBK_Base_FNEst",   FX_float,             6},
    {"ROBOT_RT.m_ARMS[0].m_ARM_OUT.m_ARM_FBK_Flange_FTSensor", FX_float,          6},

    // ===================== ARM1 =====================
    {"ROBOT_RT.m_ARMS[1].m_ARM_State.m_CurState",           FX_unsigned_short,    1},
    {"ROBOT_RT.m_ARMS[1].m_ARM_State.m_CmdState",           FX_unsigned_short,    1},
    {"ROBOT_RT.m_ARMS[1].m_ARM_State.m_ERRCode",            FX_unsigned_int,      1},
    {"ROBOT_RT.m_ARMS[1].m_ARM_IN.m_ARM_CMD_Joint_Tor",     FX_float,             7},
    {"ROBOT_RT.m_ARMS[1].m_ARM_IN.m_ARM_CMD_Joint_Pos",     FX_float,             7},
    {"ROBOT_RT.m_ARMS[1].m_ARM_IN.m_ARM_CMD_Ctrl_ForceDir", FX_float,             5},
    {"ROBOT_RT.m_ARMS[1].m_ARM_IN.m_ARM_CMD_Ctrl_TorqueDir",FX_float,             5},
    {"ROBOT_RT.m_ARMS[1].m_ARM_IN.m_ARM_pad",               FX_char,              2},
    {"ROBOT_RT.m_ARMS[1].m_ARM_IN.m_ARM_CMD_Ctrl_RefOriType",FX_short,            1},
    {"ROBOT_RT.m_ARMS[1].m_ARM_IN.m_ARM_CMD_Ctrl_RefOri",   FX_float,             3},
    {"ROBOT_RT.m_ARMS[1].m_ARM_IN.m_ARM_CMD_Tag",           FX_int,               1},
    {"ROBOT_RT.m_ARMS[1].m_ARM_IN.m_ARM_CMD_PD_Serial",     FX_short,             1},    
    {"ROBOT_RT.m_ARMS[1].m_ARM_IN.m_ARM_CMD_Ctrl_DragType", FX_short,             1},
    {"ROBOT_RT.m_ARMS[1].m_ARM_OUT.m_ARM_FBK_Joint_Pos",    FX_float,             7},
    {"ROBOT_RT.m_ARMS[1].m_ARM_OUT.m_ARM_FBK_Joint_Vel",    FX_float,             7},
    {"ROBOT_RT.m_ARMS[1].m_ARM_OUT.m_ARM_FBK_Joint_Cmd",    FX_float,             7},
    {"ROBOT_RT.m_ARMS[1].m_ARM_OUT.m_ARM_FBK_Joint_SensorTor", FX_float,          7},
    {"ROBOT_RT.m_ARMS[1].m_ARM_OUT.m_ARM_FBK_Joint_ExternalTorEst", FX_float,     7},
    {"ROBOT_RT.m_ARMS[1].m_ARM_OUT.m_ARM_FBK_Joint_GravityTor", FX_float,         7},
    {"ROBOT_RT.m_ARMS[1].m_ARM_OUT.m_ARM_FBK_Base_FNEst",   FX_float,             6},
    {"ROBOT_RT.m_ARMS[1].m_ARM_OUT.m_ARM_FBK_Flange_FTSensor", FX_float,          6},

    // ===================== BODY =====================
    {"ROBOT_RT.m_BODY.m_BODY_State.m_CurState",             FX_unsigned_short,    1},
    {"ROBOT_RT.m_BODY.m_BODY_State.m_CmdState",             FX_unsigned_short,    1},
    {"ROBOT_RT.m_BODY.m_BODY_State.m_ERRCode",              FX_unsigned_int,      1},
    {"ROBOT_RT.m_BODY.m_BODY_IN.m_BODY_CMD_Ctrl_Type",      FX_int,               1},
    {"ROBOT_RT.m_BODY.m_BODY_IN.m_BODY_CMD_Joint_Pos",      FX_float,             6},
    {"ROBOT_RT.m_BODY.m_BODY_IN.m_BODY_CMD_Tag",            FX_int,               1},
    {"ROBOT_RT.m_BODY.m_BODY_IN.m_BODY_CMD_PD_Serial",      FX_short,             1},
    {"ROBOT_RT.m_BODY.m_BODY_IN.m_BODY_pad",                FX_char,              2},
    {"ROBOT_RT.m_BODY.m_BODY_OUT.m_BODY_FBK_Joint_Pos",     FX_float,             6},
    {"ROBOT_RT.m_BODY.m_BODY_OUT.m_BODY_FBK_Joint_Vel",     FX_float,             6},
    {"ROBOT_RT.m_BODY.m_BODY_OUT.m_BODY_FBK_Joint_SensorTor", FX_float,           6},
    {"ROBOT_RT.m_BODY.m_BODY_OUT.m_BODY_FBK_Joint_GravityTor", FX_float,           6},

    // ===================== LIFT =====================
    {"ROBOT_RT.m_LIFT.m_LIFT_State.m_CurState",             FX_unsigned_short,    1},
    {"ROBOT_RT.m_LIFT.m_LIFT_State.m_CmdState",             FX_unsigned_short,    1},
    {"ROBOT_RT.m_LIFT.m_LIFT_State.m_ERRCode",              FX_unsigned_int,      1},
    {"ROBOT_RT.m_LIFT.m_LIFT_IN.m_LIFT_CMD_Joint_Pos",      FX_float,             2},
    {"ROBOT_RT.m_LIFT.m_LIFT_IN.m_LIFT_CMD_Tag",            FX_int,               1},
    {"ROBOT_RT.m_LIFT.m_LIFT_OUT.m_LIFT_FBK_Joint_Pos",     FX_float,             2},

    // ===================== SYSTEM =====================
    {"ROBOT_RT.m_SYSTEM.m_SYSTEM_UserFbk_Type",             FX_short,             4},
    {"ROBOT_RT.m_SYSTEM.m_SYSTEM_UserFbk0",                 FX_float,             8},
    {"ROBOT_RT.m_SYSTEM.m_SYSTEM_UserFbk1",                 FX_float,             8},
    {"ROBOT_RT.m_SYSTEM.m_SYSTEM_UserFbk2",                 FX_float,             8},
    {"ROBOT_RT.m_SYSTEM.m_SYSTEM_UserFbk3",                 FX_float,             8},

    {"", (FXUserDataType)0, 0 }
};

static const OffsetTableItem g_SG_OffsetTable[] =
{
    {"ROBOT_SG.m_SG_FrameSerial",                           FX_int,               1},

    // ===================== HEAD_SG =====================
    {"ROBOT_SG.m_HEAD.m_HEAD_SET.m_HEAD_Ctrl_VelRatio",     FX_float,             1},
    {"ROBOT_SG.m_HEAD.m_HEAD_SET.m_HEAD_Ctrl_AccRatio",     FX_float,             1},
    {"ROBOT_SG.m_HEAD.m_HEAD_SET.m_HEAD_SET_SetTag",        FX_char,              4},
    {"ROBOT_SG.m_HEAD.m_HEAD_SET.m_HEAD_SET_UpdateTag",     FX_char,              4},
    {"ROBOT_SG.m_HEAD.m_HEAD_GET.m_HEAD_FBK_Joint_Tor",     FX_float,             3},
    {"ROBOT_SG.m_HEAD.m_HEAD_GET.m_HEAD_FBK_Joint_ExtPos",  FX_float,             3},
    {"ROBOT_SG.m_HEAD.m_HEAD_GET.m_HEAD_FBK_Joint_Temp",    FX_char,              3},
    {"ROBOT_SG.m_HEAD.m_HEAD_GET.m_HEAD_pad",               FX_char,              1},

    // ===================== ARM_SG m_ARMS[0] =====================
    {"ROBOT_SG.m_ARMS[0].m_ARM_SET.m_ARM_Ctrl_ImpType",     FX_unsigned_int,      1},
    {"ROBOT_SG.m_ARMS[0].m_ARM_SET.m_ARM_Ctrl_VelRatio",    FX_float,             1},
    {"ROBOT_SG.m_ARMS[0].m_ARM_SET.m_ARM_Ctrl_AccRatio",    FX_float,             1},
    {"ROBOT_SG.m_ARMS[0].m_ARM_SET.m_ARM_Ctrl_JointK",      FX_float,             7},
    {"ROBOT_SG.m_ARMS[0].m_ARM_SET.m_ARM_Ctrl_JointD",      FX_float,             7},
    {"ROBOT_SG.m_ARMS[0].m_ARM_SET.m_ARM_Ctrl_CartK",       FX_float,             7},
    {"ROBOT_SG.m_ARMS[0].m_ARM_SET.m_ARM_Ctrl_CartD",       FX_float,             7},
    {"ROBOT_SG.m_ARMS[0].m_ARM_SET.m_ARM_Ctrl_ToolKine",    FX_float,             6},
    {"ROBOT_SG.m_ARMS[0].m_ARM_SET.m_ARM_Ctrl_ToolDyna",    FX_float,            10},
    {"ROBOT_SG.m_ARMS[0].m_ARM_SET.m_ARM_SET_SetTag",       FX_char,             16},
    {"ROBOT_SG.m_ARMS[0].m_ARM_SET.m_ARM_SET_UpdateTag",    FX_char,             16},
    {"ROBOT_SG.m_ARMS[0].m_ARM_GET.m_ARM_FBK_Joint_Tor",    FX_float,             7},
    {"ROBOT_SG.m_ARMS[0].m_ARM_GET.m_ARM_FBK_Joint_ExtPos", FX_float,             7},
    {"ROBOT_SG.m_ARMS[0].m_ARM_GET.m_ARM_FBK_Joint_Temp",   FX_char,              7},
    {"ROBOT_SG.m_ARMS[0].m_ARM_GET.m_ARM_FBK_Flange_DI",    FX_char,              1},
    {"ROBOT_SG.m_ARMS[0].m_ARM_GET.m_ARM_FBK_LowSpdFlag",   FX_char,              1},
    {"ROBOT_SG.m_ARMS[0].m_ARM_GET.m_ARM_FBK_TrajState",    FX_char,              1},
    {"ROBOT_SG.m_ARMS[0].m_ARM_GET.m_ARM_FBK_PD_CmdQuality",FX_char,              1},
    {"ROBOT_SG.m_ARMS[0].m_ARM_GET.m_ARM_pad",              FX_char,              1},

    // ===================== ARM_SG m_ARMS[1] =====================
    {"ROBOT_SG.m_ARMS[1].m_ARM_SET.m_ARM_Ctrl_ImpType",     FX_unsigned_int,      1},
    {"ROBOT_SG.m_ARMS[1].m_ARM_SET.m_ARM_Ctrl_VelRatio",    FX_float,             1},
    {"ROBOT_SG.m_ARMS[1].m_ARM_SET.m_ARM_Ctrl_AccRatio",    FX_float,             1},
    {"ROBOT_SG.m_ARMS[1].m_ARM_SET.m_ARM_Ctrl_JointK",      FX_float,             7},
    {"ROBOT_SG.m_ARMS[1].m_ARM_SET.m_ARM_Ctrl_JointD",      FX_float,             7},
    {"ROBOT_SG.m_ARMS[1].m_ARM_SET.m_ARM_Ctrl_CartK",       FX_float,             7},
    {"ROBOT_SG.m_ARMS[1].m_ARM_SET.m_ARM_Ctrl_CartD",       FX_float,             7},
    {"ROBOT_SG.m_ARMS[1].m_ARM_SET.m_ARM_Ctrl_ToolKine",    FX_float,             6},
    {"ROBOT_SG.m_ARMS[1].m_ARM_SET.m_ARM_Ctrl_ToolDyna",    FX_float,            10},
    {"ROBOT_SG.m_ARMS[1].m_ARM_SET.m_ARM_SET_SetTag",       FX_char,             16},
    {"ROBOT_SG.m_ARMS[1].m_ARM_SET.m_ARM_SET_UpdateTag",    FX_char,             16},
    {"ROBOT_SG.m_ARMS[1].m_ARM_GET.m_ARM_FBK_Joint_Tor",    FX_float,             7},
    {"ROBOT_SG.m_ARMS[1].m_ARM_GET.m_ARM_FBK_Joint_ExtPos", FX_float,             7},
    {"ROBOT_SG.m_ARMS[1].m_ARM_GET.m_ARM_FBK_Joint_Temp",   FX_char,              7},
    {"ROBOT_SG.m_ARMS[1].m_ARM_GET.m_ARM_FBK_Flange_DI",    FX_char,              1},
    {"ROBOT_SG.m_ARMS[1].m_ARM_GET.m_ARM_FBK_LowSpdFlag",   FX_char,              1},
    {"ROBOT_SG.m_ARMS[1].m_ARM_GET.m_ARM_FBK_TrajState",    FX_char,              1},
    {"ROBOT_SG.m_ARMS[1].m_ARM_GET.m_ARM_FBK_PD_CmdQuality",FX_char,              1},
    {"ROBOT_SG.m_ARMS[1].m_ARM_GET.m_ARM_pad",              FX_char,              1},

    // ===================== BODY_SG =====================
    {"ROBOT_SG.m_BODY.m_BODY_SET.m_BODY_Ctrl_VelRatio",     FX_float,             1},
    {"ROBOT_SG.m_BODY.m_BODY_SET.m_BODY_Ctrl_AccRatio",     FX_float,             1},
    {"ROBOT_SG.m_BODY.m_BODY_SET.m_BODY_Ctrl_PDK",          FX_float,             6},
    {"ROBOT_SG.m_BODY.m_BODY_SET.m_BODY_Ctrl_PDD",          FX_float,             6},
    {"ROBOT_SG.m_BODY.m_BODY_SET.m_BODY_SET_SetTag",        FX_char,              6},
    {"ROBOT_SG.m_BODY.m_BODY_SET.m_BODY_SET_UpdateTag",     FX_char,              6},
    {"ROBOT_SG.m_BODY.m_BODY_GET.m_BODY_FBK_Joint_Tor",     FX_float,             6},
    {"ROBOT_SG.m_BODY.m_BODY_GET.m_BODY_FBK_Joint_ExtPos",  FX_float,             6},
    {"ROBOT_SG.m_BODY.m_BODY_GET.m_BODY_FBK_Joint_Temp",    FX_char,              6},
    {"ROBOT_SG.m_BODY.m_BODY_GET.m_BODY_FBK_TrajState",     FX_char,              1},
    {"ROBOT_SG.m_BODY.m_BODY_GET.m_BODY_FBK_PD_CmdQuality", FX_char,              1},

    // ===================== LIFT_SG =====================
    {"ROBOT_SG.m_LIFT.m_LIFT_SET.m_LIFT_Ctrl_VelRatio",     FX_float,             1},
    {"ROBOT_SG.m_LIFT.m_LIFT_SET.m_LIFT_Ctrl_AccRatio",     FX_float,             1},
    {"ROBOT_SG.m_LIFT.m_LIFT_SET.m_LIFT_SET_SetTag",        FX_char,              4},
    {"ROBOT_SG.m_LIFT.m_LIFT_SET.m_LIFT_SET_UpdateTag",     FX_char,              4},
    {"ROBOT_SG.m_LIFT.m_LIFT_GET.m_LIFT_FBK_Joint_Tor",     FX_float,             2},
    {"ROBOT_SG.m_LIFT.m_LIFT_GET.m_LIFT_FBK_TrajState",     FX_char,              1},
    {"ROBOT_SG.m_LIFT.m_LIFT_GET.m_LIFT_pad",               FX_char,              3},

    // ===================== OP_SET =====================
    {"ROBOT_SG.m_OP_SET.m_OpIns",                           FX_short,             1},
    {"ROBOT_SG.m_OP_SET.m_OpValueS",                        FX_char,             30},
    {"ROBOT_SG.m_OP_SET.m_OpValueI",                        FX_int,               1},
    {"ROBOT_SG.m_OP_SET.m_OpValueF",                        FX_float,             1},
    {"ROBOT_SG.m_OP_SET.m_OpCmdSerial",                     FX_short,             1},
    {"ROBOT_SG.m_OP_SET.m_OpRetSerial",                     FX_short,             1},

    {"", (FXUserDataType)0, 0}
};

#endif