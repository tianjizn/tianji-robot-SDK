#include <robot_data_interfaces/srv/config_clear_enc_error.hpp>
#include <robot_data_interfaces/srv/config_disable_soft_limit.hpp>
#include <robot_data_interfaces/srv/config_get_pd_cmd_cycle_time.hpp>
#include <robot_data_interfaces/srv/config_reset_axis_sensor_offset.hpp>
#include <robot_data_interfaces/srv/config_reset_enc_offset.hpp>
#include <robot_data_interfaces/srv/config_reset_sensor_offset.hpp>
#include <robot_data_interfaces/srv/config_set_brake_lock.hpp>
#include <robot_data_interfaces/srv/config_set_brake_unlock.hpp>
#include <robot_data_interfaces/srv/config_set_pd_cmd_cycle_time.hpp>
#include <robot_data_interfaces/srv/config_set_traj.hpp>
#include <robot_data_interfaces/srv/fbk_check_user_data_set.hpp>
#include <robot_data_interfaces/srv/fbk_get_ctrl_obj_dof.hpp>
#include <robot_data_interfaces/srv/fbk_get_ctrl_obj_physical_state.hpp>
#include <robot_data_interfaces/srv/fbk_get_ctrl_obj_sensor_info.hpp>
#include <robot_data_interfaces/srv/fbk_get_ctrl_obj_servo_version.hpp>
#include <robot_data_interfaces/srv/fbk_get_system_msg.hpp>
#include <robot_data_interfaces/srv/fbk_get_user_data.hpp>
#include <robot_data_interfaces/srv/fbk_register_user_data_set.hpp>
#include <robot_data_interfaces/srv/fbk_reset_system_msg.hpp>
#include <robot_data_interfaces/srv/fbk_reset_user_data_set.hpp>
#include <robot_data_interfaces/srv/param_get_float.hpp>
#include <robot_data_interfaces/srv/param_get_int32.hpp>
#include <robot_data_interfaces/srv/param_get_string.hpp>
#include <robot_data_interfaces/srv/param_set_float.hpp>
#include <robot_data_interfaces/srv/param_set_int32.hpp>
#include <robot_data_interfaces/srv/runtime_emergency_stop.hpp>
#include <robot_data_interfaces/srv/runtime_run_traj.hpp>
#include <robot_data_interfaces/srv/runtime_set_acc_ratio.hpp>
#include <robot_data_interfaces/srv/runtime_set_body_pd.hpp>
#include <robot_data_interfaces/srv/runtime_set_body_pdd.hpp>
#include <robot_data_interfaces/srv/runtime_set_body_pdp.hpp>
#include <robot_data_interfaces/srv/runtime_set_cart_k.hpp>
#include <robot_data_interfaces/srv/runtime_set_cart_kd.hpp>
#include <robot_data_interfaces/srv/runtime_set_force_ctrl.hpp>
#include <robot_data_interfaces/srv/runtime_set_ref_ori.hpp>
#include <robot_data_interfaces/srv/state_get_servo_error_code.hpp>
#include <robot_data_interfaces/srv/state_reset_error.hpp>
#include <robot_data_interfaces/srv/state_switch_to_collaborative_release.hpp>
#include <robot_data_interfaces/srv/state_switch_to_drag_cart_r.hpp>
#include <robot_data_interfaces/srv/state_switch_to_drag_cart_x.hpp>
#include <robot_data_interfaces/srv/state_switch_to_drag_cart_y.hpp>
#include <robot_data_interfaces/srv/state_switch_to_drag_cart_z.hpp>
#include <robot_data_interfaces/srv/state_switch_to_drag_joint.hpp>
#include <robot_data_interfaces/srv/state_switch_to_idle.hpp>
#include <robot_data_interfaces/srv/state_switch_to_imp_cart.hpp>
#include <robot_data_interfaces/srv/state_switch_to_imp_force.hpp>
#include <robot_data_interfaces/srv/state_switch_to_imp_joint.hpp>
#include <robot_data_interfaces/srv/state_switch_to_mit.hpp>
#include <robot_data_interfaces/srv/state_switch_to_pd.hpp>
#include <robot_data_interfaces/srv/state_switch_to_position.hpp>
#include <robot_data_interfaces/srv/state_switch_to_user_mit.hpp>
#include <robot_data_interfaces/srv/system_get_controller_version.hpp>
#include <robot_data_interfaces/srv/system_get_ip.hpp>
#include <robot_data_interfaces/srv/system_get_link_state.hpp>
#include <robot_data_interfaces/srv/system_get_log_level.hpp>
#include <robot_data_interfaces/srv/system_get_robot_type.hpp>
#include <robot_data_interfaces/srv/system_get_sdk_version.hpp>
#include <robot_data_interfaces/srv/system_get_time.hpp>
#include <robot_data_interfaces/srv/system_link.hpp>
#include <robot_data_interfaces/srv/system_reboot.hpp>
#include <robot_data_interfaces/srv/system_recv_file.hpp>
#include <robot_data_interfaces/srv/system_restore_image.hpp>
#include <robot_data_interfaces/srv/system_send_file.hpp>
#include <robot_data_interfaces/srv/system_set_ip.hpp>
#include <robot_data_interfaces/srv/system_set_log_level.hpp>
#include <robot_data_interfaces/srv/system_set_time.hpp>
#include <robot_data_interfaces/srv/system_set_user_fbk_type.hpp>
#include <robot_data_interfaces/srv/system_unlink.hpp>
#include <robot_data_interfaces/srv/system_update.hpp>
#include <robot_data_interfaces/srv/terminal_ch485_get_data.hpp>
#include <robot_data_interfaces/srv/terminal_ch485_set_data.hpp>
#include <robot_data_interfaces/srv/terminal_clear_data.hpp>

using ConfigClearEncError = robot_data_interfaces::srv::ConfigClearEncError;
using ConfigDisableSoftLimit =
    robot_data_interfaces::srv::ConfigDisableSoftLimit;
using ConfigGetPDCmdCycleTime =
    robot_data_interfaces::srv::ConfigGetPDCmdCycleTime;
using ConfigResetAxisSensorOffset =
    robot_data_interfaces::srv::ConfigResetAxisSensorOffset;
using ConfigResetEncOffset = robot_data_interfaces::srv::ConfigResetEncOffset;
using ConfigResetSensorOffset =
    robot_data_interfaces::srv::ConfigResetSensorOffset;
using ConfigSetBrakeLock = robot_data_interfaces::srv::ConfigSetBrakeLock;
using ConfigSetBrakeUnlock = robot_data_interfaces::srv::ConfigSetBrakeUnlock;
using ConfigSetPDCmdCycleTime =
    robot_data_interfaces::srv::ConfigSetPDCmdCycleTime;
using ConfigSetTraj = robot_data_interfaces::srv::ConfigSetTraj;
using FbkCheckUserDataSet = robot_data_interfaces::srv::FbkCheckUserDataSet;
using FbkGetCtrlObjDof = robot_data_interfaces::srv::FbkGetCtrlObjDof;
using FbkGetCtrlObjPhysicalState =
    robot_data_interfaces::srv::FbkGetCtrlObjPhysicalState;
using FbkGetCtrlObjSensorInfo =
    robot_data_interfaces::srv::FbkGetCtrlObjSensorInfo;
using FbkGetCtrlObjServoVersion =
    robot_data_interfaces::srv::FbkGetCtrlObjServoVersion;
using FbkGetUserData = robot_data_interfaces::srv::FbkGetUserData;
using FbkRegisterUserDataSet =
    robot_data_interfaces::srv::FbkRegisterUserDataSet;
using FbkResetSystemMsg = robot_data_interfaces::srv::FbkResetSystemMsg;
using FbkResetUserDataSet = robot_data_interfaces::srv::FbkResetUserDataSet;
using ParamGetFloat = robot_data_interfaces::srv::ParamGetFloat;
using ParamGetInt32 = robot_data_interfaces::srv::ParamGetInt32;
using ParamGetString = robot_data_interfaces::srv::ParamGetString;
using ParamSetFloat = robot_data_interfaces::srv::ParamSetFloat;
using ParamSetInt32 = robot_data_interfaces::srv::ParamSetInt32;
using RuntimeEmergencyStop = robot_data_interfaces::srv::RuntimeEmergencyStop;
using RuntimeRunTraj = robot_data_interfaces::srv::RuntimeRunTraj;
using RuntimeSetAccRatio = robot_data_interfaces::srv::RuntimeSetAccRatio;
using RuntimeSetBodyPDD = robot_data_interfaces::srv::RuntimeSetBodyPDD;
using RuntimeSetBodyPDP = robot_data_interfaces::srv::RuntimeSetBodyPDP;
using RuntimeSetBodyPD = robot_data_interfaces::srv::RuntimeSetBodyPD;
using RuntimeSetCartKD = robot_data_interfaces::srv::RuntimeSetCartKD;
using RuntimeSetCartK = robot_data_interfaces::srv::RuntimeSetCartK;
using RuntimeSetForceCtrl = robot_data_interfaces::srv::RuntimeSetForceCtrl;
using RuntimeSetRefOri = robot_data_interfaces::srv::RuntimeSetRefOri;
using StateGetServoErrorCode =
    robot_data_interfaces::srv::StateGetServoErrorCode;
using StateResetError = robot_data_interfaces::srv::StateResetError;
using StateSwitchToCollaborativeRelease =
    robot_data_interfaces::srv::StateSwitchToCollaborativeRelease;
using StateSwitchToDragCartR =
    robot_data_interfaces::srv::StateSwitchToDragCartR;
using StateSwitchToDragCartX =
    robot_data_interfaces::srv::StateSwitchToDragCartX;
using StateSwitchToDragCartY =
    robot_data_interfaces::srv::StateSwitchToDragCartY;
using StateSwitchToDragCartZ =
    robot_data_interfaces::srv::StateSwitchToDragCartZ;
using StateSwitchToDragJoint =
    robot_data_interfaces::srv::StateSwitchToDragJoint;
using StateSwitchToIdle = robot_data_interfaces::srv::StateSwitchToIdle;
using StateSwitchToImpCart = robot_data_interfaces::srv::StateSwitchToImpCart;
using StateSwitchToImpForce = robot_data_interfaces::srv::StateSwitchToImpForce;
using StateSwitchToImpJoint = robot_data_interfaces::srv::StateSwitchToImpJoint;
using StateSwitchToMIT = robot_data_interfaces::srv::StateSwitchToMIT;
using StateSwitchToPD = robot_data_interfaces::srv::StateSwitchToPD;
using StateSwitchToPosition = robot_data_interfaces::srv::StateSwitchToPosition;
using StateSwitchToUserMIT = robot_data_interfaces::srv::StateSwitchToUserMIT;
using SystemGetControllerVersion =
    robot_data_interfaces::srv::SystemGetControllerVersion;
using SystemGetIP = robot_data_interfaces::srv::SystemGetIP;
using SystemGetLinkState = robot_data_interfaces::srv::SystemGetLinkState;
using SystemGetLogLevel = robot_data_interfaces::srv::SystemGetLogLevel;
using SystemGetRobotType = robot_data_interfaces::srv::SystemGetRobotType;
using SystemGetSDKVersion = robot_data_interfaces::srv::SystemGetSDKVersion;
using SystemGetTime = robot_data_interfaces::srv::SystemGetTime;
using SystemLink = robot_data_interfaces::srv::SystemLink;
using SystemReboot = robot_data_interfaces::srv::SystemReboot;
using SystemRecvFile = robot_data_interfaces::srv::SystemRecvFile;
using SystemRestoreImage = robot_data_interfaces::srv::SystemRestoreImage;
using SystemSendFile = robot_data_interfaces::srv::SystemSendFile;
using SystemSetIP = robot_data_interfaces::srv::SystemSetIP;
using SystemSetLogLevel = robot_data_interfaces::srv::SystemSetLogLevel;
using SystemSetTime = robot_data_interfaces::srv::SystemSetTime;
using SystemSetUserFbkType = robot_data_interfaces::srv::SystemSetUserFbkType;
using SystemUnlink = robot_data_interfaces::srv::SystemUnlink;
using SystemUpdate = robot_data_interfaces::srv::SystemUpdate;
using TerminalCH485GetData = robot_data_interfaces::srv::TerminalCH485GetData;
using TerminalCH485SetData = robot_data_interfaces::srv::TerminalCH485SetData;
using TerminalClearData = robot_data_interfaces::srv::TerminalClearData;