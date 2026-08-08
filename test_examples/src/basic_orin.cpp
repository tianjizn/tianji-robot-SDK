#include "basic_orin.h"

BasicOrin::BasicOrin() : Node("basic_orin") {
  initService();
  initTopic();

  RCLCPP_INFO(this->get_logger(), "BasicPD Orin ready!");
}

// --------------------------------------------------
int BasicOrin::initService() {
  serviceClearEncError_ = this->create_service<ConfigClearEncError>(
      "config_clear_enc_error",
      std::bind(&BasicOrin::handle_ConfigClearEncError_request, this,
                std::placeholders::_1, std::placeholders::_2));
  serviceDisableSoftLimit_ = this->create_service<ConfigDisableSoftLimit>(
      "config_disable_soft_limit",
      std::bind(&BasicOrin::handle_ConfigDisableSoftLimit_request, this,
                std::placeholders::_1, std::placeholders::_2));
  serviceGetPDCmdCycleTime_ = this->create_service<ConfigGetPDCmdCycleTime>(
      "config_get_pd_cmd_cycle_time",
      std::bind(&BasicOrin::handle_ConfigGetPDCmdCycleTime_request, this,
                std::placeholders::_1, std::placeholders::_2));
  serviceResetAxisSensorOffset_ =
      this->create_service<ConfigResetAxisSensorOffset>(
          "config_reset_axis_sensor_offset",
          std::bind(&BasicOrin::handle_ConfigResetAxisSensorOffset_request,
                    this, std::placeholders::_1, std::placeholders::_2));
  serviceResetEncOffset_ = this->create_service<ConfigResetEncOffset>(
      "config_reset_enc_offset",
      std::bind(&BasicOrin::handle_ConfigResetEncOffset_request, this,
                std::placeholders::_1, std::placeholders::_2));
  serviceResetSensorOffset_ = this->create_service<ConfigResetSensorOffset>(
      "config_reset_sensor_offset",
      std::bind(&BasicOrin::handle_ConfigResetSensorOffset_request, this,
                std::placeholders::_1, std::placeholders::_2));
  serviceSetBrakeLock_ = this->create_service<ConfigSetBrakeLock>(
      "config_set_brake_lock",
      std::bind(&BasicOrin::handle_ConfigSetBrakeLock_request, this,
                std::placeholders::_1, std::placeholders::_2));
  serviceSetBrakeUnlock_ = this->create_service<ConfigSetBrakeUnlock>(
      "config_set_brake_unlock",
      std::bind(&BasicOrin::handle_ConfigSetBrakeUnlock_request, this,
                std::placeholders::_1, std::placeholders::_2));
  serviceSetPDCmdCycleTime_ = this->create_service<ConfigSetPDCmdCycleTime>(
      "config_set_pd_cmd_cycle_time",
      std::bind(&BasicOrin::handle_ConfigSetPDCmdCycleTime_request, this,
                std::placeholders::_1, std::placeholders::_2));
  serviceSetTraj_ = this->create_service<ConfigSetTraj>(
      "config_set_traj",
      std::bind(&BasicOrin::handle_ConfigSetTraj_request, this,
                std::placeholders::_1, std::placeholders::_2));
  serviceCheckUserDataSet_ = this->create_service<FbkCheckUserDataSet>(
      "fbk_check_user_data_set",
      std::bind(&BasicOrin::handle_FbkCheckUserDataSet_request, this,
                std::placeholders::_1, std::placeholders::_2));
  serviceGetCtrlObjDof_ = this->create_service<FbkGetCtrlObjDof>(
      "fbk_get_ctrl_obj_dof",
      std::bind(&BasicOrin::handle_FbkGetCtrlObjDof_request, this,
                std::placeholders::_1, std::placeholders::_2));
  serviceGetCtrlObjPhysicalState_ =
      this->create_service<FbkGetCtrlObjPhysicalState>(
          "fbk_get_ctrl_obj_physical_state",
          std::bind(&BasicOrin::handle_FbkGetCtrlObjPhysicalState_request, this,
                    std::placeholders::_1, std::placeholders::_2));
  serviceGetCtrlObjSensorInfo_ = this->create_service<FbkGetCtrlObjSensorInfo>(
      "fbk_get_ctrl_obj_sensor_info",
      std::bind(&BasicOrin::handle_FbkGetCtrlObjSensorInfo_request, this,
                std::placeholders::_1, std::placeholders::_2));
  serviceGetCtrlObjServoVersion_ =
      this->create_service<FbkGetCtrlObjServoVersion>(
          "fbk_get_ctrl_obj_servo_version",
          std::bind(&BasicOrin::handle_FbkGetCtrlObjServoVersion_request, this,
                    std::placeholders::_1, std::placeholders::_2));
  //   serviceGetSystemMsg_ = this->create_service<FbkGetSystemMsg>(
  //       "fbk_get_system_msg",
  //       std::bind(&BasicOrin::handle_FbkGetSystemMsg_request, this,
  //                 std::placeholders::_1, std::placeholders::_2));
  serviceGetUserData_ = this->create_service<FbkGetUserData>(
      "fbk_get_user_data",
      std::bind(&BasicOrin::handle_FbkGetUserData_request, this,
                std::placeholders::_1, std::placeholders::_2));
  serviceRegisterUserDataSet_ = this->create_service<FbkRegisterUserDataSet>(
      "fbk_register_user_data_set",
      std::bind(&BasicOrin::handle_FbkRegisterUserDataSet_request, this,
                std::placeholders::_1, std::placeholders::_2));
  serviceResetSystemMsg_ = this->create_service<FbkResetSystemMsg>(
      "fbk_reset_system_msg",
      std::bind(&BasicOrin::handle_FbkResetSystemMsg_request, this,
                std::placeholders::_1, std::placeholders::_2));
  serviceResetUserDataSet_ = this->create_service<FbkResetUserDataSet>(
      "fbk_reset_user_data_set",
      std::bind(&BasicOrin::handle_FbkResetUserDataSet_request, this,
                std::placeholders::_1, std::placeholders::_2));
  serviceGetFloat_ = this->create_service<ParamGetFloat>(
      "param_get_float",
      std::bind(&BasicOrin::handle_ParamGetFloat_request, this,
                std::placeholders::_1, std::placeholders::_2));
  serviceGetInt32_ = this->create_service<ParamGetInt32>(
      "param_get_int32",
      std::bind(&BasicOrin::handle_ParamGetInt32_request, this,
                std::placeholders::_1, std::placeholders::_2));
  serviceGetString_ = this->create_service<ParamGetString>(
      "param_get_string",
      std::bind(&BasicOrin::handle_ParamGetString_request, this,
                std::placeholders::_1, std::placeholders::_2));
  serviceSetFloat_ = this->create_service<ParamSetFloat>(
      "param_set_float",
      std::bind(&BasicOrin::handle_ParamSetFloat_request, this,
                std::placeholders::_1, std::placeholders::_2));
  serviceSetInt32_ = this->create_service<ParamSetInt32>(
      "param_set_int32",
      std::bind(&BasicOrin::handle_ParamSetInt32_request, this,
                std::placeholders::_1, std::placeholders::_2));
  serviceEmergencyStop_ = this->create_service<RuntimeEmergencyStop>(
      "runtime_emergency_stop",
      std::bind(&BasicOrin::handle_RuntimeEmergencyStop_request, this,
                std::placeholders::_1, std::placeholders::_2));
  serviceRunTraj_ = this->create_service<RuntimeRunTraj>(
      "runtime_run_traj",
      std::bind(&BasicOrin::handle_RuntimeRunTraj_request, this,
                std::placeholders::_1, std::placeholders::_2));
  serviceSetAccRatio_ = this->create_service<RuntimeSetAccRatio>(
      "runtime_set_acc_ratio",
      std::bind(&BasicOrin::handle_RuntimeSetAccRatio_request, this,
                std::placeholders::_1, std::placeholders::_2));
  serviceSetBodyPDD_ = this->create_service<RuntimeSetBodyPDD>(
      "runtime_set_body_pdd",
      std::bind(&BasicOrin::handle_RuntimeSetBodyPDD_request, this,
                std::placeholders::_1, std::placeholders::_2));
  serviceSetBodyPDP_ = this->create_service<RuntimeSetBodyPDP>(
      "runtime_set_body_pdp",
      std::bind(&BasicOrin::handle_RuntimeSetBodyPDP_request, this,
                std::placeholders::_1, std::placeholders::_2));
  serviceSetBodyPD_ = this->create_service<RuntimeSetBodyPD>(
      "runtime_set_body_pd",
      std::bind(&BasicOrin::handle_RuntimeSetBodyPD_request, this,
                std::placeholders::_1, std::placeholders::_2));
  serviceSetCartKD_ = this->create_service<RuntimeSetCartKD>(
      "runtime_set_cart_kd",
      std::bind(&BasicOrin::handle_RuntimeSetCartKD_request, this,
                std::placeholders::_1, std::placeholders::_2));
  serviceSetCartK_ = this->create_service<RuntimeSetCartK>(
      "runtime_set_cart_k",
      std::bind(&BasicOrin::handle_RuntimeSetCartK_request, this,
                std::placeholders::_1, std::placeholders::_2));
  serviceSetForceCtrl_ = this->create_service<RuntimeSetForceCtrl>(
      "runtime_set_force_ctrl",
      std::bind(&BasicOrin::handle_RuntimeSetForceCtrl_request, this,
                std::placeholders::_1, std::placeholders::_2));
  serviceSetRefOri_ = this->create_service<RuntimeSetRefOri>(
      "runtime_set_ref_ori",
      std::bind(&BasicOrin::handle_RuntimeSetRefOri_request, this,
                std::placeholders::_1, std::placeholders::_2));
  serviceGetServoErrorCode_ = this->create_service<StateGetServoErrorCode>(
      "state_get_servo_error_code",
      std::bind(&BasicOrin::handle_StateGetServoErrorCode_request, this,
                std::placeholders::_1, std::placeholders::_2));
  serviceResetError_ = this->create_service<StateResetError>(
      "state_reset_error",
      std::bind(&BasicOrin::handle_StateResetError_request, this,
                std::placeholders::_1, std::placeholders::_2));
  serviceSwitchToCollaborativeRelease_ =
      this->create_service<StateSwitchToCollaborativeRelease>(
          "state_switch_to_collaborative_release",
          std::bind(
              &BasicOrin::handle_StateSwitchToCollaborativeRelease_request,
              this, std::placeholders::_1, std::placeholders::_2));
  serviceSwitchToDragCartR_ = this->create_service<StateSwitchToDragCartR>(
      "state_switch_to_drag_cart_r",
      std::bind(&BasicOrin::handle_StateSwitchToDragCartR_request, this,
                std::placeholders::_1, std::placeholders::_2));
  serviceSwitchToDragCartX_ = this->create_service<StateSwitchToDragCartX>(
      "state_switch_to_drag_cart_x",
      std::bind(&BasicOrin::handle_StateSwitchToDragCartX_request, this,
                std::placeholders::_1, std::placeholders::_2));
  serviceSwitchToDragCartY_ = this->create_service<StateSwitchToDragCartY>(
      "state_switch_to_drag_cart_y",
      std::bind(&BasicOrin::handle_StateSwitchToDragCartY_request, this,
                std::placeholders::_1, std::placeholders::_2));
  serviceSwitchToDragCartZ_ = this->create_service<StateSwitchToDragCartZ>(
      "state_switch_to_drag_cart_z",
      std::bind(&BasicOrin::handle_StateSwitchToDragCartZ_request, this,
                std::placeholders::_1, std::placeholders::_2));
  serviceSwitchToDragJoint_ = this->create_service<StateSwitchToDragJoint>(
      "state_switch_to_drag_joint",
      std::bind(&BasicOrin::handle_StateSwitchToDragJoint_request, this,
                std::placeholders::_1, std::placeholders::_2));
  serviceSwitchToIdle_ = this->create_service<StateSwitchToIdle>(
      "state_switch_to_idle",
      std::bind(&BasicOrin::handle_StateSwitchToIdle_request, this,
                std::placeholders::_1, std::placeholders::_2));
  serviceSwitchToImpCart_ = this->create_service<StateSwitchToImpCart>(
      "state_switch_to_imp_cart",
      std::bind(&BasicOrin::handle_StateSwitchToImpCart_request, this,
                std::placeholders::_1, std::placeholders::_2));
  serviceSwitchToImpForce_ = this->create_service<StateSwitchToImpForce>(
      "sate_switch_to_imp_force",
      std::bind(&BasicOrin::handle_StateSwitchToImpForce_request, this,
                std::placeholders::_1, std::placeholders::_2));
  serviceSwitchToImpJoint_ = this->create_service<StateSwitchToImpJoint>(
      "state_switch_to_imp_joint",
      std::bind(&BasicOrin::handle_StateSwitchToImpJoint_request, this,
                std::placeholders::_1, std::placeholders::_2));
  serviceSwitchToMIT_ = this->create_service<StateSwitchToMIT>(
      "state_switch_to_mit",
      std::bind(&BasicOrin::handle_StateSwitchToMIT_request, this,
                std::placeholders::_1, std::placeholders::_2));
  serviceSwitchToPD_ = this->create_service<StateSwitchToPD>(
      "state_switch_to_pd",
      std::bind(&BasicOrin::handle_StateSwitchToPD_request, this,
                std::placeholders::_1, std::placeholders::_2));
  serviceSwitchToPosition_ = this->create_service<StateSwitchToPosition>(
      "state_switch_to_position",
      std::bind(&BasicOrin::handle_StateSwitchToPosition_request, this,
                std::placeholders::_1, std::placeholders::_2));
  serviceSwitchToUserMIT_ = this->create_service<StateSwitchToUserMIT>(
      "state_switch_to_userMIT",
      std::bind(&BasicOrin::handle_StateSwitchToUserMIT_request, this,
                std::placeholders::_1, std::placeholders::_2));
  serviceGetCtrlVer_ = this->create_service<SystemGetControllerVersion>(
      "system_get_controller_version",
      std::bind(&BasicOrin::handle_SystemGetControllerVersion_request, this,
                std::placeholders::_1, std::placeholders::_2));
  serviceGetIP_ = this->create_service<SystemGetIP>(
      "system_get_ip", std::bind(&BasicOrin::handle_SystemGetIP_request, this,
                                 std::placeholders::_1, std::placeholders::_2));
  serviceGetLinkState_ = this->create_service<SystemGetLinkState>(
      "system_get_link_state",
      std::bind(&BasicOrin::handle_SystemGetLinkState_request, this,
                std::placeholders::_1, std::placeholders::_2));
  serviceGetLogLevel_ = this->create_service<SystemGetLogLevel>(
      "system_get_log_level",
      std::bind(&BasicOrin::handle_SystemGetLogLevel_request, this,
                std::placeholders::_1, std::placeholders::_2));
  serviceGetRobotType_ = this->create_service<SystemGetRobotType>(
      "system_get_robot_type",
      std::bind(&BasicOrin::handle_SystemGetRobotType_request, this,
                std::placeholders::_1, std::placeholders::_2));
  serviceGetSDKVer_ = this->create_service<SystemGetSDKVersion>(
      "system_get_sdk_version",
      std::bind(&BasicOrin::handle_SystemGetSDKVersion_request, this,
                std::placeholders::_1, std::placeholders::_2));
  serviceGetTime_ = this->create_service<SystemGetTime>(
      "system_get_time",
      std::bind(&BasicOrin::handle_SystemGetTime_request, this,
                std::placeholders::_1, std::placeholders::_2));
  serviceLink_ = this->create_service<SystemLink>(
      "system_link", std::bind(&BasicOrin::handle_SystemLink_request, this,
                               std::placeholders::_1, std::placeholders::_2));
  serviceReboot_ = this->create_service<SystemReboot>(
      "system_reboot", std::bind(&BasicOrin::handle_SystemReboot_request, this,
                                 std::placeholders::_1, std::placeholders::_2));
  serviceRecvFile_ = this->create_service<SystemRecvFile>(
      "system_recv_file",
      std::bind(&BasicOrin::handle_SystemRecvFile_request, this,
                std::placeholders::_1, std::placeholders::_2));
  serviceRestoreImage_ = this->create_service<SystemRestoreImage>(
      "system_restore_image",
      std::bind(&BasicOrin::handle_SystemRestoreImage_request, this,
                std::placeholders::_1, std::placeholders::_2));
  serviceSendFile_ = this->create_service<SystemSendFile>(
      "system_send_file",
      std::bind(&BasicOrin::handle_SystemSendFile_request, this,
                std::placeholders::_1, std::placeholders::_2));
  serviceSetIP_ = this->create_service<SystemSetIP>(
      "system_set_ip", std::bind(&BasicOrin::handle_SystemSetIP_request, this,
                                 std::placeholders::_1, std::placeholders::_2));
  serviceSetLogLevel_ = this->create_service<SystemSetLogLevel>(
      "system_set_log_level",
      std::bind(&BasicOrin::handle_SystemSetLogLevel_request, this,
                std::placeholders::_1, std::placeholders::_2));
  serviceSetTime_ = this->create_service<SystemSetTime>(
      "system_set_time",
      std::bind(&BasicOrin::handle_SystemSetTime_request, this,
                std::placeholders::_1, std::placeholders::_2));
  serviceSetUserFbkType_ = this->create_service<SystemSetUserFbkType>(
      "system_set_user_fbk_type",
      std::bind(&BasicOrin::handle_SystemSetUserFbkType_request, this,
                std::placeholders::_1, std::placeholders::_2));
  serviceUnlink_ = this->create_service<SystemUnlink>(
      "system_unlink", std::bind(&BasicOrin::handle_SystemUnlink_request, this,
                                 std::placeholders::_1, std::placeholders::_2));
  serviceUpdate_ = this->create_service<SystemUpdate>(
      "System_Update", std::bind(&BasicOrin::handle_SystemUpdate_request, this,
                                 std::placeholders::_1, std::placeholders::_2));
  serviceCH485GetData_ = this->create_service<TerminalCH485GetData>(
      "terminal_ch485_get_data",
      std::bind(&BasicOrin::handle_TerminalCH485GetData_request, this,
                std::placeholders::_1, std::placeholders::_2));
  serviceCH485SetData_ = this->create_service<TerminalCH485SetData>(
      "terminal_ch485_set_data",
      std::bind(&BasicOrin::handle_TerminalCH485SetData_request, this,
                std::placeholders::_1, std::placeholders::_2));
  serviceClearData_ = this->create_service<TerminalClearData>(
      "terminal_clear_data",
      std::bind(&BasicOrin::handle_TerminalClearData_request, this,
                std::placeholders::_1, std::placeholders::_2));

  //   serviceLink_ = this->create_service<SystemLink>(
  //       "system_link", std::bind(&BasicOrin::handle_SystemLink_request, this,
  //                                std::placeholders::_1,
  //                                std::placeholders::_2));

  //   serviceUnlink_ = this->create_service<SystemUnlink>(
  //       "system_unlink", std::bind(&BasicOrin::handle_SystemUnlink_request,
  //       this,
  //                                  std::placeholders::_1,
  //                                  std::placeholders::_2));

  //   serviceGetSDKVer_ = this->create_service<SystemGetSDKVersion>(
  //       "system_get_sdk_version",
  //       std::bind(&BasicOrin::handle_SystemGetSDKVersion_request, this,
  //                 std::placeholders::_1, std::placeholders::_2));
  //   serviceGetCtrlVer_ = this->create_service<SystemGetControllerVersion>(
  //       "system_get_controller_version",
  //       std::bind(&BasicOrin::handle_SystemGetControllerVersion_request,
  //       this,
  //                 std::placeholders::_1, std::placeholders::_2));

  //   serviceResetError_ = this->create_service<StateResetError>(
  //       "state_reset_error",
  //       std::bind(&BasicOrin::handle_StateResetError_request, this,
  //                 std::placeholders::_1, std::placeholders::_2));
  //   serviceSwitchToIdle_ = this->create_service<StateSwitchToIdle>(
  //       "state_switch_to_idle",
  //       std::bind(&BasicOrin::handle_StateSwitchToIdle_request, this,
  //                 std::placeholders::_1, std::placeholders::_2));
  //   serviceSetPDCmdCycleTime_ =
  //   this->create_service<ConfigSetPDCmdCycleTime>(
  //       "config_set_pd_cmd_cycle_time",
  //       std::bind(&BasicOrin::handle_ConfigSetPDCmdCycleTime_request, this,
  //                 std::placeholders::_1, std::placeholders::_2));
  //   serviceSwitchToPD_ = this->create_service<StateSwitchToPD>(
  //       "state_switch_to_pd",
  //       std::bind(&BasicOrin::handle_StateSwitchToPD_request, this,
  //                 std::placeholders::_1, std::placeholders::_2));
  return 0;
}

// --------------------------------------------------
int BasicOrin::initTopic() {
  // sub ---------------------------------------

  subStopTraj_ = this->create_subscription<RuntimeStopTraj>(
      "/robot/runtime_stop_traj", 100,
      std::bind(&BasicOrin::handle_RuntimeStopTraj_callback, this,
                std::placeholders::_1));

  subSetVelRatio_ = this->create_subscription<RuntimeSetVelRatio>(
      "/robot/runtime_set_vel_ratio", 100,
      std::bind(&BasicOrin::handle_RuntimeSetVelRatio_callback, this,
                std::placeholders::_1));

  subSetTorqueCtrl_ = this->create_subscription<RuntimeSetTorqueCtrl>(
      "/robot/runtime_set_torque_ctrl", 100,
      std::bind(&BasicOrin::handle_RuntimeSetTorqueCtrl_callback, this,
                std::placeholders::_1));

  subSetToolKD_ = this->create_subscription<RuntimeSetToolKD>(
      "/robot/runtime_set_tool_kd", 100,
      std::bind(&BasicOrin::handle_RuntimeSetToolKD_callback, this,
                std::placeholders::_1));

  subSetToolK_ = this->create_subscription<RuntimeSetToolK>(
      "/robot/runtime_set_tool_k", 100,
      std::bind(&BasicOrin::handle_RuntimeSetToolK_callback, this,
                std::placeholders::_1));

  subSetToolD_ = this->create_subscription<RuntimeSetToolD>(
      "/robot/runtime_set_tool_d", 100,
      std::bind(&BasicOrin::handle_RuntimeSetToolD_callback, this,
                std::placeholders::_1));

  subSetTag_ = this->create_subscription<RuntimeSetTag>(
      "/robot/runtime_set_tag", 100,
      std::bind(&BasicOrin::handle_RuntimeSetTag_callback, this,
                std::placeholders::_1));

  subSetSpeedRatio_ = this->create_subscription<RuntimeSetSpeedRatio>(
      "/robot/runtime_set_speed_ratio", 100,
      std::bind(&BasicOrin::handle_RuntimeSetSpeedRatio_callback, this,
                std::placeholders::_1));

  subSetJointMITCmd_ = this->create_subscription<RuntimeSetJointMITCmd>(
      "/robot/runtime_set_joint_mit_cmd", 100,
      std::bind(&BasicOrin::handle_RuntimeSetJointMITCmd_callback, this,
                std::placeholders::_1));

  subSetJointKD_ = this->create_subscription<RuntimeSetJointKD>(
      "/robot/runtime_set_joint_kd", 100,
      std::bind(&BasicOrin::handle_RuntimeSetJointKD_callback, this,
                std::placeholders::_1));

  subSetJointK_ = this->create_subscription<RuntimeSetJointK>(
      "/robot/runtime_set_joint_k", 100,
      std::bind(&BasicOrin::handle_RuntimeSetJointK_callback, this,
                std::placeholders::_1));

  subSetJointD_ = this->create_subscription<RuntimeSetJointD>(
      "/robot/runtime_set_joint_d", 100,
      std::bind(&BasicOrin::handle_RuntimeSetJointD_callback, this,
                std::placeholders::_1));

  subSetJointPosCmd_ = this->create_subscription<RuntimeSetJointPosCmd>(
      "/robot/runtime_set_joint_pos_cmd", 100,
      std::bind(&BasicOrin::handle_RuntimeSetJointPosCmd_callback, this,
                std::placeholders::_1));

  subSetJointPosPDCmd_ = this->create_subscription<RuntimeSetJointPosPDCmd>(
      "/robot/runtime_set_joint_pos_pd_cmd", 100,
      std::bind(&BasicOrin::handle_RuntimeSetJointPosPDCmd_callback, this,
                std::placeholders::_1));

  // pub---------------------------------------
  pubRobotSG_ = this->create_publisher<RobotSG>("/robot/sg_feedback", 250);
  pubRobotRT_ = this->create_publisher<RobotRT>("/robot/rt_feedback", 500);
  pubStateArm0_ =
      this->create_publisher<FXStateTypeArm0>("/robot/state_arm0", 100);
  //---------------------------------------
  timerSG_ =
      this->create_wall_timer(std::chrono::milliseconds(1000),
                              std::bind(&BasicOrin::timerSGCallback, this));
  timerRT_ =
      this->create_wall_timer(std::chrono::milliseconds(1000),
                              std::bind(&BasicOrin::timerRTCallback, this));
  timerStateType_ = this->create_wall_timer(
      std::chrono::milliseconds(50),
      std::bind(&BasicOrin::timerStateArm0Callback, this));
}

// Service
// ---------------------------------------------------------------------------------------------------------
void BasicOrin::handle_ConfigClearEncError_request(
    const std::shared_ptr<ConfigClearEncError::Request> request,
    std::shared_ptr<ConfigClearEncError::Response> response) {
  RCLCPP_INFO(this->get_logger(), "Received ConfigClearEncError request!");

  response->result = FX_L1_Config_ClearEncError(
      static_cast<FXObjType>(request->obj_type), request->axis_mask);

  RCLCPP_INFO(this->get_logger(), "Response ConfigClearEncError: result = %d",
              response->result);
}

void BasicOrin::handle_ConfigDisableSoftLimit_request(
    const std::shared_ptr<ConfigDisableSoftLimit::Request> request,
    std::shared_ptr<ConfigDisableSoftLimit::Response> response) {
  RCLCPP_INFO(this->get_logger(), "Received ConfigDisableSoftLimit request!");

  response->result = FX_L1_Config_DisableSoftLimit(
      static_cast<FXObjType>(request->obj_type), request->axis_mask);

  RCLCPP_INFO(this->get_logger(),
              "Response ConfigDisableSoftLimit: result = %d", response->result);
}

void BasicOrin::handle_ConfigGetPDCmdCycleTime_request(
    const std::shared_ptr<ConfigGetPDCmdCycleTime::Request> request,
    std::shared_ptr<ConfigGetPDCmdCycleTime::Response> response) {
  RCLCPP_INFO(this->get_logger(), "Received ConfigGetPDCmdCycleTime request!");

  response->result = FX_L1_Config_GetPDCmdCycleTime(&(response->cycle_time));

  RCLCPP_INFO(this->get_logger(),
              "Response ConfigGetPDCmdCycleTime: cycle-time = %d ,result = %d",
              response->cycle_time, response->result);
}

void BasicOrin::handle_ConfigResetAxisSensorOffset_request(
    const std::shared_ptr<ConfigResetAxisSensorOffset::Request> request,
    std::shared_ptr<ConfigResetAxisSensorOffset::Response> response) {
  RCLCPP_INFO(this->get_logger(),
              "Received ConfigResetAxisSensorOffset request!");

  response->result = FX_L1_Config_ResetAxisSensorOffset(
      static_cast<FXObjType>(request->obj_type), request->axis_id);

  RCLCPP_INFO(this->get_logger(),
              "Response ConfigResetAxisSensorOffset: result = %d",
              response->result);
}

void BasicOrin::handle_ConfigResetEncOffset_request(
    const std::shared_ptr<ConfigResetEncOffset::Request> request,
    std::shared_ptr<ConfigResetEncOffset::Response> response) {
  RCLCPP_INFO(this->get_logger(), "Received ConfigResetEncOffset request!");
  response->result = FX_L1_Config_ResetEncOffset(
      static_cast<FXObjType>(request->obj_type), request->axis_mask);
  if (response->result != FUNC_RET_SUCCESS) {
    printf(
        "Reset arm0's motor encoder and external encoder feedbacks to zero "
        "failed\n");
  }
  RCLCPP_INFO(this->get_logger(), "Response ConfigResetEncOffset: result = %d",
              response->result);
}

void BasicOrin::handle_ConfigResetSensorOffset_request(
    const std::shared_ptr<ConfigResetSensorOffset::Request> request,
    std::shared_ptr<ConfigResetSensorOffset::Response> response) {
  RCLCPP_INFO(this->get_logger(), "Received ConfigResetSensorOffset request!");
  response->result =
      FX_L1_Config_ResetSensorOffset(static_cast<FXObjType>(request->obj_type));
  if (response->result != FUNC_RET_SUCCESS) {
    printf("Clear arm%d's sensor offset to zero failed\n", request->obj_type);
  }
  RCLCPP_INFO(this->get_logger(),
              "Response ConfigResetSensorOffset: result = %d",
              response->result);
}

void BasicOrin::handle_ConfigSetBrakeLock_request(
    const std::shared_ptr<ConfigSetBrakeLock::Request> request,
    std::shared_ptr<ConfigSetBrakeLock::Response> response) {
  RCLCPP_INFO(this->get_logger(), "Received ConfigSetBrakeLock request!");

  response->result = FX_L1_Config_SetBrakeLock(
      static_cast<FXObjType>(request->obj_type), request->axis_mask);
  if (response->result != FUNC_RET_SUCCESS) {
    printf("Lock all brakes of arm%d failed\n", request->obj_type);
  }
  RCLCPP_INFO(this->get_logger(), "Response ConfigSetBrakeLock: result = %d",
              response->result);
}

void BasicOrin::handle_ConfigSetBrakeUnlock_request(
    const std::shared_ptr<ConfigSetBrakeUnlock::Request> request,
    std::shared_ptr<ConfigSetBrakeUnlock::Response> response) {
  RCLCPP_INFO(this->get_logger(), "Received ConfigSetBrakeUnlock request!");

  response->result = FX_L1_Config_SetBrakeUnlock(
      static_cast<FXObjType>(request->obj_type), request->axis_mask);
  if (response->result != FUNC_RET_SUCCESS) {
    printf("Unlock all brakes of arm%d failed\n", request->obj_type);
  }

  RCLCPP_INFO(this->get_logger(), "Response ConfigSetBrakeUnlock: result = %d",
              response->result);
}

void BasicOrin::handle_ConfigSetPDCmdCycleTime_request(
    const std::shared_ptr<ConfigSetPDCmdCycleTime::Request> request,
    std::shared_ptr<ConfigSetPDCmdCycleTime::Response> response) {
  RCLCPP_INFO(this->get_logger(), "Received ConfigSetPDCmdCycleTime request");

  response->result = FX_L1_Config_SetPDCmdCycleTime(request->cycle_time);
  if (response->result != FUNC_RET_SUCCESS) {
    printf("Failed to set PD command cycle time\n");
  }
  RCLCPP_INFO(this->get_logger(),
              "Response ConfigSetPDCmdCycleTime: result = %d",
              response->result);
}

void BasicOrin::handle_ConfigSetTraj_request(
    const std::shared_ptr<ConfigSetTraj::Request> request,
    std::shared_ptr<ConfigSetTraj::Response> response) {
  RCLCPP_INFO(this->get_logger(), "Received ConfigSetTraj request");

  response->result =
      FX_L1_Config_SetTraj(static_cast<FXObjType>(request->obj_type),
                           request->point_num, request->point_data.data());
  if (response->result != FUNC_RET_SUCCESS) {
    printf("Failed to set traj\n");
  } else {
    printf("Set traj success\n");
  }
  RCLCPP_INFO(this->get_logger(), "Response ConfigSetTraj: result = %d",
              response->result);
}

void BasicOrin::handle_FbkCheckUserDataSet_request(
    const std::shared_ptr<FbkCheckUserDataSet::Request> request,
    std::shared_ptr<FbkCheckUserDataSet::Response> response) {
  RCLCPP_INFO(this->get_logger(), "Received FbkCheckUserDataSet request");

  response->result = FX_L1_Fbk_CheckUserDataSet(request->user_data_len);
  if (response->result != FUNC_RET_SUCCESS) {
    printf("Invalid user data definition\n");
  }

  RCLCPP_INFO(this->get_logger(), "Response FbkCheckUserDataSet: result = %d",
              response->result);
}

void BasicOrin::handle_FbkGetCtrlObjDof_request(
    const std::shared_ptr<FbkGetCtrlObjDof::Request> request,
    std::shared_ptr<FbkGetCtrlObjDof::Response> response) {
  RCLCPP_INFO(this->get_logger(), "Received FbkGetCtrlObjDof request");

  response->result =
      FX_L1_Fbk_GetCtrlObjDof(static_cast<FXObjType>(request->obj_type));
  if (response->result == -1) {
    printf("Invalid user data definition\n");
  }

  RCLCPP_INFO(this->get_logger(), "Response FbkGetCtrlObjDof: result = %d",
              response->result);
}

void BasicOrin::handle_FbkGetCtrlObjPhysicalState_request(
    const std::shared_ptr<FbkGetCtrlObjPhysicalState::Request> request,
    std::shared_ptr<FbkGetCtrlObjPhysicalState::Response> response) {
  RCLCPP_INFO(this->get_logger(),
              "Received FbkGetCtrlObjPhysicalState request");

  response->result = FX_L1_Fbk_GetCtrlObjPhysicalState(
      static_cast<FXObjType>(request->obj_type), (&response->physical_state));

  RCLCPP_INFO(
      this->get_logger(),
      "Response FbkGetCtrlObjPhysicalState: physical_state = %d , result = %d",
      response->physical_state, response->result);
}

void BasicOrin::handle_FbkGetCtrlObjSensorInfo_request(
    const std::shared_ptr<FbkGetCtrlObjSensorInfo::Request> request,
    std::shared_ptr<FbkGetCtrlObjSensorInfo::Response> response) {
  RCLCPP_INFO(this->get_logger(), "Received FbkGetCtrlObjSensorInfo request");

  response->result = FX_L1_Fbk_GetCtrlObjSensorVersionAndSerial(
      static_cast<FXObjType>(request->obj_type), response->version.data(),
      response->serial.data());

  RCLCPP_INFO(this->get_logger(),
              "Response FbkGetCtrlObjSensorInfo: result = %d",
              response->result);
}

void BasicOrin::handle_FbkGetCtrlObjServoVersion_request(
    const std::shared_ptr<FbkGetCtrlObjServoVersion::Request> request,
    std::shared_ptr<FbkGetCtrlObjServoVersion::Response> response) {
  RCLCPP_INFO(this->get_logger(), "Received FbkGetCtrlObjServoVersion request");

  char firmware_buf[7][30] = {0};
  unsigned int config_buf[7] = {0};

  // 2. 调用底层函数
  int ret = FX_L1_Fbk_GetCtrlObjServoVersion(
      static_cast<FXObjType>(request->obj_type), firmware_buf, config_buf);

  // 3. 填回 response
  response->result = ret;
  if (ret == 0) {
    for (int i = 0; i < 7; ++i) {
      response->firmware_version[i] = std::string(firmware_buf[i]);
      response->config_version[i] = config_buf[i];
    }
  } else {
    // 错误处理，可清空响应字段
    for (int i = 0; i < 7; ++i) {
      response->firmware_version[i].clear();
      response->config_version[i] = 0;
    }
  }

  //   response->result = FX_L1_Fbk_GetCtrlObjServoVersion(
  //       static_cast<FXObjType>(request->obj_type),
  //       response->firmware_version.data(), response->config_version.data());

  RCLCPP_INFO(this->get_logger(),
              "Response FbkGetCtrlObjServoVersion: result = %d",
              response->result);
}

// void BasicOrin::handle_FbkGetSystemMsg_request(
//     const std::shared_ptr<FbkGetSystemMsg::Request> request,
//     std::shared_ptr<FbkGetSystemMsg::Response> response) {
//   RCLCPP_INFO(this->get_logger(), "Received FbkGetSystemMsg request");

//   response->result =
//   FX_L1_Fbk_GetSystemMsg(response->msg.c_str(),response->); if
//   (response->result > 0) {
//     printf("Receive message from controller: %s\n", response->msg.c_str());
//   }

//   RCLCPP_INFO(this->get_logger(), "Response FbkGetSystemMsg: result = %d",
//               response->result);
// }

void BasicOrin::handle_FbkGetUserData_request(
    const std::shared_ptr<FbkGetUserData::Request> request,
    std::shared_ptr<FbkGetUserData::Response> response) {
  RCLCPP_INFO(this->get_logger(), "Received FbkGetUserData request");

  constexpr size_t MAX_DATA_SIZE = 1024;
  std::vector<uint8_t> buffer(MAX_DATA_SIZE);

  FX_L1_Fbk_GetUserData(buffer.data());
  response->data_ptr.assign(buffer.begin(), buffer.end());
  size_t actual_len =
      strnlen(reinterpret_cast<const char *>(buffer.data()), MAX_DATA_SIZE);
  response->data_ptr.assign(buffer.begin(), buffer.begin() + actual_len);

  response->result = 1;

  RCLCPP_INFO(this->get_logger(), "Response FbkGetUserData: result = %d",
              response->result);
}

void BasicOrin::handle_FbkRegisterUserDataSet_request(
    const std::shared_ptr<FbkRegisterUserDataSet::Request> request,
    std::shared_ptr<FbkRegisterUserDataSet::Response> response) {
  RCLCPP_INFO(this->get_logger(), "Received FbkRegisterUserDataSet request");

  response->result = FX_L1_Fbk_RegisterUserDataSet(
      const_cast<char *>(request->name.c_str()),
      static_cast<FXUserDataType>(request->data_type), request->sub,
      request->data_num);
  if (response->result != FUNC_RET_SUCCESS) {
    printf("Failed to register user data\n");
  }

  RCLCPP_INFO(this->get_logger(),
              "Response FbkRegisterUserDataSet: result = %d", response->result);
}

void BasicOrin::handle_FbkResetSystemMsg_request(
    const std::shared_ptr<FbkResetSystemMsg::Request> request,
    std::shared_ptr<FbkResetSystemMsg::Response> response) {
  RCLCPP_INFO(this->get_logger(), "Received FbkResetSystemMsg request");

  response->result = 1;
  FX_L1_Fbk_ResetSystemMsg();
  RCLCPP_INFO(this->get_logger(), "Response FbkResetSystemMsg: result = %d",
              response->result);
}

void BasicOrin::handle_FbkResetUserDataSet_request(
    const std::shared_ptr<FbkResetUserDataSet::Request> request,
    std::shared_ptr<FbkResetUserDataSet::Response> response) {
  RCLCPP_INFO(this->get_logger(), "Received FbkResetUserDataSet request");

  response->result = 1;
  FX_L1_Fbk_ResetUserDataSet();

  RCLCPP_INFO(this->get_logger(), "Response FbkResetUserDataSet: result = %d",
              response->result);
}

void BasicOrin::handle_ParamGetFloat_request(
    const std::shared_ptr<ParamGetFloat::Request> request,
    std::shared_ptr<ParamGetFloat::Response> response) {
  RCLCPP_INFO(this->get_logger(), "Received ParamGetFloat request");

  response->result = FX_L1_Param_GetFloat(
      const_cast<char *>(request->name.c_str()), &(response->value));
  if (response->result != FUNC_RET_SUCCESS) {
    printf("Failed to get float parameter: %s\n", request->name.c_str());
  }

  RCLCPP_INFO(this->get_logger(),
              "Response ParamGetFloat: %s = %f , result = %d",
              request->name.c_str(), response->value, response->result);
}

void BasicOrin::handle_ParamGetInt32_request(
    const std::shared_ptr<ParamGetInt32::Request> request,
    std::shared_ptr<ParamGetInt32::Response> response) {
  RCLCPP_INFO(this->get_logger(), "Received ParamGetInt32 request");

  response->result = FX_L1_Param_GetInt32(
      const_cast<char *>(request->name.c_str()), &(response->value));
  if (response->result != FUNC_RET_SUCCESS) {
    printf("Failed to get int parameter: %s\n", request->name.c_str());
  }

  RCLCPP_INFO(this->get_logger(),
              "Response ParamGetInt32: %s = %d , result = %d",
              request->name.c_str(), response->value, response->result);
}

void BasicOrin::handle_ParamGetString_request(
    const std::shared_ptr<ParamGetString::Request> request,
    std::shared_ptr<ParamGetString::Response> response) {
  RCLCPP_INFO(this->get_logger(), "Received ParamGetString request");

  response->result =
      FX_L1_Param_GetString(const_cast<char *>(request->name.c_str()),
                            const_cast<char *>(response->value.c_str()));
  if (response->result != FUNC_RET_SUCCESS) {
    printf("Failed to get string parameter: %s\n", request->name.c_str());
  }

  RCLCPP_INFO(this->get_logger(),
              "Response ParamGetString: %s = %s , result = %d",
              request->name.c_str(), response->value, response->result);
}

void BasicOrin::handle_ParamSetFloat_request(
    const std::shared_ptr<ParamSetFloat::Request> request,
    std::shared_ptr<ParamSetFloat::Response> response) {
  RCLCPP_INFO(this->get_logger(), "Received ParamSetFloat request");

  response->result = FX_L1_Param_SetFloat(
      const_cast<char *>(request->name.c_str()), request->value);
  if (response->result != FUNC_RET_SUCCESS) {
    printf("Failed to set float parameter %s\n", request->name.c_str());
  }

  RCLCPP_INFO(this->get_logger(), "Response ParamSetFloat: result = %d",
              response->result);
}

void BasicOrin::handle_ParamSetInt32_request(
    const std::shared_ptr<ParamSetInt32::Request> request,
    std::shared_ptr<ParamSetInt32::Response> response) {
  RCLCPP_INFO(this->get_logger(), "Received ParamSetInt32 request");

  response->result = FX_L1_Param_SetInt32(
      const_cast<char *>(request->name.c_str()), request->value);
  if (response->result != FUNC_RET_SUCCESS) {
    printf("Failed to set int parameter %s\n", request->name.c_str());
  }

  RCLCPP_INFO(this->get_logger(), "Response ParamSetInt32: result = %d",
              response->result);
}

void BasicOrin::handle_RuntimeEmergencyStop_request(
    const std::shared_ptr<RuntimeEmergencyStop::Request> request,
    std::shared_ptr<RuntimeEmergencyStop::Response> response) {
  RCLCPP_INFO(this->get_logger(), "Received ParamSetInt32 request");

  response->stopped_mask =
      FX_L1_Runtime_EmergencyStop(request->thread_id, request->obj_mask);
  if (response->stopped_mask != request->obj_mask) {
    printf("Failed to trigger %d's emergency stop\n", response->stopped_mask);
  }

  RCLCPP_INFO(this->get_logger(), "Response ParamSetInt32: stopped_mask=%d",
              response->stopped_mask);
}

void BasicOrin::handle_RuntimeRunTraj_request(
    const std::shared_ptr<RuntimeRunTraj::Request> request,
    std::shared_ptr<RuntimeRunTraj::Response> response) {
  RCLCPP_INFO(this->get_logger(), "Received RuntimeRunTraj request");

  unsigned int started_mask = 0;

  /* Request runtime execution of the trajectory that has already been uploaded
   * to the controller. */
  started_mask = FX_L1_Runtime_RunTraj(request->thread_id, request->obj_mask);
  if ((started_mask & request->obj_mask) != request->obj_mask) {
    printf("Failed to run trajectory, returned mask = 0x%08x\n", started_mask);
    response->result = -1;
  }
  response->result = 0;

  RCLCPP_INFO(this->get_logger(), "Response RuntimeRunTraj: result = %d",
              response->result);
}

void BasicOrin::handle_RuntimeSetAccRatio_request(
    const std::shared_ptr<RuntimeSetAccRatio::Request> request,
    std::shared_ptr<RuntimeSetAccRatio::Response> response) {
  RCLCPP_INFO(this->get_logger(), "Received RuntimeSetAccRatio request");

  response->result = FX_L1_Runtime_SetAccRatio(
      request->thread_id, static_cast<FXObjType>(request->obj_type),
      request->acc_ratio);

  RCLCPP_INFO(this->get_logger(), "Response RuntimeSetAccRatio: result = %d",
              response->result);
}

void BasicOrin::handle_RuntimeSetBodyPDD_request(
    const std::shared_ptr<RuntimeSetBodyPDD::Request> request,
    std::shared_ptr<RuntimeSetBodyPDD::Response> response) {
  RCLCPP_INFO(this->get_logger(), "Received RuntimeSetBodyPDD request");

  response->result =
      FX_L1_Runtime_SetBodyPDD(request->thread_id, request->d.data());

  RCLCPP_INFO(this->get_logger(), "Response RuntimeSetBodyPDD: result = %d",
              response->result);
}

void BasicOrin::handle_RuntimeSetBodyPDP_request(
    const std::shared_ptr<RuntimeSetBodyPDP::Request> request,
    std::shared_ptr<RuntimeSetBodyPDP::Response> response) {
  RCLCPP_INFO(this->get_logger(), "Received RuntimeSetBodyPDP request");

  response->result =
      FX_L1_Runtime_SetBodyPDP(request->thread_id, request->p.data());

  RCLCPP_INFO(this->get_logger(), "Response RuntimeSetBodyPDP: result = %d",
              response->result);
}

void BasicOrin::handle_RuntimeSetBodyPD_request(
    const std::shared_ptr<RuntimeSetBodyPD::Request> request,
    std::shared_ptr<RuntimeSetBodyPD::Response> response) {
  RCLCPP_INFO(this->get_logger(), "Received RuntimeSetBodyPD request");

  response->result = FX_L1_Runtime_SetBodyPD(
      request->thread_id, request->p.data(), request->d.data());

  RCLCPP_INFO(this->get_logger(), "Response RuntimeSetBodyPD: result = %d",
              response->result);
}

void BasicOrin::handle_RuntimeSetCartKD_request(
    const std::shared_ptr<RuntimeSetCartKD::Request> request,
    std::shared_ptr<RuntimeSetCartKD::Response> response) {
  RCLCPP_INFO(this->get_logger(), "Received RuntimeSetCartKD request");

  response->result = FX_L1_Runtime_SetCartKD(
      request->thread_id, static_cast<FXObjType>(request->obj_type),
      request->k.data(), request->d.data());

  RCLCPP_INFO(this->get_logger(), "Response RuntimeSetCartKD: result = %d",
              response->result);
}

void BasicOrin::handle_RuntimeSetCartK_request(
    const std::shared_ptr<RuntimeSetCartK::Request> request,
    std::shared_ptr<RuntimeSetCartK::Response> response) {
  RCLCPP_INFO(this->get_logger(), "Received RuntimeSetCartK request");

  response->result = FX_L1_Runtime_SetCartK(
      request->thread_id, static_cast<FXObjType>(request->obj_type),
      request->k.data());

  RCLCPP_INFO(this->get_logger(), "Response RuntimeSetCartK: result = %d",
              response->result);
}

void BasicOrin::handle_RuntimeSetForceCtrl_request(
    const std::shared_ptr<RuntimeSetForceCtrl::Request> request,
    std::shared_ptr<RuntimeSetForceCtrl::Response> response) {
  RCLCPP_INFO(this->get_logger(), "Received RuntimeSetForceCtrl request");

  response->result = FX_L1_Runtime_SetForceCtrl(
      request->thread_id, static_cast<FXObjType>(request->obj_type),
      request->force_ctrl.data());

  RCLCPP_INFO(this->get_logger(), "Response RuntimeSetForceCtrl: result = %d",
              response->result);
}

void BasicOrin::handle_RuntimeSetRefOri_request(
    const std::shared_ptr<RuntimeSetRefOri::Request> request,
    std::shared_ptr<RuntimeSetRefOri::Response> response) {
  RCLCPP_INFO(this->get_logger(), "Received RuntimeSetRefOri request");

  response->result = FX_L1_Runtime_SetRefOri(
      request->thread_id, static_cast<FXObjType>(request->obj_type),
      static_cast<FXRefOriType>(request->ref_ori_type),
      request->ref_ori.data());

  RCLCPP_INFO(this->get_logger(), "Response RuntimeSetRefOri: result = %d",
              response->result);
}

void BasicOrin::handle_StateGetServoErrorCode_request(
    const std::shared_ptr<StateGetServoErrorCode::Request> request,
    std::shared_ptr<StateGetServoErrorCode::Response> response) {
  RCLCPP_INFO(this->get_logger(), "Received StateGetServoErrorCode request");

  response->result = FX_L1_State_GetServoErrorCode(
      static_cast<FXObjType>(request->obj_type), response->error_codes.data());
  if (response->result != FUNC_RET_SUCCESS) {
    printf("Failed to get servo errorcode for %d\n", request->obj_type);
  }

  RCLCPP_INFO(this->get_logger(),
              "Response StateGetServoErrorCode: result = %d", response->result);
}

void BasicOrin::handle_StateResetError_request(
    const std::shared_ptr<StateResetError::Request> request,
    std::shared_ptr<StateResetError::Response> response) {
  RCLCPP_INFO(this->get_logger(), "Received StateResetError request");

  response->result = 0;
  if (FX_L1_State_ResetError(static_cast<FXObjType>(request->obj_type),
                             request->timeout, &(response->system_errorcode)) !=
      FUNC_RET_SUCCESS) {
    printf("Failed to reset arm0 error, errorcode = 0x%08x\n",
           response->system_errorcode);

    response->result = -1;
  }

  RCLCPP_INFO(this->get_logger(), "Response StateResetError: result = %d",
              response->result);
}

void BasicOrin::handle_StateSwitchToCollaborativeRelease_request(
    const std::shared_ptr<StateSwitchToCollaborativeRelease::Request> request,
    std::shared_ptr<StateSwitchToCollaborativeRelease::Response> response) {
  RCLCPP_INFO(this->get_logger(),
              "Received StateSwitchToCollaborativeRelease request");

  response->result = FX_L1_State_SwitchToCollaborativeRelease(
      static_cast<FXObjType>(request->obj_type), request->timeout);
  if (response->result != FUNC_RET_SUCCESS) {
    printf("Failed to transfer %d to STATE_CR state\n", request->obj_type);
  }

  RCLCPP_INFO(this->get_logger(),
              "Response StateSwitchToCollaborativeRelease: result = %d",
              response->result);
}

void BasicOrin::handle_StateSwitchToDragCartR_request(
    const std::shared_ptr<StateSwitchToDragCartR::Request> request,
    std::shared_ptr<StateSwitchToDragCartR::Response> response) {
  RCLCPP_INFO(this->get_logger(), "Received StateSwitchToDragCartR request");

  response->result = FX_L1_State_SwitchToDragCartR(
      static_cast<FXObjType>(request->obj_type), request->timeout,
      static_cast<FXRefOriType>(request->ref_ori_type), request->ref_ori.data(),
      request->k.data(), request->d.data());

  RCLCPP_INFO(this->get_logger(),
              "Response StateSwitchToDragCartR: result = %d", response->result);
}

void BasicOrin::handle_StateSwitchToDragCartX_request(
    const std::shared_ptr<StateSwitchToDragCartX::Request> request,
    std::shared_ptr<StateSwitchToDragCartX::Response> response) {
  RCLCPP_INFO(this->get_logger(), "Received StateSwitchToDragCartX request");

  response->result = FX_L1_State_SwitchToDragCartX(
      static_cast<FXObjType>(request->obj_type), request->timeout,
      static_cast<FXRefOriType>(request->ref_ori_type), request->ref_ori.data(),
      request->k.data(), request->d.data());

  RCLCPP_INFO(this->get_logger(),
              "Response StateSwitchToDragCartX: result = %d", response->result);
}

void BasicOrin::handle_StateSwitchToDragCartY_request(
    const std::shared_ptr<StateSwitchToDragCartY::Request> request,
    std::shared_ptr<StateSwitchToDragCartY::Response> response) {
  RCLCPP_INFO(this->get_logger(), "Received StateSwitchToDragCartY request");

  response->result = FX_L1_State_SwitchToDragCartY(
      static_cast<FXObjType>(request->obj_type), request->timeout,
      static_cast<FXRefOriType>(request->ref_ori_type), request->ref_ori.data(),
      request->k.data(), request->d.data());

  RCLCPP_INFO(this->get_logger(),
              "Response StateSwitchToDragCartY: result = %d", response->result);
}

void BasicOrin::handle_StateSwitchToDragCartZ_request(
    const std::shared_ptr<StateSwitchToDragCartZ::Request> request,
    std::shared_ptr<StateSwitchToDragCartZ::Response> response) {
  RCLCPP_INFO(this->get_logger(), "Received StateSwitchToDragCartZ request");

  FX_L1_State_SwitchToDragCartZ(
      static_cast<FXObjType>(request->obj_type), request->timeout,
      static_cast<FXRefOriType>(request->ref_ori_type), request->ref_ori.data(),
      request->k.data(), request->d.data());

  RCLCPP_INFO(this->get_logger(),
              "Response StateSwitchToDragCartZ: result = %d", response->result);
}

void BasicOrin::handle_StateSwitchToDragJoint_request(
    const std::shared_ptr<StateSwitchToDragJoint::Request> request,
    std::shared_ptr<StateSwitchToDragJoint::Response> response) {
  RCLCPP_INFO(this->get_logger(), "Received StateSwitchToDragJoint request");

  response->result = FX_L1_State_SwitchToDragJoint(
      static_cast<FXObjType>(request->obj_type), request->timeout,
      request->k.data(), request->d.data());
  if (response->result != FUNC_RET_SUCCESS) {
    printf("Failed to transfer %d to STATE_DRAG_JOINT state\n",
           request->obj_type);
  }

  RCLCPP_INFO(this->get_logger(),
              "Response StateSwitchToDragJoint: result = %d", response->result);
}

void BasicOrin::handle_StateSwitchToIdle_request(
    const std::shared_ptr<StateSwitchToIdle::Request> request,
    std::shared_ptr<StateSwitchToIdle::Response> response) {
  RCLCPP_INFO(this->get_logger(), "Received StateSwitchToIdle request");

  response->result = 0;
  if (FX_L1_State_SwitchToIdle(static_cast<FXObjType>(request->obj_type),
                               request->timeout) != FUNC_RET_SUCCESS) {
    printf("Failed to switch %d to IDLE state\n", request->obj_type);
    response->result = -1;
  }

  RCLCPP_INFO(this->get_logger(), "Response StateSwitchToIdle: result = %d",
              response->result);
}

void BasicOrin::handle_StateSwitchToImpCart_request(
    const std::shared_ptr<StateSwitchToImpCart::Request> request,
    std::shared_ptr<StateSwitchToImpCart::Response> response) {
  RCLCPP_INFO(this->get_logger(), "Received StateSwitchToImpCart request");
  response->result = FX_L1_State_SwitchToImpCartMode(
      static_cast<FXObjType>(request->obj_type), request->timeout,
      static_cast<FXRefOriType>(request->ref_ori_type), request->ref_ori.data(),
      request->vel_ratio, request->acc_ratio, request->k.data(),
      request->d.data());
  if (response->result != FUNC_RET_SUCCESS) {
    printf("Failed to switch %d to IDLE state\n", request->obj_type);
  }

  RCLCPP_INFO(this->get_logger(), "Response StateSwitchToImpCart: result = %d",
              response->result);
}

void BasicOrin::handle_StateSwitchToImpForce_request(
    const std::shared_ptr<StateSwitchToImpForce::Request> request,
    std::shared_ptr<StateSwitchToImpForce::Response> response) {
  RCLCPP_INFO(this->get_logger(), "Received StateSwitchToImpForce request");
  response->result = FX_L1_State_SwitchToImpForceMode(
      static_cast<FXObjType>(request->obj_type), request->timeout,
      static_cast<FXRefOriType>(request->ref_ori_type), request->ref_ori.data(),
      request->force_ctrl.data(), request->torque_ctrl.data());

  RCLCPP_INFO(this->get_logger(), "Response StateSwitchToImpForce: result = %d",
              response->result);
}

void BasicOrin::handle_StateSwitchToImpJoint_request(
    const std::shared_ptr<StateSwitchToImpJoint::Request> request,
    std::shared_ptr<StateSwitchToImpJoint::Response> response) {
  RCLCPP_INFO(this->get_logger(), "Received StateSwitchToImpJoint request");
  response->result = FX_L1_State_SwitchToImpJointMode(
      static_cast<FXObjType>(request->obj_type), request->timeout,
      request->vel_ratio, request->acc_ratio, request->k.data(),
      request->d.data());

  RCLCPP_INFO(this->get_logger(), "Response StateSwitchToImpJoint: result = %d",
              response->result);
}

void BasicOrin::handle_StateSwitchToMIT_request(
    const std::shared_ptr<StateSwitchToMIT::Request> request,
    std::shared_ptr<StateSwitchToMIT::Response> response) {
  RCLCPP_INFO(this->get_logger(), "Received StateSwitchToMIT request");
  response->result = FX_L1_State_SwitchToMITMode(
      static_cast<FXObjType>(request->obj_type), request->timeout,
      request->k.data(), request->d.data());

  RCLCPP_INFO(this->get_logger(), "Response StateSwitchToMIT: result = %d",
              response->result);
}

void BasicOrin::handle_StateSwitchToPD_request(
    const std::shared_ptr<StateSwitchToPD::Request> request,
    std::shared_ptr<StateSwitchToPD::Response> response) {
  RCLCPP_INFO(this->get_logger(), "Received StateSwitchToPD request");

  response->result = FX_L1_State_SwitchToPDMode(
      static_cast<FXObjType>(request->obj_type), request->timeout,
      request->vel_ratio, request->acc_ratio, request->k.data(),
      request->d.data());

  if (response->result != FUNC_RET_SUCCESS) {
    printf("Failed to switch %d to STATE_PD\n", request->obj_type);
  }

  RCLCPP_INFO(this->get_logger(), "Response StateSwitchToPD: result = %d",
              response->result);
}

void BasicOrin::handle_StateSwitchToPosition_request(
    const std::shared_ptr<StateSwitchToPosition::Request> request,
    std::shared_ptr<StateSwitchToPosition::Response> response) {
  RCLCPP_INFO(this->get_logger(), "Received StateSwitchToPosition request");

  response->result = FX_L1_State_SwitchToPositionMode(
      static_cast<FXObjType>(request->obj_type), request->timeout,
      request->vel_ratio, request->acc_ratio);

  if (response->result != FUNC_RET_SUCCESS) {
    printf("Failed to transfer %d to STATE_POSITION state\n",
           request->obj_type);
  }

  RCLCPP_INFO(this->get_logger(), "Response StateSwitchToPosition: result = %d",
              response->result);
}

void BasicOrin::handle_StateSwitchToUserMIT_request(
    const std::shared_ptr<StateSwitchToUserMIT::Request> request,
    std::shared_ptr<StateSwitchToUserMIT::Response> response) {
  RCLCPP_INFO(this->get_logger(), "Received StateSwitchToUserMIT request");

  response->result = FX_L1_State_SwitchToUserMITMode(
      static_cast<FXObjType>(request->obj_type), request->timeout,
      request->pos.data(), request->vel.data(), request->tor.data(),
      request->k.data(), request->d.data());

  RCLCPP_INFO(this->get_logger(), "Response StateSwitchToUserMIT: result = %d",
              response->result);
}

void BasicOrin::handle_SystemGetControllerVersion_request(
    const std::shared_ptr<SystemGetControllerVersion::Request> request,
    std::shared_ptr<SystemGetControllerVersion::Response> response) {
  RCLCPP_INFO(this->get_logger(),
              "Received SystemGetControllerVersion request");

  response->controller_version = FX_L1_System_GetControllerVersion();
  response->result = 0;
  RCLCPP_INFO(this->get_logger(),
              "Response SystemGetControllerVersion: Controller version is "
              "0x%08x, result = %d",
              response->controller_version, response->result);
}

void BasicOrin::handle_SystemGetIP_request(
    const std::shared_ptr<SystemGetIP::Request> request,
    std::shared_ptr<SystemGetIP::Response> response) {
  RCLCPP_INFO(this->get_logger(), "Received SystemGetIP request");
  char ip_buffer[32] = {0};  // 足够容纳 IP 地址（含 '\0'）

  // 2. 调用底层函数（假设原型为 void FX_L1_System_GetSystemIP(char* ip)）

  response->result = FX_L1_System_GetSystemIP(ip_buffer);
  // 3. 将结果填入响应
  if (FUNC_RET_SUCCESS == response->result) {
    response->ip_str = std::string(ip_buffer);

    RCLCPP_INFO(this->get_logger(),
                "Response SystemGetIP: IP is %s, result = %d",
                response->ip_str.c_str(), response->result);
  } else {
    RCLCPP_INFO(this->get_logger(), "Response SystemGetIP: result = %d",
                response->result);
  }
}

void BasicOrin::handle_SystemGetLinkState_request(
    const std::shared_ptr<SystemGetLinkState::Request> request,
    std::shared_ptr<SystemGetLinkState::Response> response) {
  RCLCPP_INFO(this->get_logger(), "Received SystemGetLinkState request");

  response->link_state = FX_L1_System_GetLinkState();
  if (response->link_state == 0) {
    printf("Link is not established\n");
  } else if (response->link_state == -1) {
    printf("Link is established, but no data arrived in 100ms\n");
  } else if (response->link_state == 1) {
    printf("Link is established\n");
  } else {
    printf("Link state is unknown(%d)\n", response->link_state);
  }

  RCLCPP_INFO(this->get_logger(),
              "Response SystemGetLinkState: link_state = %d",
              response->link_state);
}

void BasicOrin::handle_SystemGetLogLevel_request(
    const std::shared_ptr<SystemGetLogLevel::Request> request,
    std::shared_ptr<SystemGetLogLevel::Response> response) {
  RCLCPP_INFO(this->get_logger(), "Received SystemGetLogLevel request");

  response->result = FX_L1_System_GetLogLevel();

  RCLCPP_INFO(this->get_logger(), "Response SystemGetLogLevel: result = %d",
              response->result);
}

void BasicOrin::handle_SystemGetRobotType_request(
    const std::shared_ptr<SystemGetRobotType::Request> request,
    std::shared_ptr<SystemGetRobotType::Response> response) {
  RCLCPP_INFO(this->get_logger(), "Received SystemGetRobotType request");

  response->robot_type = static_cast<FXRobotType>(FX_L1_Fbk_GetRobotType());
  response->result = 0;
  RCLCPP_INFO(this->get_logger(), "Response SystemGetRobotType: result = %d",
              response->result);
}

void BasicOrin::handle_SystemGetSDKVersion_request(
    const std::shared_ptr<SystemGetSDKVersion::Request> request,
    std::shared_ptr<SystemGetSDKVersion::Response> response) {
  RCLCPP_INFO(this->get_logger(), "Received SystemGetSDKVersion request");

  //
  response->sdk_version = FX_L1_System_GetSDKVersion();
  response->result = 0;
  RCLCPP_INFO(
      this->get_logger(),
      "Response SystemGetSDKVersion: SDK version is 0x%08x, result = %d",
      response->sdk_version, response->result);
}

void BasicOrin::handle_SystemGetTime_request(
    const std::shared_ptr<SystemGetTime::Request> request,
    std::shared_ptr<SystemGetTime::Response> response) {
  RCLCPP_INFO(this->get_logger(), "Received SystemGetTime request");

  response->result = FX_L1_System_GetSystemTime(
      const_cast<char *>(response->time_str.c_str()));

  RCLCPP_INFO(this->get_logger(),
              "Response SystemGetTime: time_str is %s, result = %d",
              response->time_str.c_str(), response->result);
}

void BasicOrin::handle_SystemLink_request(
    const std::shared_ptr<SystemLink::Request> request,
    std::shared_ptr<SystemLink::Response> response) {
  RCLCPP_INFO(this->get_logger(), "Received SystemLink request");
  // 按照规则计算 latency = log_level * 10 + 1
  response->result = FX_L1_System_Link(request->ip1, request->ip2, request->ip3,
                                       request->ip4, request->log_level);
  if (response->result < 0) {
    printf("Failed to link system\n");
  } else {
    setPublishEnabled(true);
  }
  RCLCPP_INFO(this->get_logger(), "Response: latency = %d, result = %d",
              response->latency, response->result);
}

void BasicOrin::handle_SystemReboot_request(
    const std::shared_ptr<SystemReboot::Request> request,
    std::shared_ptr<SystemReboot::Response> response) {
  RCLCPP_INFO(this->get_logger(), "Received SystemReboot request");

  response->result = FX_L1_System_Reboot();

  RCLCPP_INFO(this->get_logger(), "Response SystemReboot: result = %d",
              response->result);
}

void BasicOrin::handle_SystemRecvFile_request(
    const std::shared_ptr<SystemRecvFile::Request> request,
    std::shared_ptr<SystemRecvFile::Response> response) {
  RCLCPP_INFO(this->get_logger(), "Received SystemRecvFile request");

  response->result = FX_L1_System_RecvFile(
      const_cast<char *>(request->local_file_path.c_str()),
      const_cast<char *>(request->remote_file_path.c_str()));

  RCLCPP_INFO(this->get_logger(), "Response SystemRecvFile: result = %d",
              response->result);
}

void BasicOrin::handle_SystemRestoreImage_request(
    const std::shared_ptr<SystemRestoreImage::Request> request,
    std::shared_ptr<SystemRestoreImage::Response> response) {
  RCLCPP_INFO(this->get_logger(), "Received SystemRestoreImage request");

  response->result = FX_L1_System_RestoreImage();

  RCLCPP_INFO(this->get_logger(), "Response SystemRestoreImage: result = %d",
              response->result);
}

void BasicOrin::handle_SystemSendFile_request(
    const std::shared_ptr<SystemSendFile::Request> request,
    std::shared_ptr<SystemSendFile::Response> response) {
  RCLCPP_INFO(this->get_logger(), "Received SystemSendFile request");

  response->result = FX_L1_System_SendFile(
      const_cast<char *>(request->local_file_path.c_str()),
      const_cast<char *>(request->remote_file_path.c_str()));
  if (response->result != FUNC_RET_SUCCESS) {
    printf("Failed to transfer %s to remote path\n",
           request->local_file_path.c_str());
  } else {
    printf("Transfer %s to remote path success\n",
           request->local_file_path.c_str());
  }

  RCLCPP_INFO(this->get_logger(), "Response SystemSendFile: result = %d",
              response->result);
}

void BasicOrin::handle_SystemSetIP_request(
    const std::shared_ptr<SystemSetIP::Request> request,
    std::shared_ptr<SystemSetIP::Response> response) {
  RCLCPP_INFO(this->get_logger(), "Received SystemSetIP request");

  response->result = FX_L1_System_SetSystemIP(request->ip.data());

  RCLCPP_INFO(this->get_logger(), "Response SystemSetIP: result = %d",
              response->result);
}

void BasicOrin::handle_SystemSetLogLevel_request(
    const std::shared_ptr<SystemSetLogLevel::Request> request,
    std::shared_ptr<SystemSetLogLevel::Response> response) {
  RCLCPP_INFO(this->get_logger(), "Received SystemSetLogLevel request");

  FX_L1_System_SetLogLevel(request->log_level);
  response->result = 0;
  RCLCPP_INFO(this->get_logger(), "Response SystemSetLogLevel: result = %d",
              response->result);
}

void BasicOrin::handle_SystemSetTime_request(
    const std::shared_ptr<SystemSetTime::Request> request,
    std::shared_ptr<SystemSetTime::Response> response) {
  RCLCPP_INFO(this->get_logger(), "Received SystemSetTime request");

  response->result = FX_L1_System_SetSystemTime(
      request->year, request->month, request->day, request->hour,
      request->minute, request->second);

  RCLCPP_INFO(this->get_logger(), "Response SystemSetTime: result = %d",
              response->result);
}

void BasicOrin::handle_SystemSetUserFbkType_request(
    const std::shared_ptr<SystemSetUserFbkType::Request> request,
    std::shared_ptr<SystemSetUserFbkType::Response> response) {
  RCLCPP_INFO(this->get_logger(), "Received SystemSetUserFbkType request");

  response->result =
      FX_L1_System_SetUserFbkType(request->chn_id, request->fbk_type);

  RCLCPP_INFO(this->get_logger(), "Response SystemSetUserFbkType: result = %d",
              response->result);
}

void BasicOrin::handle_SystemUnlink_request(
    const std::shared_ptr<SystemUnlink::Request> request,
    std::shared_ptr<SystemUnlink::Response> response) {
  RCLCPP_INFO(this->get_logger(), "Received SystemUnlink request");

  FX_L1_System_Unlink();
  setPublishEnabled(false);
  response->result = 0;
  RCLCPP_INFO(this->get_logger(), "Response SystemUnlink: result = %d",
              response->result);
}

void BasicOrin::handle_SystemUpdate_request(
    const std::shared_ptr<SystemUpdate::Request> request,
    std::shared_ptr<SystemUpdate::Response> response) {
  RCLCPP_INFO(this->get_logger(), "Received SystemUpdate request");

  response->result =
      FX_L1_System_Update(const_cast<char *>(request->update_file_path.c_str()),
                          const_cast<char *>(request->ini_file_path.c_str()));
  RCLCPP_INFO(this->get_logger(), "Response SystemUpdate: result = %d",
              response->result);
}

void BasicOrin::handle_TerminalCH485GetData_request(
    const std::shared_ptr<TerminalCH485GetData::Request> request,
    std::shared_ptr<TerminalCH485GetData::Response> response) {
  RCLCPP_INFO(this->get_logger(), "Received TerminalCH485GetData request");

  FXChnType resChnType;

  response->result = FX_L1_Terminal_GetData(
      static_cast<FXObjType>(request->terminal_type), request->timeout,
      &resChnType, response->data.data(), &(response->receiving_time));
  response->chn_type = resChnType;
  RCLCPP_INFO(this->get_logger(), "Response TerminalCH485GetData: result = %d",
              response->result);
}

void BasicOrin::handle_TerminalCH485SetData_request(
    const std::shared_ptr<TerminalCH485SetData::Request> request,
    std::shared_ptr<TerminalCH485SetData::Response> response) {
  RCLCPP_INFO(this->get_logger(), "Received TerminalCH485SetData request");

  response->result = FX_L1_Terminal_SetData(
      static_cast<FXObjType>(request->terminal_type),
      static_cast<FXChnType>(request->chn_type), request->timeout,
      request->data.data(), request->data_len, (&response->sending_time));

  RCLCPP_INFO(this->get_logger(), "Response TerminalCH485SetData: result = %d",
              response->result);
}

void BasicOrin::handle_TerminalClearData_request(
    const std::shared_ptr<TerminalClearData::Request> request,
    std::shared_ptr<TerminalClearData::Response> response) {
  RCLCPP_INFO(this->get_logger(), "Received TerminalClearData request");

  response->result =
      FX_L1_Terminal_ClearData(static_cast<FXObjType>(request->terminal_type));
  if (response->result != FUNC_RET_SUCCESS) {
    printf("Failed to get ready for transferring %d's terminal data...\n",
           request->terminal_type);
  }

  RCLCPP_INFO(this->get_logger(), "Response TerminalClearData: result = %d",
              response->result);
}

// TOPIC
// -------------------------------------------------------------------------------
void BasicOrin::handle_RuntimeStopTraj_callback(const RuntimeStopTraj &msg) {
  (void)msg;  // 避免未使用参数警告
  RCLCPP_INFO(this->get_logger(), "RuntimeStopTraj callback called");
  // TODO: 实现停止轨迹逻辑
}

void BasicOrin::handle_RuntimeSetVelRatio_callback(
    const RuntimeSetVelRatio &msg) {
  (void)msg;
  RCLCPP_INFO(this->get_logger(), "RuntimeSetVelRatio callback called");
  // TODO: 实现速度比例设置
}

void BasicOrin::handle_RuntimeSetTorqueCtrl_callback(
    const RuntimeSetTorqueCtrl &msg) {
  (void)msg;
  RCLCPP_INFO(this->get_logger(), "RuntimeSetTorqueCtrl callback called");
  // TODO: 实现力矩控制设置
}

void BasicOrin::handle_RuntimeSetToolKD_callback(const RuntimeSetToolKD &msg) {
  (void)msg;
  RCLCPP_INFO(this->get_logger(), "RuntimeSetToolKD callback called");
  // TODO: 实现工具KD参数设置
}

void BasicOrin::handle_RuntimeSetToolK_callback(const RuntimeSetToolK &msg) {
  (void)msg;
  RCLCPP_INFO(this->get_logger(), "RuntimeSetToolK callback called");
  // TODO: 实现工具K参数设置
}

void BasicOrin::handle_RuntimeSetToolD_callback(const RuntimeSetToolD &msg) {
  (void)msg;
  RCLCPP_INFO(this->get_logger(), "RuntimeSetToolD callback called");
  // TODO: 实现工具D参数设置
}

void BasicOrin::handle_RuntimeSetTag_callback(const RuntimeSetTag &msg) {
  (void)msg;
  RCLCPP_INFO(this->get_logger(), "RuntimeSetTag callback called");
  // TODO: 实现标签设置
}

void BasicOrin::handle_RuntimeSetSpeedRatio_callback(
    const RuntimeSetSpeedRatio &msg) {
  (void)msg;
  RCLCPP_INFO(this->get_logger(), "RuntimeSetSpeedRatio callback called");
  // TODO: 实现速度倍率设置
}

void BasicOrin::handle_RuntimeSetJointMITCmd_callback(
    const RuntimeSetJointMITCmd &msg) {
  (void)msg;
  RCLCPP_INFO(this->get_logger(), "RuntimeSetJointMITCmd callback called");
  // TODO: 实现MIT关节命令设置
}

void BasicOrin::handle_RuntimeSetJointKD_callback(
    const RuntimeSetJointKD &msg) {
  (void)msg;
  RCLCPP_INFO(this->get_logger(), "RuntimeSetJointKD callback called");
  // TODO: 实现关节KD参数设置
}

void BasicOrin::handle_RuntimeSetJointK_callback(const RuntimeSetJointK &msg) {
  (void)msg;
  RCLCPP_INFO(this->get_logger(), "RuntimeSetJointK callback called");
  // TODO: 实现关节K参数设置
}

void BasicOrin::handle_RuntimeSetJointD_callback(const RuntimeSetJointD &msg) {
  (void)msg;
  RCLCPP_INFO(this->get_logger(), "RuntimeSetJointD callback called");
  // TODO: 实现关节D参数设置
}

void BasicOrin::handle_RuntimeSetJointPosCmd_callback(
    const RuntimeSetJointPosCmd &msg) {
  RCLCPP_INFO(this->get_logger(), "RuntimeSetJointPosCmd callback called");
  // TODO: 实现关节位置命令设置
  int nRes = FX_L1_Runtime_SetJointPosCmd(
      msg.thread_id, static_cast<FXObjType>(msg.obj_type),
      const_cast<double *>(msg.pos_cmd.data()));
  if (nRes != FUNC_RET_SUCCESS) {
    printf("Failed to send trajectory point");
  }
  RCLCPP_INFO(this->get_logger(), "RuntimeSetJointPosCmd Result = %d !", nRes);
}

void BasicOrin::handle_RuntimeSetJointPosPDCmd_callback(
    const RuntimeSetJointPosPDCmd &msg) {
  RCLCPP_INFO(this->get_logger(),
              "Topic /robot/runtime_set_joint_pos_pd_cmd, [2] = %f",
              msg.pos_cmd[2]);
  if (FX_L1_Runtime_SetJointPosPDCmd(
          msg.thread_id, static_cast<FXObjType>(msg.obj_type),
          const_cast<double *>(msg.pos_cmd.data())) != FUNC_RET_SUCCESS) {
    printf("Failed to send trajectory point");
  }
}

//--------------------------------------------------------------------------------------------
void BasicOrin::publishRobotSG(const RobotSG &msg) {
  pubRobotSG_->publish(msg);
}

void BasicOrin::publishRobotRT(const RobotRT &msg) {
  pubRobotRT_->publish(msg);
}

void BasicOrin::publishStateArm0(const FXStateTypeArm0 &msg) {
  pubStateArm0_->publish(msg);
}

//--------------------------------------------------------------------------------------------

void BasicOrin::timerSGCallback() {
  if (!enable_publish_.load()) {
    // RCLCPP_INFO(this->get_logger(), "Topic /robot/sg_feedback RETURN!");
    return;
  }
  std::lock_guard<std::mutex> lock(mtxSG_);
  sg_ptr_ = FX_L1_Fbk_GetSG();  ///< Status group feedback
  if (sg_ptr_) {
    RobotSG msg;
    convertROBOT_SG_to_RobotSG(sg_ptr_, msg);
    this->publishRobotSG(msg);
    RCLCPP_INFO(this->get_logger(), "Publish Topic /robot/sg_feedback!");
  } else {
    RCLCPP_INFO(this->get_logger(), "Publish Topic /robot/sg_feedback NULL!");
  }
}

void BasicOrin::timerRTCallback() {
  if (!enable_publish_.load()) {
    return;
  }
  std::lock_guard<std::mutex> lock(mtxRT_);

  rt_ptr_ = FX_L1_Fbk_GetRT();  ///< Real-time feedback
  if (rt_ptr_) {
    RobotRT msg;
    convertROBOT_RT_to_RobotRT(rt_ptr_, msg);  // 穷举赋值
    this->publishRobotRT(msg);
    RCLCPP_INFO(this->get_logger(), "Publish Topic /robot/rt_feedback!");
  } else {
    RCLCPP_INFO(this->get_logger(), "Topic /robot/rt_feedback NULL!");
  }
}

void BasicOrin::timerStateArm0Callback() {
  if (!enable_publish_.load()) {
    return;
  }
  std::lock_guard<std::mutex> lock(mtxStateType_);

  stateArm0_ = FX_L1_Fbk_CurrentState(FX_OBJ_ARM0);  ///< Real-time feedback
  FXStateTypeArm0 msg;
  msg.state_type = stateArm0_;
  this->publishStateArm0(msg);
  RCLCPP_INFO(this->get_logger(),
              "Publish Topic /robot/state_arm0 ,state = %d!", stateArm0_);
}
