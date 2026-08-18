#include <algorithm>  // 确保包含
#include <chrono>
#include <functional>
#include <iomanip>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>

#include "L1Robot.h"
#include "interfaces_msg.h"
#include "interfaces_srv.h"

using namespace std::chrono_literals;

class BasicOrin : public rclcpp::Node {
 public:
  BasicOrin();

 private:
  int initService();
  int initTopic();

  //------------------------------------------------------------------------------------------------------------------
 public:
  rclcpp::Publisher<Arm0HandGetData>::SharedPtr pubArm0GetData_;
  rclcpp::Publisher<Arm1HandGetData>::SharedPtr pubArm1GetData_;
  rclcpp::Publisher<FbkGetSystemMsg>::SharedPtr pubGetSystemMsg_;

  rclcpp::Publisher<RobotSG>::SharedPtr pubRobotSG_;
  rclcpp::Publisher<RobotRT>::SharedPtr pubRobotRT_;
  rclcpp::Publisher<FXStateTypeArm0>::SharedPtr pubStateArm0_;
  rclcpp::Publisher<FXStateTypeArm1>::SharedPtr pubStateArm1_;
  rclcpp::Publisher<FXStateTypeBody>::SharedPtr pubStateBody_;
  rclcpp::Publisher<FXStateTypeHead>::SharedPtr pubStateHead_;
  rclcpp::Publisher<FXStateTypeLift>::SharedPtr pubStateLift_;

  rclcpp::Publisher<TerminalArm0CanFDGetData>::SharedPtr pubArm0CanFDGet_;
  rclcpp::Publisher<TerminalArm1CanFDGetData>::SharedPtr pubArm1CanFDGet_;

  //------------------------------------------------------------------------
 private:
  rclcpp::Subscription<Arm0HandSetData>::SharedPtr subArm0HandSetData_;
  rclcpp::Subscription<Arm1HandSetData>::SharedPtr subArm1HandSetData_;

  rclcpp::Subscription<RuntimeSetJointD>::SharedPtr subSetJointD_;
  rclcpp::Subscription<RuntimeSetJointK>::SharedPtr subSetJointK_;
  rclcpp::Subscription<RuntimeSetJointKD>::SharedPtr subSetJointKD_;
  rclcpp::Subscription<RuntimeSetJointMITCmd>::SharedPtr subSetJointMITCmd_;
  rclcpp::Subscription<RuntimeSetJointPosCmd>::SharedPtr subSetJointPosCmd_;
  rclcpp::Subscription<RuntimeSetJointPosPDCmd>::SharedPtr subSetJointPosPDCmd_;
  rclcpp::Subscription<RuntimeSetSpeedRatio>::SharedPtr subSetSpeedRatio_;
  rclcpp::Subscription<RuntimeSetTag>::SharedPtr subSetTag_;
  rclcpp::Subscription<RuntimeSetToolD>::SharedPtr subSetToolD_;
  rclcpp::Subscription<RuntimeSetToolK>::SharedPtr subSetToolK_;
  rclcpp::Subscription<RuntimeSetToolKD>::SharedPtr subSetToolKD_;
  rclcpp::Subscription<RuntimeSetTorqueCtrl>::SharedPtr subSetTorqueCtrl_;
  rclcpp::Subscription<RuntimeSetVelRatio>::SharedPtr subSetVelRatio_;
  rclcpp::Subscription<RuntimeStopTraj>::SharedPtr subStopTraj_;

  rclcpp::Subscription<TerminalArm0CanFDSetData>::SharedPtr subArm0CanFDSet_;
  rclcpp::Subscription<TerminalArm1CanFDSetData>::SharedPtr subArm1CanFDSet_;

  //-----------------------------------------------------------------------------------------
 private:
  void timerSGCallback();
  void timerRTCallback();

  void timerStateTypeCallback();

  void timerCanFDarm0Callback();
  void timerCanFDarm1Callback();

  // sub_topic------------------------------------------------------------------------------------------
 public:
  void handle_Arm0HandSetData_callback(const Arm0HandSetData &msg);
  void handle_Arm1HandSetData_callback(const Arm1HandSetData &msg);

  void handle_RuntimeSetJointD_callback(const RuntimeSetJointD &msg);
  void handle_RuntimeSetJointK_callback(const RuntimeSetJointK &msg);
  void handle_RuntimeSetJointKD_callback(const RuntimeSetJointKD &msg);
  void handle_RuntimeSetJointMITCmd_callback(const RuntimeSetJointMITCmd &msg);
  void handle_RuntimeSetJointPosCmd_callback(const RuntimeSetJointPosCmd &msg);
  void handle_RuntimeSetJointPosPDCmd_callback(
      const RuntimeSetJointPosPDCmd &msg);
  void handle_RuntimeSetSpeedRatio_callback(const RuntimeSetSpeedRatio &msg);
  void handle_RuntimeSetTag_callback(const RuntimeSetTag &msg);
  void handle_RuntimeSetToolD_callback(const RuntimeSetToolD &msg);
  void handle_RuntimeSetToolK_callback(const RuntimeSetToolK &msg);
  void handle_RuntimeSetToolKD_callback(const RuntimeSetToolKD &msg);
  void handle_RuntimeSetTorqueCtrl_callback(const RuntimeSetTorqueCtrl &msg);
  void handle_RuntimeSetVelRatio_callback(const RuntimeSetVelRatio &msg);
  void handle_RuntimeStopTraj_callback(const RuntimeStopTraj &msg);

  void handle_TerminalArm0CanFDSetData_callback(
      const TerminalArm0CanFDSetData &msg);
  void handle_TerminalArm1CanFDSetData_callback(
      const TerminalArm1CanFDSetData &msg);

  //------------------------------------------------------------------------------------------
 private:
  std::atomic<bool> enable_publish_{false};

  rclcpp::TimerBase::SharedPtr timerSG_;
  rclcpp::TimerBase::SharedPtr timerRT_;
  rclcpp::TimerBase::SharedPtr timerStateType_;
  rclcpp::TimerBase::SharedPtr timerCanFDarm0_;
  rclcpp::TimerBase::SharedPtr timerCanFDarm1_;

  std::mutex mtxSG_;
  std::mutex mtxRT_;
  std::mutex mtxStateType_;

  const ROBOT_SG *sg_ptr_;  ///< Status group feedback
  const ROBOT_RT *rt_ptr_;  ///< Real-time feedback

  // FX_OBJ_ARM0 = 0, ///< Left arm
  //     FX_OBJ_ARM1 = 1, ///< Right arm
  //     FX_OBJ_HEAD = 2, ///< Head module
  //     FX_OBJ_BODY = 3, ///< Body module
  //     FX_OBJ_LIFT = 4, ///< Lift module

  FXStateType stateArm0_ = FX_STATE_UNKNOWN;
  FXStateType stateArm1_ = FX_STATE_UNKNOWN;
  FXStateType stateHead_ = FX_STATE_UNKNOWN;
  FXStateType stateBody_ = FX_STATE_UNKNOWN;
  FXStateType stateLift_ = FX_STATE_UNKNOWN;

 private:
  void setPublishEnabled(bool enable) { enable_publish_.store(enable); }
  //------------------------------------------------------------------------------------------------------------------
  void convertROBOT_SG_to_RobotSG(const ROBOT_SG *sg_ptr, RobotSG &msg);
  //------------------------------------------------------------------------------------------------------------------
  void convertROBOT_RT_to_RobotRT(const ROBOT_RT *rt_ptr, RobotRT &msg);

  //----------------------------------------------------------------------------------------------------------------
 private:
  rclcpp::Service<ConfigClearEncError>::SharedPtr serviceClearEncError_;
  rclcpp::Service<ConfigDisableSoftLimit>::SharedPtr serviceDisableSoftLimit_;
  rclcpp::Service<ConfigGetPDCmdCycleTime>::SharedPtr serviceGetPDCmdCycleTime_;
  rclcpp::Service<ConfigResetAxisSensorOffset>::SharedPtr
      serviceResetAxisSensorOffset_;
  rclcpp::Service<ConfigResetEncOffset>::SharedPtr serviceResetEncOffset_;
  rclcpp::Service<ConfigResetSensorOffset>::SharedPtr serviceResetSensorOffset_;
  rclcpp::Service<ConfigSetBrakeLock>::SharedPtr serviceSetBrakeLock_;
  rclcpp::Service<ConfigSetBrakeUnlock>::SharedPtr serviceSetBrakeUnlock_;
  rclcpp::Service<ConfigSetPDCmdCycleTime>::SharedPtr serviceSetPDCmdCycleTime_;
  rclcpp::Service<ConfigSetTraj>::SharedPtr serviceSetTraj_;
  rclcpp::Service<FbkCheckUserDataSet>::SharedPtr serviceCheckUserDataSet_;
  rclcpp::Service<FbkGetCtrlObjDof>::SharedPtr serviceGetCtrlObjDof_;
  rclcpp::Service<FbkGetCtrlObjPhysicalState>::SharedPtr
      serviceGetCtrlObjPhysicalState_;
  rclcpp::Service<FbkGetCtrlObjSensorInfo>::SharedPtr
      serviceGetCtrlObjSensorInfo_;
  rclcpp::Service<FbkGetCtrlObjServoVersion>::SharedPtr
      serviceGetCtrlObjServoVersion_;
  // rclcpp::Service<FbkGetSystemMsg>::SharedPtr serviceGetSystemMsg_;
  rclcpp::Service<FbkGetUserData>::SharedPtr serviceGetUserData_;
  rclcpp::Service<FbkRegisterUserDataSet>::SharedPtr
      serviceRegisterUserDataSet_;
  rclcpp::Service<FbkResetSystemMsg>::SharedPtr serviceResetSystemMsg_;
  rclcpp::Service<FbkResetUserDataSet>::SharedPtr serviceResetUserDataSet_;
  rclcpp::Service<ParamGetFloat>::SharedPtr serviceGetFloat_;
  rclcpp::Service<ParamGetInt32>::SharedPtr serviceGetInt32_;
  rclcpp::Service<ParamGetString>::SharedPtr serviceGetString_;
  rclcpp::Service<ParamSetFloat>::SharedPtr serviceSetFloat_;
  rclcpp::Service<ParamSetInt32>::SharedPtr serviceSetInt32_;
  rclcpp::Service<RuntimeEmergencyStop>::SharedPtr serviceEmergencyStop_;
  rclcpp::Service<RuntimeRunTraj>::SharedPtr serviceRunTraj_;
  rclcpp::Service<RuntimeSetAccRatio>::SharedPtr serviceSetAccRatio_;
  rclcpp::Service<RuntimeSetBodyPDD>::SharedPtr serviceSetBodyPDD_;
  rclcpp::Service<RuntimeSetBodyPDP>::SharedPtr serviceSetBodyPDP_;
  rclcpp::Service<RuntimeSetBodyPD>::SharedPtr serviceSetBodyPD_;
  rclcpp::Service<RuntimeSetCartKD>::SharedPtr serviceSetCartKD_;
  rclcpp::Service<RuntimeSetCartK>::SharedPtr serviceSetCartK_;
  rclcpp::Service<RuntimeSetForceCtrl>::SharedPtr serviceSetForceCtrl_;
  rclcpp::Service<RuntimeSetRefOri>::SharedPtr serviceSetRefOri_;
  rclcpp::Service<StateGetServoErrorCode>::SharedPtr serviceGetServoErrorCode_;
  rclcpp::Service<StateResetError>::SharedPtr serviceResetError_;
  rclcpp::Service<StateSwitchToCollaborativeRelease>::SharedPtr
      serviceSwitchToCollaborativeRelease_;
  rclcpp::Service<StateSwitchToDragCartR>::SharedPtr serviceSwitchToDragCartR_;
  rclcpp::Service<StateSwitchToDragCartX>::SharedPtr serviceSwitchToDragCartX_;
  rclcpp::Service<StateSwitchToDragCartY>::SharedPtr serviceSwitchToDragCartY_;
  rclcpp::Service<StateSwitchToDragCartZ>::SharedPtr serviceSwitchToDragCartZ_;
  rclcpp::Service<StateSwitchToDragJoint>::SharedPtr serviceSwitchToDragJoint_;
  rclcpp::Service<StateSwitchToIdle>::SharedPtr serviceSwitchToIdle_;
  rclcpp::Service<StateSwitchToImpCart>::SharedPtr serviceSwitchToImpCart_;
  rclcpp::Service<StateSwitchToImpForce>::SharedPtr serviceSwitchToImpForce_;
  rclcpp::Service<StateSwitchToImpJoint>::SharedPtr serviceSwitchToImpJoint_;
  rclcpp::Service<StateSwitchToMIT>::SharedPtr serviceSwitchToMIT_;
  rclcpp::Service<StateSwitchToPD>::SharedPtr serviceSwitchToPD_;
  rclcpp::Service<StateSwitchToPosition>::SharedPtr serviceSwitchToPosition_;
  rclcpp::Service<StateSwitchToUserMIT>::SharedPtr serviceSwitchToUserMIT_;
  rclcpp::Service<SystemGetControllerVersion>::SharedPtr serviceGetCtrlVer_;
  rclcpp::Service<SystemGetIP>::SharedPtr serviceGetIP_;
  rclcpp::Service<SystemGetLinkState>::SharedPtr serviceGetLinkState_;
  rclcpp::Service<SystemGetLogLevel>::SharedPtr serviceGetLogLevel_;
  rclcpp::Service<SystemGetRobotType>::SharedPtr serviceGetRobotType_;
  rclcpp::Service<SystemGetSDKVersion>::SharedPtr serviceGetSDKVer_;
  rclcpp::Service<SystemGetTime>::SharedPtr serviceGetTime_;
  rclcpp::Service<SystemLink>::SharedPtr serviceLink_;
  rclcpp::Service<SystemReboot>::SharedPtr serviceReboot_;
  rclcpp::Service<SystemRecvFile>::SharedPtr serviceRecvFile_;
  rclcpp::Service<SystemRestoreImage>::SharedPtr serviceRestoreImage_;
  rclcpp::Service<SystemSendFile>::SharedPtr serviceSendFile_;
  rclcpp::Service<SystemSetIP>::SharedPtr serviceSetIP_;
  rclcpp::Service<SystemSetLogLevel>::SharedPtr serviceSetLogLevel_;
  rclcpp::Service<SystemSetTime>::SharedPtr serviceSetTime_;
  rclcpp::Service<SystemSetUserFbkType>::SharedPtr serviceSetUserFbkType_;
  rclcpp::Service<SystemUnlink>::SharedPtr serviceUnlink_;
  rclcpp::Service<SystemUpdate>::SharedPtr serviceUpdate_;
  rclcpp::Service<TerminalCH485GetData>::SharedPtr serviceCH485GetData_;
  rclcpp::Service<TerminalCH485SetData>::SharedPtr serviceCH485SetData_;
  rclcpp::Service<TerminalClearData>::SharedPtr serviceClearData_;

  // service------------------------------------------------------------------------------------------------------------------
 private:
  void handle_ConfigClearEncError_request(
      const std::shared_ptr<ConfigClearEncError::Request> request,
      std::shared_ptr<ConfigClearEncError::Response> response);
  void handle_ConfigDisableSoftLimit_request(
      const std::shared_ptr<ConfigDisableSoftLimit::Request> request,
      std::shared_ptr<ConfigDisableSoftLimit::Response> response);
  void handle_ConfigGetPDCmdCycleTime_request(
      const std::shared_ptr<ConfigGetPDCmdCycleTime::Request> request,
      std::shared_ptr<ConfigGetPDCmdCycleTime::Response> response);
  void handle_ConfigResetAxisSensorOffset_request(
      const std::shared_ptr<ConfigResetAxisSensorOffset::Request> request,
      std::shared_ptr<ConfigResetAxisSensorOffset::Response> response);
  void handle_ConfigResetEncOffset_request(
      const std::shared_ptr<ConfigResetEncOffset::Request> request,
      std::shared_ptr<ConfigResetEncOffset::Response> response);
  void handle_ConfigResetSensorOffset_request(
      const std::shared_ptr<ConfigResetSensorOffset::Request> request,
      std::shared_ptr<ConfigResetSensorOffset::Response> response);
  void handle_ConfigSetBrakeLock_request(
      const std::shared_ptr<ConfigSetBrakeLock::Request> request,
      std::shared_ptr<ConfigSetBrakeLock::Response> response);
  void handle_ConfigSetBrakeUnlock_request(
      const std::shared_ptr<ConfigSetBrakeUnlock::Request> request,
      std::shared_ptr<ConfigSetBrakeUnlock::Response> response);
  void handle_ConfigSetPDCmdCycleTime_request(
      const std::shared_ptr<ConfigSetPDCmdCycleTime::Request> request,
      std::shared_ptr<ConfigSetPDCmdCycleTime::Response> response);
  void handle_ConfigSetTraj_request(
      const std::shared_ptr<ConfigSetTraj::Request> request,
      std::shared_ptr<ConfigSetTraj::Response> response);
  void handle_FbkCheckUserDataSet_request(
      const std::shared_ptr<FbkCheckUserDataSet::Request> request,
      std::shared_ptr<FbkCheckUserDataSet::Response> response);
  void handle_FbkGetCtrlObjDof_request(
      const std::shared_ptr<FbkGetCtrlObjDof::Request> request,
      std::shared_ptr<FbkGetCtrlObjDof::Response> response);
  void handle_FbkGetCtrlObjPhysicalState_request(
      const std::shared_ptr<FbkGetCtrlObjPhysicalState::Request> request,
      std::shared_ptr<FbkGetCtrlObjPhysicalState::Response> response);
  void handle_FbkGetCtrlObjSensorInfo_request(
      const std::shared_ptr<FbkGetCtrlObjSensorInfo::Request> request,
      std::shared_ptr<FbkGetCtrlObjSensorInfo::Response> response);
  void handle_FbkGetCtrlObjServoVersion_request(
      const std::shared_ptr<FbkGetCtrlObjServoVersion::Request> request,
      std::shared_ptr<FbkGetCtrlObjServoVersion::Response> response);
  void handle_FbkGetUserData_request(
      const std::shared_ptr<FbkGetUserData::Request> request,
      std::shared_ptr<FbkGetUserData::Response> response);
  void handle_FbkRegisterUserDataSet_request(
      const std::shared_ptr<FbkRegisterUserDataSet::Request> request,
      std::shared_ptr<FbkRegisterUserDataSet::Response> response);
  void handle_FbkResetSystemMsg_request(
      const std::shared_ptr<FbkResetSystemMsg::Request> request,
      std::shared_ptr<FbkResetSystemMsg::Response> response);
  void handle_FbkResetUserDataSet_request(
      const std::shared_ptr<FbkResetUserDataSet::Request> request,
      std::shared_ptr<FbkResetUserDataSet::Response> response);
  void handle_ParamGetFloat_request(
      const std::shared_ptr<ParamGetFloat::Request> request,
      std::shared_ptr<ParamGetFloat::Response> response);
  void handle_ParamGetInt32_request(
      const std::shared_ptr<ParamGetInt32::Request> request,
      std::shared_ptr<ParamGetInt32::Response> response);
  void handle_ParamGetString_request(
      const std::shared_ptr<ParamGetString::Request> request,
      std::shared_ptr<ParamGetString::Response> response);
  void handle_ParamSetFloat_request(
      const std::shared_ptr<ParamSetFloat::Request> request,
      std::shared_ptr<ParamSetFloat::Response> response);
  void handle_ParamSetInt32_request(
      const std::shared_ptr<ParamSetInt32::Request> request,
      std::shared_ptr<ParamSetInt32::Response> response);
  void handle_RuntimeEmergencyStop_request(
      const std::shared_ptr<RuntimeEmergencyStop::Request> request,
      std::shared_ptr<RuntimeEmergencyStop::Response> response);
  void handle_RuntimeRunTraj_request(
      const std::shared_ptr<RuntimeRunTraj::Request> request,
      std::shared_ptr<RuntimeRunTraj::Response> response);
  void handle_RuntimeSetAccRatio_request(
      const std::shared_ptr<RuntimeSetAccRatio::Request> request,
      std::shared_ptr<RuntimeSetAccRatio::Response> response);
  void handle_RuntimeSetBodyPDD_request(
      const std::shared_ptr<RuntimeSetBodyPDD::Request> request,
      std::shared_ptr<RuntimeSetBodyPDD::Response> response);
  void handle_RuntimeSetBodyPDP_request(
      const std::shared_ptr<RuntimeSetBodyPDP::Request> request,
      std::shared_ptr<RuntimeSetBodyPDP::Response> response);
  void handle_RuntimeSetBodyPD_request(
      const std::shared_ptr<RuntimeSetBodyPD::Request> request,
      std::shared_ptr<RuntimeSetBodyPD::Response> response);
  void handle_RuntimeSetCartKD_request(
      const std::shared_ptr<RuntimeSetCartKD::Request> request,
      std::shared_ptr<RuntimeSetCartKD::Response> response);
  void handle_RuntimeSetCartK_request(
      const std::shared_ptr<RuntimeSetCartK::Request> request,
      std::shared_ptr<RuntimeSetCartK::Response> response);
  void handle_RuntimeSetForceCtrl_request(
      const std::shared_ptr<RuntimeSetForceCtrl::Request> request,
      std::shared_ptr<RuntimeSetForceCtrl::Response> response);
  void handle_RuntimeSetRefOri_request(
      const std::shared_ptr<RuntimeSetRefOri::Request> request,
      std::shared_ptr<RuntimeSetRefOri::Response> response);
  void handle_StateGetServoErrorCode_request(
      const std::shared_ptr<StateGetServoErrorCode::Request> request,
      std::shared_ptr<StateGetServoErrorCode::Response> response);
  void handle_StateResetError_request(
      const std::shared_ptr<StateResetError::Request> request,
      std::shared_ptr<StateResetError::Response> response);
  void handle_StateSwitchToCollaborativeRelease_request(
      const std::shared_ptr<StateSwitchToCollaborativeRelease::Request> request,
      std::shared_ptr<StateSwitchToCollaborativeRelease::Response> response);
  void handle_StateSwitchToDragCartR_request(
      const std::shared_ptr<StateSwitchToDragCartR::Request> request,
      std::shared_ptr<StateSwitchToDragCartR::Response> response);
  void handle_StateSwitchToDragCartX_request(
      const std::shared_ptr<StateSwitchToDragCartX::Request> request,
      std::shared_ptr<StateSwitchToDragCartX::Response> response);
  void handle_StateSwitchToDragCartY_request(
      const std::shared_ptr<StateSwitchToDragCartY::Request> request,
      std::shared_ptr<StateSwitchToDragCartY::Response> response);
  void handle_StateSwitchToDragCartZ_request(
      const std::shared_ptr<StateSwitchToDragCartZ::Request> request,
      std::shared_ptr<StateSwitchToDragCartZ::Response> response);
  void handle_StateSwitchToDragJoint_request(
      const std::shared_ptr<StateSwitchToDragJoint::Request> request,
      std::shared_ptr<StateSwitchToDragJoint::Response> response);
  void handle_StateSwitchToIdle_request(
      const std::shared_ptr<StateSwitchToIdle::Request> request,
      std::shared_ptr<StateSwitchToIdle::Response> response);
  void handle_StateSwitchToImpCart_request(
      const std::shared_ptr<StateSwitchToImpCart::Request> request,
      std::shared_ptr<StateSwitchToImpCart::Response> response);
  void handle_StateSwitchToImpForce_request(
      const std::shared_ptr<StateSwitchToImpForce::Request> request,
      std::shared_ptr<StateSwitchToImpForce::Response> response);
  void handle_StateSwitchToImpJoint_request(
      const std::shared_ptr<StateSwitchToImpJoint::Request> request,
      std::shared_ptr<StateSwitchToImpJoint::Response> response);
  void handle_StateSwitchToMIT_request(
      const std::shared_ptr<StateSwitchToMIT::Request> request,
      std::shared_ptr<StateSwitchToMIT::Response> response);
  void handle_StateSwitchToPD_request(
      const std::shared_ptr<StateSwitchToPD::Request> request,
      std::shared_ptr<StateSwitchToPD::Response> response);
  void handle_StateSwitchToPosition_request(
      const std::shared_ptr<StateSwitchToPosition::Request> request,
      std::shared_ptr<StateSwitchToPosition::Response> response);
  void handle_StateSwitchToUserMIT_request(
      const std::shared_ptr<StateSwitchToUserMIT::Request> request,
      std::shared_ptr<StateSwitchToUserMIT::Response> response);
  void handle_SystemGetControllerVersion_request(
      const std::shared_ptr<SystemGetControllerVersion::Request> request,
      std::shared_ptr<SystemGetControllerVersion::Response> response);
  void handle_SystemGetIP_request(
      const std::shared_ptr<SystemGetIP::Request> request,
      std::shared_ptr<SystemGetIP::Response> response);
  void handle_SystemGetLinkState_request(
      const std::shared_ptr<SystemGetLinkState::Request> request,
      std::shared_ptr<SystemGetLinkState::Response> response);
  void handle_SystemGetLogLevel_request(
      const std::shared_ptr<SystemGetLogLevel::Request> request,
      std::shared_ptr<SystemGetLogLevel::Response> response);
  void handle_SystemGetRobotType_request(
      const std::shared_ptr<SystemGetRobotType::Request> request,
      std::shared_ptr<SystemGetRobotType::Response> response);
  void handle_SystemGetSDKVersion_request(
      const std::shared_ptr<SystemGetSDKVersion::Request> request,
      std::shared_ptr<SystemGetSDKVersion::Response> response);
  void handle_SystemGetTime_request(
      const std::shared_ptr<SystemGetTime::Request> request,
      std::shared_ptr<SystemGetTime::Response> response);
  void handle_SystemLink_request(
      const std::shared_ptr<SystemLink::Request> request,
      std::shared_ptr<SystemLink::Response> response);
  void handle_SystemReboot_request(
      const std::shared_ptr<SystemReboot::Request> request,
      std::shared_ptr<SystemReboot::Response> response);
  void handle_SystemRecvFile_request(
      const std::shared_ptr<SystemRecvFile::Request> request,
      std::shared_ptr<SystemRecvFile::Response> response);
  void handle_SystemRestoreImage_request(
      const std::shared_ptr<SystemRestoreImage::Request> request,
      std::shared_ptr<SystemRestoreImage::Response> response);
  void handle_SystemSendFile_request(
      const std::shared_ptr<SystemSendFile::Request> request,
      std::shared_ptr<SystemSendFile::Response> response);
  void handle_SystemSetIP_request(
      const std::shared_ptr<SystemSetIP::Request> request,
      std::shared_ptr<SystemSetIP::Response> response);
  void handle_SystemSetLogLevel_request(
      const std::shared_ptr<SystemSetLogLevel::Request> request,
      std::shared_ptr<SystemSetLogLevel::Response> response);
  void handle_SystemSetTime_request(
      const std::shared_ptr<SystemSetTime::Request> request,
      std::shared_ptr<SystemSetTime::Response> response);
  void handle_SystemSetUserFbkType_request(
      const std::shared_ptr<SystemSetUserFbkType::Request> request,
      std::shared_ptr<SystemSetUserFbkType::Response> response);
  void handle_SystemUnlink_request(
      const std::shared_ptr<SystemUnlink::Request> request,
      std::shared_ptr<SystemUnlink::Response> response);
  void handle_SystemUpdate_request(
      const std::shared_ptr<SystemUpdate::Request> request,
      std::shared_ptr<SystemUpdate::Response> response);
  void handle_TerminalCH485GetData_request(
      const std::shared_ptr<TerminalCH485GetData::Request> request,
      std::shared_ptr<TerminalCH485GetData::Response> response);
  void handle_TerminalCH485SetData_request(
      const std::shared_ptr<TerminalCH485SetData::Request> request,
      std::shared_ptr<TerminalCH485SetData::Response> response);
  void handle_TerminalClearData_request(
      const std::shared_ptr<TerminalClearData::Request> request,
      std::shared_ptr<TerminalClearData::Response> response);
};
