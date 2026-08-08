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

  //------------------------------------------------------------------------------------------------------------------
 private:
  rclcpp::Publisher<Arm0HandGetData>::SharedPtr pubArm0GetData_;
  rclcpp::Publisher<Arm1HandGetData>::SharedPtr pubArm1GetData_;
  rclcpp::Publisher<FbkGetSystemMsg>::SharedPtr pubGetSystemMsg_;
  rclcpp::Publisher<TerminalArm0CHNCanFDGetData>::SharedPtr
      pubArm0CHNCanFDGetData_;
  rclcpp::Publisher<TerminalArm1CHNCanFDGetData>::SharedPtr
      pubArm1CHNCanFDGetData_;
  rclcpp::Publisher<RobotSG>::SharedPtr pubRobotSG_;
  rclcpp::Publisher<RobotRT>::SharedPtr pubRobotRT_;
  rclcpp::Publisher<FXStateTypeArm0>::SharedPtr pubStateArm0_;
  rclcpp::Publisher<FXStateTypeArm1>::SharedPtr pubStateArm1_;
  rclcpp::Publisher<FXStateTypeBody>::SharedPtr pubStateBody_;
  rclcpp::Publisher<FXStateTypeHead>::SharedPtr pubStateHead_;
  rclcpp::Publisher<FXStateTypeLift>::SharedPtr pubStateLift_;

  //------------------------------------------------------------------------
 private:
  rclcpp::Subscription<Arm0HandSetData>::SharedPtr subArm0HandSetData_;
  rclcpp::Subscription<Arm1HandSetData>::SharedPtr subArm1HandSetData_;

  rclcpp::Subscription<RuntimeStopTraj>::SharedPtr subStopTraj_;
  rclcpp::Subscription<RuntimeSetVelRatio>::SharedPtr subSetVelRatio_;
  rclcpp::Subscription<RuntimeSetTorqueCtrl>::SharedPtr subSetTorqueCtrl_;
  rclcpp::Subscription<RuntimeSetToolKD>::SharedPtr subSetToolKD_;
  rclcpp::Subscription<RuntimeSetToolK>::SharedPtr subSetToolK_;
  rclcpp::Subscription<RuntimeSetToolD>::SharedPtr subSetToolD_;
  rclcpp::Subscription<RuntimeSetTag>::SharedPtr subSetTag_;
  rclcpp::Subscription<RuntimeSetSpeedRatio>::SharedPtr subSetSpeedRatio_;
  rclcpp::Subscription<RuntimeSetJointMITCmd>::SharedPtr subSetJointMITCmd_;
  rclcpp::Subscription<RuntimeSetJointKD>::SharedPtr subSetJointKD_;
  rclcpp::Subscription<RuntimeSetJointK>::SharedPtr subSetJointK_;
  rclcpp::Subscription<RuntimeSetJointD>::SharedPtr subSetJointD_;
  rclcpp::Subscription<RuntimeSetJointPosCmd>::SharedPtr subSetJointPosCmd_;
  rclcpp::Subscription<RuntimeSetJointPosPDCmd>::SharedPtr subSetJointPosPDCmd_;

  // service
  // ------------------------------------------------------------------------------------------------------------------
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
  //   void handle_FbkGetSystemMsg_request(
  //       const std::shared_ptr<FbkGetSystemMsg::Request> request,
  //       std::shared_ptr<FbkGetSystemMsg::Response> response);
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

  //-----------------------------------------------------------------------------------------
 public:
  void publishRobotSG(const RobotSG &msg);
  void publishRobotRT(const RobotRT &msg);
  void publishStateArm0(const FXStateTypeArm0 &msg);

 private:
  void timerSGCallback();
  void timerRTCallback();
  void timerStateArm0Callback();
  // sub_topic------------------------------------------------------------------------------------------
 public:
  void handle_RuntimeStopTraj_callback(const RuntimeStopTraj &msg);
  void handle_RuntimeSetVelRatio_callback(const RuntimeSetVelRatio &msg);
  void handle_RuntimeSetTorqueCtrl_callback(const RuntimeSetTorqueCtrl &msg);
  void handle_RuntimeSetToolKD_callback(const RuntimeSetToolKD &msg);
  void handle_RuntimeSetToolK_callback(const RuntimeSetToolK &msg);
  void handle_RuntimeSetToolD_callback(const RuntimeSetToolD &msg);
  void handle_RuntimeSetTag_callback(const RuntimeSetTag &msg);
  void handle_RuntimeSetSpeedRatio_callback(const RuntimeSetSpeedRatio &msg);
  void handle_RuntimeSetJointMITCmd_callback(const RuntimeSetJointMITCmd &msg);
  void handle_RuntimeSetJointKD_callback(const RuntimeSetJointKD &msg);
  void handle_RuntimeSetJointK_callback(const RuntimeSetJointK &msg);
  void handle_RuntimeSetJointD_callback(const RuntimeSetJointD &msg);
  void handle_RuntimeSetJointPosCmd_callback(const RuntimeSetJointPosCmd &msg);

  void handle_RuntimeSetJointPosPDCmd_callback(
      const RuntimeSetJointPosPDCmd &msg);

  //------------------------------------------------------------------------------------------
 private:
  std::atomic<bool> enable_publish_{false};

  rclcpp::TimerBase::SharedPtr timerSG_;
  rclcpp::TimerBase::SharedPtr timerRT_;
  rclcpp::TimerBase::SharedPtr timerStateType_;

  std::mutex mtxSG_;
  std::mutex mtxRT_;
  std::mutex mtxStateType_;

  const ROBOT_SG *sg_ptr_;  ///< Status group feedback
  const ROBOT_RT *rt_ptr_;  ///< Real-time feedback
  FXStateType stateArm0_ = FX_STATE_UNKNOWN;

 private:
  void setPublishEnabled(bool enable) { enable_publish_.store(enable); }
  //------------------------------------------------------------------------------------------------------------------
  void convertROBOT_SG_to_RobotSG(const ROBOT_SG *sg_ptr, RobotSG &msg) {
    if (!sg_ptr) return;

    // ---- 顶层字段 ----
    msg.sg_frame_serial = sg_ptr->m_SG_FrameSerial;

    // ========== 头部 (HeadSG) ==========
    // HeadSet
    msg.head.head_set.vel_ratio =
        sg_ptr->m_HEAD.m_HEAD_SET.m_HEAD_Ctrl_VelRatio;
    msg.head.head_set.acc_ratio =
        sg_ptr->m_HEAD.m_HEAD_SET.m_HEAD_Ctrl_AccRatio;

    // HeadGet
    for (int i = 0; i < 3; ++i) {
      msg.head.head_get.fbk_joint_tor[i] =
          sg_ptr->m_HEAD.m_HEAD_GET.m_HEAD_FBK_Joint_Tor[i];
      msg.head.head_get.fbk_joint_ext_pos[i] =
          sg_ptr->m_HEAD.m_HEAD_GET.m_HEAD_FBK_Joint_ExtPos[i];
      msg.head.head_get.fbk_joint_temp[i] =
          sg_ptr->m_HEAD.m_HEAD_GET.m_HEAD_FBK_Joint_Temp[i];
    }
    // 注意：head_get.pad 在 ROS2 消息中没有对应字段，忽略

    // ========== 手臂 (ArmSG) 两个 ==========
    for (int arm_idx = 0; arm_idx < 2; ++arm_idx) {
      const ARM_SG &c_arm = sg_ptr->m_ARMS[arm_idx];
      auto &ros_arm = msg.arms[arm_idx];

      // ArmSet
      ros_arm.arm_set.ctrl_imp_type = c_arm.m_ARM_SET.m_ARM_Ctrl_ImpType;
      ros_arm.arm_set.ctrl_vel_ratio = c_arm.m_ARM_SET.m_ARM_Ctrl_VelRatio;
      ros_arm.arm_set.ctrl_acc_ratio = c_arm.m_ARM_SET.m_ARM_Ctrl_AccRatio;
      for (int i = 0; i < 7; ++i) {
        ros_arm.arm_set.ctrl_joint_k[i] = c_arm.m_ARM_SET.m_ARM_Ctrl_JointK[i];
        ros_arm.arm_set.ctrl_joint_d[i] = c_arm.m_ARM_SET.m_ARM_Ctrl_JointD[i];
        ros_arm.arm_set.ctrl_cart_k[i] = c_arm.m_ARM_SET.m_ARM_Ctrl_CartK[i];
        ros_arm.arm_set.ctrl_cart_d[i] = c_arm.m_ARM_SET.m_ARM_Ctrl_CartD[i];
      }
      for (int i = 0; i < 6; ++i) {
        ros_arm.arm_set.ctrl_tool_kine[i] =
            c_arm.m_ARM_SET.m_ARM_Ctrl_ToolKine[i];
      }
      for (int i = 0; i < 10; ++i) {
        ros_arm.arm_set.ctrl_tool_dyna[i] =
            c_arm.m_ARM_SET.m_ARM_Ctrl_ToolDyna[i];
      }

      // ArmGet
      for (int i = 0; i < 7; ++i) {
        ros_arm.arm_get.fbk_joint_tor[i] =
            c_arm.m_ARM_GET.m_ARM_FBK_Joint_Tor[i];
        ros_arm.arm_get.fbk_joint_ext_pos[i] =
            c_arm.m_ARM_GET.m_ARM_FBK_Joint_ExtPos[i];
        ros_arm.arm_get.fbk_joint_temp[i] =
            c_arm.m_ARM_GET.m_ARM_FBK_Joint_Temp[i];
      }
      ros_arm.arm_get.fbk_flange_di = c_arm.m_ARM_GET.m_ARM_FBK_Flange_DI;
      ros_arm.arm_get.fbk_low_spd_flag = c_arm.m_ARM_GET.m_ARM_FBK_LowSpdFlag;
      ros_arm.arm_get.fbk_traj_state = c_arm.m_ARM_GET.m_ARM_FBK_TrajState;
      ros_arm.arm_get.fbk_pd_cmd_quality =
          c_arm.m_ARM_GET.m_ARM_FBK_PD_CmdQuality;
      // 注意：arm_get.pad 在 ROS2 消息中没有，忽略
    }
    // ========== 身体 (BodySG) ==========
    const BODY_SG &c_body = sg_ptr->m_BODY;
    auto &ros_body = msg.body;

    // BodySet
    ros_body.body_set.ctrl_vel_ratio = c_body.m_BODY_SET.m_BODY_Ctrl_VelRatio;
    ros_body.body_set.ctrl_acc_ratio = c_body.m_BODY_SET.m_BODY_Ctrl_AccRatio;
    for (int i = 0; i < 6; ++i) {
      ros_body.body_set.ctrl_pd_k[i] = c_body.m_BODY_SET.m_BODY_Ctrl_PDK[i];
      ros_body.body_set.ctrl_pd_d[i] = c_body.m_BODY_SET.m_BODY_Ctrl_PDD[i];
    }

    // BodyGet
    for (int i = 0; i < 6; ++i) {
      ros_body.body_get.fbk_joint_tor[i] =
          c_body.m_BODY_GET.m_BODY_FBK_Joint_Tor[i];
      ros_body.body_get.fbk_joint_ext_pos[i] =
          c_body.m_BODY_GET.m_BODY_FBK_Joint_ExtPos[i];
      ros_body.body_get.fbk_joint_temp[i] =
          c_body.m_BODY_GET.m_BODY_FBK_Joint_Temp[i];
    }
    ros_body.body_get.fbk_traj_state = c_body.m_BODY_GET.m_BODY_FBK_TrajState;
    ros_body.body_get.fbk_pd_cmd_quality =
        c_body.m_BODY_GET.m_BODY_FBK_PD_CmdQuality;

    // ========== 升降 (LiftSG) ==========
    const LIFT_SG &c_lift = sg_ptr->m_LIFT;
    auto &ros_lift = msg.lift;

    // LiftSet
    ros_lift.lift_set.ctrl_vel_ratio = c_lift.m_LIFT_SET.m_LIFT_Ctrl_VelRatio;
    ros_lift.lift_set.ctrl_acc_ratio = c_lift.m_LIFT_SET.m_LIFT_Ctrl_AccRatio;

    // LiftGet
    for (int i = 0; i < 2; ++i) {
      ros_lift.lift_get.fbk_joint_tor[i] =
          c_lift.m_LIFT_GET.m_LIFT_FBK_Joint_Tor[i];
    }
    ros_lift.lift_get.fbk_traj_state = c_lift.m_LIFT_GET.m_LIFT_FBK_TrajState;
    // lift_get.pad 忽略
  }

  //------------------------------------------------------------------------------------------------------------------
  void convertROBOT_RT_to_RobotRT(const ROBOT_RT *rt_ptr, RobotRT &msg) {
    if (!rt_ptr) return;

    // ===== 顶层 =====
    msg.rt_frame_serial = rt_ptr->m_RT_FrameSerial;
    // msg.wait_serial = rt_ptr->wait_serial;
    // for (int i = 0; i < 3; ++i)
    //     msg.pad[i] = rt_ptr->pad[i];

    // ===== DeviceRT =====
    for (int i = 0; i < 6; ++i)
      msg.device.base_gyro[i] = rt_ptr->m_DEVICE.m_DEVICE_FBK_Base_Gyro[i];

    // ===== HeadRT =====
    // HeadIn
    for (int i = 0; i < 3; ++i)
      msg.head.head_in.cmd_joint_pos[i] =
          rt_ptr->m_HEAD.m_HEAD_IN.m_HEAD_CMD_Joint_Pos[i];

    // HeadOut
    for (int i = 0; i < 3; ++i)
      msg.head.head_out.fbk_joint_pos[i] =
          rt_ptr->m_HEAD.m_HEAD_OUT.m_HEAD_FBK_Joint_Pos[i];

    // ===== Arms (2) =====
    for (int arm = 0; arm < 2; ++arm) {
      const ARM_RT &c = rt_ptr->m_ARMS[arm];
      auto &r = msg.arms[arm];

      // StateCtr
      r.state.cur_state = c.m_ARM_State.m_CurState;
      r.state.cmd_state = c.m_ARM_State.m_CmdState;
      r.state.err_code = c.m_ARM_State.m_ERRCode;

      // ArmIn
      for (int i = 0; i < 7; ++i) {
        r.arm_in.cmd_joint_tor[i] = c.m_ARM_IN.m_ARM_CMD_Joint_Tor[i];
        r.arm_in.cmd_joint_pos[i] = c.m_ARM_IN.m_ARM_CMD_Joint_Pos[i];
      }
      r.arm_in.cmd_ctrl_drag_type = c.m_ARM_IN.m_ARM_CMD_Ctrl_DragType;

      for (int i = 0; i < 5; ++i) {
        r.arm_in.cmd_ctrl_force_dir[i] = c.m_ARM_IN.m_ARM_CMD_Ctrl_ForceDir[i];
        r.arm_in.cmd_ctrl_torque_dir[i] =
            c.m_ARM_IN.m_ARM_CMD_Ctrl_TorqueDir[i];
      }
      r.arm_in.cmd_tag = c.m_ARM_IN.m_ARM_CMD_Tag;
      r.arm_in.cmd_pd_serial = c.m_ARM_IN.m_ARM_CMD_PD_Serial;

      // ArmOut
      for (int i = 0; i < 7; ++i) {
        r.arm_out.fbk_joint_pos[i] = c.m_ARM_OUT.m_ARM_FBK_Joint_Pos[i];
        r.arm_out.fbk_joint_vel[i] = c.m_ARM_OUT.m_ARM_FBK_Joint_Vel[i];
        r.arm_out.fbk_joint_cmd[i] = c.m_ARM_OUT.m_ARM_FBK_Joint_Cmd[i];
        r.arm_out.fbk_joint_sensor_tor[i] =
            c.m_ARM_OUT.m_ARM_FBK_Joint_SensorTor[i];
        r.arm_out.fbk_joint_external_tor_est[i] =
            c.m_ARM_OUT.m_ARM_FBK_Joint_ExternalTorEst[i];
      }
      for (int i = 0; i < 6; ++i) {
        r.arm_out.fbk_base_fn_est[i] = c.m_ARM_OUT.m_ARM_FBK_Base_FNEst[i];
        r.arm_out.fbk_flange_ft_sensor[i] =
            c.m_ARM_OUT.m_ARM_FBK_Flange_FTSensor[i];
      }
    }

    // ===== BodyRT =====
    // 根据您的 BodyRT 定义，没有 state，只有 body_in 和 body_out
    const BODY_RT &cb = rt_ptr->m_BODY;
    auto &rb = msg.body;

    rb.body_in.cmd_ctrl_type = cb.m_BODY_IN.m_BODY_CMD_Ctrl_Type;
    for (int i = 0; i < 6; ++i) {
      rb.body_in.cmd_joint_pos[i] = cb.m_BODY_IN.m_BODY_CMD_Joint_Pos[i];
      rb.body_out.fbk_joint_pos[i] = cb.m_BODY_OUT.m_BODY_FBK_Joint_Pos[i];
      rb.body_out.fbk_joint_vel[i] = cb.m_BODY_OUT.m_BODY_FBK_Joint_Vel[i];
      rb.body_out.fbk_joint_sensor_tor[i] =
          cb.m_BODY_OUT.m_BODY_FBK_Joint_SensorTor[i];
    }
    rb.body_in.cmd_tag = cb.m_BODY_IN.m_BODY_CMD_Tag;
    rb.body_in.cmd_pd_serial = cb.m_BODY_IN.m_BODY_CMD_PD_Serial;

    // ===== LiftRT =====
    // 同样没有 state
    const LIFT_RT &cl = rt_ptr->m_LIFT;
    auto &rl = msg.lift;

    for (int i = 0; i < 2; ++i) {
      rl.lift_in.cmd_joint_pos[i] = cl.m_LIFT_IN.m_LIFT_CMD_Joint_Pos[i];
      rl.lift_out.fbk_joint_pos[i] = cl.m_LIFT_OUT.m_LIFT_FBK_Joint_Pos[i];
    }
  }
};
