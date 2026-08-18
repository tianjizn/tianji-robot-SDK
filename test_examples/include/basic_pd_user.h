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

class BasicPDUser : public rclcpp::Node {
 public:
  BasicPDUser();
  //----------------------------------------------------------------------------------------------------------------
 private:
  rclcpp::Client<SystemLink>::SharedPtr client0_;
  rclcpp::Client<SystemUnlink>::SharedPtr clientUnlink_;
  rclcpp::Client<SystemGetSDKVersion>::SharedPtr client1_;
  rclcpp::Client<SystemGetControllerVersion>::SharedPtr client2_;

  rclcpp::Client<StateResetError>::SharedPtr client4_;
  rclcpp::Client<StateSwitchToIdle>::SharedPtr client5_;
  rclcpp::Client<ConfigSetPDCmdCycleTime>::SharedPtr client6_;
  rclcpp::Client<StateSwitchToPD>::SharedPtr client7_;
  rclcpp::Client<StateSwitchToPosition>::SharedPtr client8_;

  rclcpp::Subscription<RobotRT>::SharedPtr subscription0_;
  rclcpp::Subscription<RobotSG>::SharedPtr subscription1_;
  rclcpp::Subscription<FXStateTypeArm0>::SharedPtr subscription2_;
  rclcpp::Subscription<TerminalArm0CanFDGetData>::SharedPtr subscription3_;
  rclcpp::Subscription<TerminalArm1CanFDGetData>::SharedPtr subscription4_;
  rclcpp::Subscription<FXStateTypeArm1>::SharedPtr subscription5_;

  rclcpp::Publisher<RuntimeSetJointPosPDCmd>::SharedPtr publisher_;
  rclcpp::Publisher<RuntimeSetJointPosCmd>::SharedPtr publisher1_;
  rclcpp::Publisher<TerminalArm0CanFDSetData>::SharedPtr publisher2_;
  rclcpp::Publisher<TerminalArm1CanFDSetData>::SharedPtr publisher3_;

  rclcpp::Publisher<RuntimeSetToolKD>::SharedPtr publisher4_;

  std::atomic<bool> bSystemLinked_{false};
  std::atomic<FXStateType> stateArm0_{FXStateType::FX_STATE_UNKNOWN};
  std::atomic<FXStateType> stateArm1_{FXStateType::FX_STATE_UNKNOWN};

  mutable std::mutex mtx_rt_;
  std::shared_ptr<RobotRT> rt_ptr_;

  mutable std::mutex mtx_sg_;
  std::shared_ptr<RobotSG> sg_ptr_;

  mutable std::mutex mtx_canfd0_;
  std::shared_ptr<TerminalArm0CanFDGetData> canFDarm0_ptr_;

  mutable std::mutex mtx_canfd1_;
  std::shared_ptr<TerminalArm1CanFDGetData> canFDarm1_ptr_;

  rclcpp::Executor *executor_ = nullptr;

 public:
  void setExecutor(rclcpp::Executor *exec) { executor_ = exec; }

  void setSystemLinked(bool bLinked) { bSystemLinked_.store(bLinked); }

  bool getSystemLinked() const {
    return bSystemLinked_.load(std::memory_order_acquire);
  }

  FXStateType getStateArm0() const {
    return stateArm0_.load(std::memory_order_acquire);
  }
  FXStateType getStateArm1() const {
    return stateArm1_.load(std::memory_order_acquire);
  }

  RobotRT::SharedPtr getRobotRT() const {
    std::lock_guard<std::mutex> lock(mtx_rt_);
    return rt_ptr_;
  }

  RobotSG::SharedPtr getRobotSG() const {
    std::lock_guard<std::mutex> lock(mtx_sg_);
    return sg_ptr_;
  }

  TerminalArm0CanFDGetData::SharedPtr getCanFDarm0Data() const {
    std::lock_guard<std::mutex> lock(mtx_canfd0_);
    return canFDarm0_ptr_;
  }

  TerminalArm1CanFDGetData::SharedPtr getCanFDarm1Data() const {
    std::lock_guard<std::mutex> lock(mtx_canfd1_);
    return canFDarm1_ptr_;
  }

  //------------------------------------------------------------------------------
  std::shared_ptr<SystemLink::Response> send_SystemLink_request(
      uint8_t ip1, uint8_t ip2, uint8_t ip3, uint8_t ip4, uint32_t log_level);

  std::shared_ptr<SystemUnlink::Response> send_SystemUnlink_request();

  std::shared_ptr<SystemGetSDKVersion::Response>
  send_SystemGetSDKVersion_request();
  std::shared_ptr<SystemGetControllerVersion::Response>
  send_SystemGetControllerVersion_request();

  std::shared_ptr<StateResetError::Response> send_StateResetError_request(
      uint8_t obj_type, uint32_t timeout);
  std::shared_ptr<StateSwitchToIdle::Response> send_StateSwitchToIdle_request(
      uint8_t obj_type, uint32_t timeout);
  std::shared_ptr<ConfigSetPDCmdCycleTime::Response>
  send_ConfigSetPDCmdCycleTime_request(int32_t cycle_time);
  std::shared_ptr<StateSwitchToPD::Response> send_StateSwitchToPD_request(
      uint8_t obj_type, uint32_t timeout, float vel_ratio, float acc_ratio,
      const std::array<double, 7> &k, const std::array<double, 7> &d);

  std::shared_ptr<StateSwitchToPosition::Response>
  send_StateSwitchToPosition_request(uint8_t obj_type, uint32_t timeout,
                                     float vel_ratio, float acc_ratio);

  //------------------------------------------------------------------------------
  void publishSetJointPosPDCmd(const RuntimeSetJointPosPDCmd &msg) {
    publisher_->publish(msg);
  }

  void publishSetJointPosCmd(const RuntimeSetJointPosCmd &msg) {
    publisher1_->publish(msg);
  }

  void publishSetTerminalArm0CanFDData(const TerminalArm0CanFDSetData &msg) {
    publisher2_->publish(msg);
  }

  void publishSetTerminalArm1CanFDData(const TerminalArm1CanFDSetData &msg) {
    publisher3_->publish(msg);
  }

  void publishSetRuntimeToolKD(const RuntimeSetToolKD &msg) {
    publisher4_->publish(msg);
  }

  //------------------------------------------------------------------------------------------
 private:
  void handle_RobotRT_callback(RobotRT::SharedPtr msg);

  void handle_RobotSG_callback(RobotSG::SharedPtr msg);

  void handle_StateArm0_callback(FXStateTypeArm0::SharedPtr msg);
  void handle_StateArm1_callback(FXStateTypeArm1::SharedPtr msg);

  void handle_TerminalArm0CanFDGet_callback(
      TerminalArm0CanFDGetData::SharedPtr msg);
  void handle_TerminalArm1CanFDGet_callback(
      TerminalArm1CanFDGetData::SharedPtr msg);

  void setStateArm0(FXStateType newState) {
    stateArm0_.store(newState, std::memory_order_release);  // 释放语义
  }
  void setStateArm1(FXStateType newState) {
    stateArm1_.store(newState, std::memory_order_release);  // 释放语义
  }
};
