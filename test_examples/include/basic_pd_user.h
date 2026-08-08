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

  rclcpp::Publisher<RuntimeSetJointPosPDCmd>::SharedPtr publisher_;
  rclcpp::Publisher<RuntimeSetJointPosCmd>::SharedPtr publisher1_;

  RobotSG::SharedPtr sg_ptr_ = nullptr;  ///< Status group feedback
  RobotRT::SharedPtr rt_ptr_ = nullptr;  ///< Real-time feedback

  std::atomic<bool> bSystemLinked_{false};
  std::atomic<FXStateType> stateArm0_{FXStateType::FX_STATE_UNKNOWN};

  std::mutex mtxSG_;
  std::mutex mtxRT_;

  rclcpp::Executor *executor_ = nullptr;

 public:
  void setExecutor(rclcpp::Executor *exec) { executor_ = exec; }

  void setSystemLinked(bool bLinked) { bSystemLinked_.store(bLinked); }

  std::shared_ptr<const RobotSG> getRobotSG() const { return sg_ptr_; }
  std::shared_ptr<const RobotRT> getRobotRT() const { return rt_ptr_; }

  bool getSystemLinked() const {
    return bSystemLinked_.load(std::memory_order_acquire);
  }
  FXStateType getStateArm0() const {
    return stateArm0_.load(std::memory_order_acquire);
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

  //------------------------------------------------------------------------------------------
 private:
  void handle_RobotRT_callback(RobotRT::SharedPtr msg);

  void handle_RobotSG_callback(RobotSG::SharedPtr msg);

  void handle_StateArm0_callback(FXStateTypeArm0::SharedPtr msg);

  void setStateArm0(FXStateType newState) {
    stateArm0_.store(newState, std::memory_order_release);  // 释放语义
  }
};
