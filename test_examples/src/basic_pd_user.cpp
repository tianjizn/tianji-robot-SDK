#include "basic_pd_user.h"

BasicPDUser::BasicPDUser() : Node("basic_pd_user") {
  client0_ = this->create_client<SystemLink>("system_link");
  clientUnlink_ = this->create_client<SystemUnlink>("system_unlink");
  client1_ = this->create_client<SystemGetSDKVersion>("system_get_sdk_version");
  client2_ = this->create_client<SystemGetControllerVersion>(
      "system_get_controller_version");

  client4_ = this->create_client<StateResetError>("state_reset_error");
  client5_ = this->create_client<StateSwitchToIdle>("state_switch_to_idle");
  client6_ = this->create_client<ConfigSetPDCmdCycleTime>(
      "config_set_pd_cmd_cycle_time");
  client7_ = this->create_client<StateSwitchToPD>("state_switch_to_pd");
  client8_ =
      this->create_client<StateSwitchToPosition>("state_switch_to_position");
  //---------------------------------------------------------------------------------------
  subscription0_ = this->create_subscription<RobotRT>(
      "/robot/robot_rt", 500,
      std::bind(&BasicPDUser::handle_RobotRT_callback, this,
                std::placeholders::_1));

  subscription1_ = this->create_subscription<RobotSG>(
      "/robot/robot_sg", 250,
      std::bind(&BasicPDUser::handle_RobotSG_callback, this,
                std::placeholders::_1));

  subscription2_ = this->create_subscription<FXStateTypeArm0>(
      "/robot/state_type_arm0", 100,
      std::bind(&BasicPDUser::handle_StateArm0_callback, this,
                std::placeholders::_1));

  subscription3_ = this->create_subscription<TerminalArm0CanFDGetData>(
      "/robot/terminal_arm0_can_fd_get_data", 100,
      std::bind(&BasicPDUser::handle_TerminalArm0CanFDGet_callback, this,
                std::placeholders::_1));

  subscription4_ = this->create_subscription<TerminalArm1CanFDGetData>(
      "/robot/terminal_arm1_can_fd_get_data", 100,
      std::bind(&BasicPDUser::handle_TerminalArm1CanFDGet_callback, this,
                std::placeholders::_1));
  subscription5_ = this->create_subscription<FXStateTypeArm1>(
      "/robot/state_type_arm1", 100,
      std::bind(&BasicPDUser::handle_StateArm1_callback, this,
                std::placeholders::_1));
  //---------------------------------------------------------------------------------------
  publisher_ = this->create_publisher<RuntimeSetJointPosPDCmd>(
      "/robot/runtime_set_joint_pos_pd_cmd", 100);

  publisher1_ = this->create_publisher<RuntimeSetJointPosCmd>(
      "/robot/runtime_set_joint_pos_cmd", 100);

  publisher2_ = this->create_publisher<TerminalArm0CanFDSetData>(
      "/robot/terminal_arm0_can_fd_set_data", 100);

  publisher3_ = this->create_publisher<TerminalArm1CanFDSetData>(
      "/robot/terminal_arm1_can_fd_set_data", 100);

  publisher4_ = this->create_publisher<RuntimeSetToolKD>(
      "/robot/runtime_set_tool_kd", 10);

  RCLCPP_INFO(this->get_logger(), "BasicPD User ready!");
  // timer_ = this->create_wall_timer(
  //     std::chrono::milliseconds(10),
  //     std::bind(&BasicPDUser::timer_callback, this));
}

std::shared_ptr<SystemLink::Response> BasicPDUser::send_SystemLink_request(
    uint8_t ip1, uint8_t ip2, uint8_t ip3, uint8_t ip4, uint32_t log_level) {
  while (!client0_->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for service");
      return nullptr;
    }
    RCLCPP_INFO(this->get_logger(), "Waiting for service 'system_link'...");
  }

  auto request = std::make_shared<SystemLink::Request>();
  request->ip1 = ip1;
  request->ip2 = ip2;
  request->ip3 = ip3;
  request->ip4 = ip4;
  request->log_level = log_level;

  auto future = client0_->async_send_request(request);

  if (executor_ == nullptr) {
    RCLCPP_ERROR(this->get_logger(), "Executor not set!");
    return nullptr;
  }

  if (executor_->spin_until_future_complete(future, std::chrono::seconds(1)) ==
      rclcpp::FutureReturnCode::SUCCESS) {
    return future.get();
  } else {
    RCLCPP_ERROR(this->get_logger(), "Service call failed");
    return nullptr;
  }
}

std::shared_ptr<SystemUnlink::Response>
BasicPDUser::send_SystemUnlink_request() {
  bool service_ready = false;
  for (int i = 0; i < 3; ++i) {
    if (clientUnlink_->wait_for_service(std::chrono::seconds(1))) {
      service_ready = true;
      break;
    }
    RCLCPP_WARN(this->get_logger(),
                "Waiting for service 'SystemUnlink'... (attempt %d/3)", i + 1);
  }

  if (!service_ready) {
    RCLCPP_ERROR(this->get_logger(),
                 "Service 'SystemUnlink' not available after 3 attempts");
    return nullptr;
  }

  auto request = std::make_shared<SystemUnlink::Request>();
  auto future = clientUnlink_->async_send_request(request);
  if (executor_ == nullptr) {
    RCLCPP_ERROR(this->get_logger(), "Executor not set!");
    return nullptr;
  }

  if (executor_->spin_until_future_complete(future, std::chrono::seconds(1)) ==
      rclcpp::FutureReturnCode::SUCCESS) {
    return future.get();
  } else {
    RCLCPP_ERROR(this->get_logger(), "SystemUnlink Service call failed");
    return nullptr;
  }
}

std::shared_ptr<SystemGetSDKVersion::Response>
BasicPDUser::send_SystemGetSDKVersion_request() {
  bool service_ready = false;
  for (int i = 0; i < 3; ++i) {
    if (client1_->wait_for_service(std::chrono::seconds(1))) {
      service_ready = true;
      break;
    }
    RCLCPP_WARN(this->get_logger(),
                "Waiting for service 'SystemGetSDKVersion'... (attempt %d/3)",
                i + 1);
  }

  if (!service_ready) {
    RCLCPP_ERROR(
        this->get_logger(),
        "Service 'SystemGetSDKVersion' not available after 3 attempts");
    return nullptr;
  }

  auto request = std::make_shared<SystemGetSDKVersion::Request>();
  auto future = client1_->async_send_request(request);
  if (executor_ == nullptr) {
    RCLCPP_ERROR(this->get_logger(), "Executor not set!");
    return nullptr;
  }

  if (executor_->spin_until_future_complete(future, std::chrono::seconds(1)) ==
      rclcpp::FutureReturnCode::SUCCESS) {
    return future.get();
  } else {
    RCLCPP_ERROR(this->get_logger(), "SystemGetSDKVersion Service call failed");
    return nullptr;
  }
}

std::shared_ptr<SystemGetControllerVersion::Response>
BasicPDUser::send_SystemGetControllerVersion_request() {
  bool service_ready = false;
  for (int i = 0; i < 3; ++i) {
    if (client2_->wait_for_service(std::chrono::seconds(1))) {
      service_ready = true;
      break;
    }
    RCLCPP_WARN(
        this->get_logger(),
        "Waiting for service 'SystemGetControllerVersion'... (attempt %d/3)",
        i + 1);
  }

  if (!service_ready) {
    RCLCPP_ERROR(
        this->get_logger(),
        "Service 'SystemGetControllerVersion' not available after 3 attempts");
    return nullptr;
  }

  auto request = std::make_shared<SystemGetControllerVersion::Request>();
  auto future = client2_->async_send_request(request);
  if (executor_ == nullptr) {
    RCLCPP_ERROR(this->get_logger(), "Executor not set!");
    return nullptr;
  }

  if (executor_->spin_until_future_complete(future, std::chrono::seconds(1)) ==
      rclcpp::FutureReturnCode::SUCCESS) {
    return future.get();
  } else {
    RCLCPP_ERROR(this->get_logger(),
                 "SystemGetControllerVersion Service call failed");
    return nullptr;
  }
}

std::shared_ptr<StateResetError::Response>
BasicPDUser::send_StateResetError_request(uint8_t obj_type, uint32_t timeout) {
  bool service_ready = false;
  for (int i = 0; i < 3; ++i) {
    if (client4_->wait_for_service(std::chrono::seconds(1))) {
      service_ready = true;
      break;
    }
    RCLCPP_WARN(this->get_logger(),
                "Waiting for service 'StateResetError'... (attempt %d/3)",
                i + 1);
  }

  if (!service_ready) {
    RCLCPP_ERROR(this->get_logger(),
                 "Service 'StateResetError' not available after 3 attempts");
    return nullptr;
  }

  auto request = std::make_shared<StateResetError::Request>();
  request->obj_type = obj_type;
  request->timeout = timeout;

  auto future = client4_->async_send_request(request);
  if (executor_ == nullptr) {
    RCLCPP_ERROR(this->get_logger(), "Executor not set!");
    return nullptr;
  }

  if (executor_->spin_until_future_complete(
          future, std::chrono::seconds(timeout / 1000)) ==
      rclcpp::FutureReturnCode::SUCCESS) {
    return future.get();
  } else {
    RCLCPP_ERROR(this->get_logger(), "StateResetError service call failed");
    return nullptr;
  }
}

std::shared_ptr<StateSwitchToIdle::Response>
BasicPDUser::send_StateSwitchToIdle_request(uint8_t obj_type,
                                            uint32_t timeout) {
  bool service_ready = false;
  for (int i = 0; i < 3; ++i) {
    if (client5_->wait_for_service(std::chrono::seconds(1))) {
      service_ready = true;
      break;
    }
    RCLCPP_WARN(this->get_logger(),
                "Waiting for service 'StateSwitchToIdle'... (attempt %d/3)",
                i + 1);
  }

  if (!service_ready) {
    RCLCPP_ERROR(this->get_logger(),
                 "Service 'StateSwitchToIdle' not available after 3 attempts");
    return nullptr;
  }

  auto request = std::make_shared<StateSwitchToIdle::Request>();
  request->obj_type = obj_type;
  request->timeout = timeout;

  auto future = client5_->async_send_request(request);
  if (executor_ == nullptr) {
    RCLCPP_ERROR(this->get_logger(), "Executor not set!");
    return nullptr;
  }

  if (executor_->spin_until_future_complete(
          future, std::chrono::seconds(timeout / 1000)) ==
      rclcpp::FutureReturnCode::SUCCESS) {
    return future.get();
  } else {
    RCLCPP_ERROR(this->get_logger(), "StateSwitchToIdle service call failed");
    return nullptr;
  }
}

std::shared_ptr<ConfigSetPDCmdCycleTime::Response>
BasicPDUser::send_ConfigSetPDCmdCycleTime_request(int32_t cycle_time) {
  bool service_ready = false;
  for (int i = 0; i < 3; ++i) {
    if (client6_->wait_for_service(std::chrono::seconds(1))) {
      service_ready = true;
      break;
    }
    RCLCPP_WARN(
        this->get_logger(),
        "Waiting for service 'ConfigSetPDCmdCycleTime'... (attempt %d/3)",
        i + 1);
  }

  if (!service_ready) {
    RCLCPP_ERROR(
        this->get_logger(),
        "Service 'ConfigSetPDCmdCycleTime' not available after 3 attempts");
    return nullptr;
  }

  auto request = std::make_shared<ConfigSetPDCmdCycleTime::Request>();
  request->cycle_time = cycle_time;

  auto future = client6_->async_send_request(request);
  if (executor_ == nullptr) {
    RCLCPP_ERROR(this->get_logger(), "Executor not set!");
    return nullptr;
  }

  if (executor_->spin_until_future_complete(future, std::chrono::seconds(1)) ==
      rclcpp::FutureReturnCode::SUCCESS) {
    return future.get();
  } else {
    RCLCPP_ERROR(this->get_logger(),
                 "ConfigSetPDCmdCycleTime service call failed");
    return nullptr;
  }
}

std::shared_ptr<StateSwitchToPD::Response>
BasicPDUser::send_StateSwitchToPD_request(uint8_t obj_type, uint32_t timeout,
                                          float vel_ratio, float acc_ratio,
                                          const std::array<double, 7> &k,
                                          const std::array<double, 7> &d) {
  bool service_ready = false;
  for (int i = 0; i < 3; ++i) {
    if (client7_->wait_for_service(std::chrono::seconds(1))) {
      service_ready = true;
      break;
    }
    RCLCPP_WARN(this->get_logger(),
                "Waiting for service 'StateSwitchToPD'... (attempt %d/3)",
                i + 1);
  }

  if (!service_ready) {
    RCLCPP_ERROR(this->get_logger(),
                 "Service 'StateSwitchToPD' not available after 3 attempts");
    return nullptr;
  }

  auto request = std::make_shared<StateSwitchToPD::Request>();
  request->obj_type = obj_type;
  request->timeout = timeout;
  request->vel_ratio = vel_ratio;
  request->acc_ratio = acc_ratio;
  request->k = k;
  request->d = d;

  auto future = client7_->async_send_request(request);
  if (executor_ == nullptr) {
    RCLCPP_ERROR(this->get_logger(), "Executor not set!");
    return nullptr;
  }

  if (executor_->spin_until_future_complete(future, std::chrono::seconds(1)) ==
      rclcpp::FutureReturnCode::SUCCESS) {
    return future.get();
  } else {
    RCLCPP_ERROR(this->get_logger(), "StateSwitchToPD service call failed");
    return nullptr;
  }
}

std::shared_ptr<StateSwitchToPosition::Response>
BasicPDUser::send_StateSwitchToPosition_request(uint8_t obj_type,
                                                uint32_t timeout,
                                                float vel_ratio,
                                                float acc_ratio) {
  bool service_ready = false;
  for (int i = 0; i < 3; ++i) {
    if (client7_->wait_for_service(std::chrono::seconds(1))) {
      service_ready = true;
      break;
    }
    RCLCPP_WARN(this->get_logger(),
                "Waiting for service 'StateSwitchToPosition'... (attempt %d/3)",
                i + 1);
  }

  if (!service_ready) {
    RCLCPP_ERROR(
        this->get_logger(),
        "Service 'StateSwitchToPosition' not available after 3 attempts");
    return nullptr;
  }

  auto request = std::make_shared<StateSwitchToPosition::Request>();
  request->obj_type = obj_type;
  request->timeout = timeout;
  request->vel_ratio = vel_ratio;
  request->acc_ratio = acc_ratio;

  auto future = client8_->async_send_request(request);
  if (executor_ == nullptr) {
    RCLCPP_ERROR(this->get_logger(), "Executor not set!");
    return nullptr;
  }

  if (executor_->spin_until_future_complete(future, std::chrono::seconds(1)) ==
      rclcpp::FutureReturnCode::SUCCESS) {
    return future.get();
  } else {
    RCLCPP_ERROR(this->get_logger(), "StateSwitchToIdle service call failed");
    return nullptr;
  }
}

//----------------------------------------------------------------------------------------
void BasicPDUser::handle_RobotRT_callback(RobotRT::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(mtx_rt_);
  rt_ptr_ = std::move(msg);

  // printf("Get Topic RobotRT !\n");
}

void BasicPDUser::handle_RobotSG_callback(RobotSG::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(mtx_sg_);
  sg_ptr_ = std::move(msg);
  // printf("Get Topic RobotSG %d!\n", j++);
  // printf("Get Topic RobotSG !\n");
}

void BasicPDUser::handle_StateArm0_callback(FXStateTypeArm0::SharedPtr msg) {
  if (!msg) {
    return;
  }

  setStateArm0(static_cast<FXStateType>(msg->state_type));

  // printf("Get Topic StateArm0 %d!\n", getStateArm0());
}

void BasicPDUser::handle_StateArm1_callback(FXStateTypeArm1::SharedPtr msg) {
  if (!msg) {
    return;
  }

  setStateArm1(static_cast<FXStateType>(msg->state_type));

  // printf("Get Topic StateArm1 %d!\n", getStateArm1());
}

void BasicPDUser::handle_TerminalArm0CanFDGet_callback(
    TerminalArm0CanFDGetData::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(mtx_canfd0_);
  canFDarm0_ptr_ = std::move(msg);

  // printf("Get Topic TerminalArm0CanFDGetData !\n");
}

void BasicPDUser::handle_TerminalArm1CanFDGet_callback(
    TerminalArm1CanFDGetData::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(mtx_canfd1_);
  canFDarm1_ptr_ = std::move(msg);

  // printf("Get Topic TerminalArm1CanFDGetData !\n");
}