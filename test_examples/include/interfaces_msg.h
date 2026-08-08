#include <robot_data_interfaces/msg/arm0_hand_get_data.hpp>
#include <robot_data_interfaces/msg/arm0_hand_set_data.hpp>
#include <robot_data_interfaces/msg/arm1_hand_get_data.hpp>
#include <robot_data_interfaces/msg/arm1_hand_set_data.hpp>
#include <robot_data_interfaces/msg/arm_get.hpp>
#include <robot_data_interfaces/msg/arm_in.hpp>
#include <robot_data_interfaces/msg/arm_out.hpp>
#include <robot_data_interfaces/msg/arm_rt.hpp>
#include <robot_data_interfaces/msg/arm_set.hpp>
#include <robot_data_interfaces/msg/arm_sg.hpp>
#include <robot_data_interfaces/msg/body_get.hpp>
#include <robot_data_interfaces/msg/body_in.hpp>
#include <robot_data_interfaces/msg/body_out.hpp>
#include <robot_data_interfaces/msg/body_rt.hpp>
#include <robot_data_interfaces/msg/body_set.hpp>
#include <robot_data_interfaces/msg/body_sg.hpp>
#include <robot_data_interfaces/msg/cart_kd.hpp>
#include <robot_data_interfaces/msg/device_rt.hpp>
#include <robot_data_interfaces/msg/fbk_get_system_msg.hpp>
#include <robot_data_interfaces/msg/fx_state_type_arm0.hpp>
#include <robot_data_interfaces/msg/fx_state_type_arm1.hpp>
#include <robot_data_interfaces/msg/fx_state_type_body.hpp>
#include <robot_data_interfaces/msg/fx_state_type_head.hpp>
#include <robot_data_interfaces/msg/fx_state_type_lift.hpp>
#include <robot_data_interfaces/msg/hand_get.hpp>
#include <robot_data_interfaces/msg/hand_in.hpp>
#include <robot_data_interfaces/msg/hand_out.hpp>
#include <robot_data_interfaces/msg/hand_rt.hpp>
#include <robot_data_interfaces/msg/hand_set.hpp>
#include <robot_data_interfaces/msg/hand_sg.hpp>
#include <robot_data_interfaces/msg/head_get.hpp>
#include <robot_data_interfaces/msg/head_in.hpp>
#include <robot_data_interfaces/msg/head_out.hpp>
#include <robot_data_interfaces/msg/head_rt.hpp>
#include <robot_data_interfaces/msg/head_set.hpp>
#include <robot_data_interfaces/msg/head_sg.hpp>
#include <robot_data_interfaces/msg/joint_kd.hpp>
#include <robot_data_interfaces/msg/joint_pos_cmd.hpp>
#include <robot_data_interfaces/msg/lift_get.hpp>
#include <robot_data_interfaces/msg/lift_in.hpp>
#include <robot_data_interfaces/msg/lift_out.hpp>
#include <robot_data_interfaces/msg/lift_rt.hpp>
#include <robot_data_interfaces/msg/lift_set.hpp>
#include <robot_data_interfaces/msg/lift_sg.hpp>
#include <robot_data_interfaces/msg/robot_rt.hpp>
#include <robot_data_interfaces/msg/robot_sg.hpp>
#include <robot_data_interfaces/msg/rosidl_generator_cpp__visibility_control.hpp>
#include <robot_data_interfaces/msg/runtime_set_joint_d.hpp>
#include <robot_data_interfaces/msg/runtime_set_joint_k.hpp>
#include <robot_data_interfaces/msg/runtime_set_joint_kd.hpp>
#include <robot_data_interfaces/msg/runtime_set_joint_mit_cmd.hpp>
#include <robot_data_interfaces/msg/runtime_set_joint_pos_cmd.hpp>
#include <robot_data_interfaces/msg/runtime_set_joint_pos_pd_cmd.hpp>
#include <robot_data_interfaces/msg/runtime_set_speed_ratio.hpp>
#include <robot_data_interfaces/msg/runtime_set_tag.hpp>
#include <robot_data_interfaces/msg/runtime_set_tool_d.hpp>
#include <robot_data_interfaces/msg/runtime_set_tool_k.hpp>
#include <robot_data_interfaces/msg/runtime_set_tool_kd.hpp>
#include <robot_data_interfaces/msg/runtime_set_torque_ctrl.hpp>
#include <robot_data_interfaces/msg/runtime_set_vel_ratio.hpp>
#include <robot_data_interfaces/msg/runtime_stop_traj.hpp>
#include <robot_data_interfaces/msg/state_ctr.hpp>
#include <robot_data_interfaces/msg/system_rt.hpp>
#include <robot_data_interfaces/msg/terminal_arm0_can_fd_set_data.hpp>
#include <robot_data_interfaces/msg/terminal_arm0_chn_can_fd_get_data.hpp>
#include <robot_data_interfaces/msg/terminal_arm1_can_fd_set_data.hpp>
#include <robot_data_interfaces/msg/terminal_arm1_chn_can_fd_get_data.hpp>

using Arm0HandGetData = robot_data_interfaces::msg::Arm0HandGetData;
using Arm0HandSetData = robot_data_interfaces::msg::Arm0HandSetData;
using Arm1HandGetData = robot_data_interfaces::msg::Arm1HandGetData;
using Arm1HandSetData = robot_data_interfaces::msg::Arm1HandSetData;
using ArmGet = robot_data_interfaces::msg::ArmGet;
using ArmIn = robot_data_interfaces::msg::ArmIn;
using ArmOut = robot_data_interfaces::msg::ArmOut;
using ArmRT = robot_data_interfaces::msg::ArmRT;
using ArmSet = robot_data_interfaces::msg::ArmSet;
using ArmSG = robot_data_interfaces::msg::ArmSG;
using BodyGet = robot_data_interfaces::msg::BodyGet;
using BodyIn = robot_data_interfaces::msg::BodyIn;
using BodyOut = robot_data_interfaces::msg::BodyOut;
using BodyRT = robot_data_interfaces::msg::BodyRT;
using BodySet = robot_data_interfaces::msg::BodySet;
using BodySG = robot_data_interfaces::msg::BodySG;
using CartKD = robot_data_interfaces::msg::CartKD;
using DeviceRT = robot_data_interfaces::msg::DeviceRT;
using FbkGetSystemMsg = robot_data_interfaces::msg::FbkGetSystemMsg;
using FXStateTypeArm0 = robot_data_interfaces::msg::FXStateTypeArm0;
using FXStateTypeArm1 = robot_data_interfaces::msg::FXStateTypeArm1;
using FXStateTypeBody = robot_data_interfaces::msg::FXStateTypeBody;
using FXStateTypeHead = robot_data_interfaces::msg::FXStateTypeHead;
using FXStateTypeLift = robot_data_interfaces::msg::FXStateTypeLift;
using HandGet = robot_data_interfaces::msg::HandGet;
using HandIn = robot_data_interfaces::msg::HandIn;
using HandOut = robot_data_interfaces::msg::HandOut;
using HandRT = robot_data_interfaces::msg::HandRT;
using HandSet = robot_data_interfaces::msg::HandSet;
using HandSG = robot_data_interfaces::msg::HandSG;
using HeadGet = robot_data_interfaces::msg::HeadGet;
using HeadIn = robot_data_interfaces::msg::HeadIn;
using HeadOut = robot_data_interfaces::msg::HeadOut;
using HeadRT = robot_data_interfaces::msg::HeadRT;
using HeadSet = robot_data_interfaces::msg::HeadSet;
using HeadSG = robot_data_interfaces::msg::HeadSG;
using JointKD = robot_data_interfaces::msg::JointKD;
using JointPosCmd = robot_data_interfaces::msg::JointPosCmd;
using LiftGet = robot_data_interfaces::msg::LiftGet;
using LiftIn = robot_data_interfaces::msg::LiftIn;
using LiftOut = robot_data_interfaces::msg::LiftOut;
using LiftRT = robot_data_interfaces::msg::LiftRT;
using LiftSet = robot_data_interfaces::msg::LiftSet;
using LiftSG = robot_data_interfaces::msg::LiftSG;
using RobotRT = robot_data_interfaces::msg::RobotRT;
using RobotSG = robot_data_interfaces::msg::RobotSG;
using RuntimeSetJointD = robot_data_interfaces::msg::RuntimeSetJointD;
using RuntimeSetJointKD = robot_data_interfaces::msg::RuntimeSetJointKD;
using RuntimeSetJointK = robot_data_interfaces::msg::RuntimeSetJointK;
using RuntimeSetJointMITCmd = robot_data_interfaces::msg::RuntimeSetJointMITCmd;
using RuntimeSetJointPosCmd = robot_data_interfaces::msg::RuntimeSetJointPosCmd;
using RuntimeSetJointPosPDCmd =
    robot_data_interfaces::msg::RuntimeSetJointPosPDCmd;
using RuntimeSetSpeedRatio = robot_data_interfaces::msg::RuntimeSetSpeedRatio;
using RuntimeSetTag = robot_data_interfaces::msg::RuntimeSetTag;
using RuntimeSetToolD = robot_data_interfaces::msg::RuntimeSetToolD;
using RuntimeSetToolKD = robot_data_interfaces::msg::RuntimeSetToolKD;
using RuntimeSetToolK = robot_data_interfaces::msg::RuntimeSetToolK;
using RuntimeSetTorqueCtrl = robot_data_interfaces::msg::RuntimeSetTorqueCtrl;
using RuntimeSetVelRatio = robot_data_interfaces::msg::RuntimeSetVelRatio;
using RuntimeStopTraj = robot_data_interfaces::msg::RuntimeStopTraj;
using SystemRT = robot_data_interfaces::msg::SystemRT;
using TerminalArm0CanFDSetData =
    robot_data_interfaces::msg::TerminalArm0CanFDSetData;
using TerminalArm0CHNCanFDGetData =
    robot_data_interfaces::msg::TerminalArm0CHNCanFDGetData;
using TerminalArm1CanFDSetData =
    robot_data_interfaces::msg::TerminalArm1CanFDSetData;
using TerminalArm1CHNCanFDGetData =
    robot_data_interfaces::msg::TerminalArm1CHNCanFDGetData;