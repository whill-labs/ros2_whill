// Copyright (c) 2024 WHILL, Inc.
// Released under the MIT license
// https://opensource.org/licenses/mit-license.php

/**
 * @file    whill_node.cpp
 * @brief   Functions of the node that works as WHILL
 */
#include "whill_driver/whill_node.hpp"

#include <chrono>
#include <thread>

using namespace std::chrono_literals;
using namespace std::placeholders;

namespace whill_driver
{

/**
 * The default serial port name
 */
const std::string kDefaultPortName = "/dev/ttyUSB0";

/**
 * The default publish interval [ms]
 */
constexpr uint16_t kDefaulPpublishIntervalMs = 500;

void WhillNode::Initialize()
{
  // load parameters
  declare_parameter("port_name", kDefaultPortName);
  std::string port_name = get_parameter("port_name").as_string();
  whill_ = std::make_shared<model_cr2::Whill>(port_name);

  declare_parameter("publish_interval_ms", kDefaulPpublishIntervalMs);
  int publish_interval_ms = get_parameter("publish_interval_ms").as_int();
  auto publish_duration = std::chrono::duration<double, std::milli>(publish_interval_ms);

  // publish
  states_model_cr2_pub_ = this->create_publisher<whill_msgs::msg::ModelCr2State>(
    "/whill/states/model_cr2", 10);
  states_model_cr2_timer_ =
    this->create_wall_timer(publish_duration, std::bind(&WhillNode::OnStatesModelCr2Timer, this));

  // subscription
  controller_joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
    "/whill/controller/joy", 10, std::bind(&WhillNode::OnControllerJoy, this, _1));
  controller_cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "/whill/controller/cmd_vel", 10, std::bind(&WhillNode::OnControllerCmdVel, this, _1));

  // service
  set_power_srv_ = this->create_service<whill_msgs::srv::SetPower>(
    "/whill/set_power_srv", std::bind(&WhillNode::OnSetPowerSrv, this, _1, _2, _3));
  set_speed_profile_srv_ = this->create_service<whill_msgs::srv::SetSpeedProfile>(
    "/whill/set_speed_profile_srv", std::bind(&WhillNode::OnSetSpeedProfileSrv, this, _1, _2, _3));
  set_battery_voltage_out_srv_ = this->create_service<whill_msgs::srv::SetBatteryVoltageOut>(
    "/whill/set_battery_voltage_out_srv",
    std::bind(&WhillNode::OnSetBatteryVoltageOutSrv, this, _1, _2, _3));
  set_battery_saving_srv_ = this->create_service<whill_msgs::srv::SetBatterySaving>(
    "/whill/set_battery_saving_srv", std::bind(&WhillNode::OnSetBatterySavingSrv, this, _1, _2, _3));
  set_joystick_lock_srv_ = this->create_service<whill_msgs::srv::SetJoystickLock>(
    "/whill/set_joystick_lock_srv", std::bind(&WhillNode::OnSetJoystickLockSrv, this, _1, _2, _3));

  // start sending WHILL State Dataset1
  whill_->SendStartSendingDataCommand(
    publish_interval_ms, model_cr2::kDatasetNumber1,
    model_cr2::kSpeedMode0);
}

WhillNode::WhillNode()
: Node("whill")
{
  this->Initialize();
}

WhillNode::WhillNode(const rclcpp::NodeOptions & options)
: Node("whill_node", options)
{
  this->Initialize();
}

WhillNode::~WhillNode()
{
}

void WhillNode::OnStatesModelCr2Timer()
{
  auto msg = std::make_shared<whill_msgs::msg::ModelCr2State>();
  if (whill_->ReceiveDataset1(msg) < 1) {return;}
  states_model_cr2_pub_->publish(*msg);
}

void WhillNode::OnControllerJoy(const sensor_msgs::msg::Joy::SharedPtr joy)
{
  whill_->SendSetJoystickCommand(ConvertToWhillJoy(joy->axes[1]), ConvertToWhillJoy(-joy->axes[0]));
  RCLCPP_INFO(this->get_logger(), "[Joy] front:['%f'],  right:['%f']", joy->axes[1], joy->axes[0]);
}

void WhillNode::OnControllerCmdVel(const geometry_msgs::msg::Twist::SharedPtr cmd_vel)
{
  // [m/s] to [km/h]: *3.6
  // SetVelocityCommand takes command unit (0.004 km/h): *250
  int linear = cmd_vel->linear.x * 900;

  // wheel_tread: 0.496
  // [rad/s] to [km/h]: *wheel_tread*3.6
  // SetVelocityCommand takes command unit (0.004 km/h): *250
  // The direction of rotation is reversed in ROS and SetVelocityCommand
  int angular = cmd_vel->angular.z * -446.4;
  whill_->SendSetVelocityCommand(linear, angular);
  RCLCPP_INFO(
    this->get_logger(), "[CmdVel] linear:['%f'], angular:['%f']", cmd_vel->linear.x,
    cmd_vel->angular.z);
}

void WhillNode::OnSetPowerSrv(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<whill_msgs::srv::SetPower::Request> request,
  const std::shared_ptr<whill_msgs::srv::SetPower::Response> response)
{
  (void)request_header;
  switch (request->p0) {
    case 0:
      whill_->SetPowerOff();
      RCLCPP_INFO(this->get_logger(), "WHILL power off");
      response->result = 1;
      break;
    case 1:
      whill_->SetPowerOn();
      std::this_thread::sleep_for(10ms);
      whill_->SetPowerOn();
      std::this_thread::sleep_for(2ms);
      RCLCPP_INFO(this->get_logger(), "WHILL power on");
      response->result = 1;
      break;
    default:
      RCLCPP_WARN(this->get_logger(), "p0 must be assinged 0 or 1");
      response->result = -1;
      break;
  }
}


void WhillNode::OnSetSpeedProfileSrv(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<whill_msgs::srv::SetSpeedProfile::Request> request,
  const std::shared_ptr<whill_msgs::srv::SetSpeedProfile::Response> response)
{
  (void)request_header;
  response->result = -1;

  uint8_t s1 = uint8_t(request->s1);
  uint8_t fm1 = uint8_t(request->fm1);
  uint8_t fa1 = uint8_t(request->fa1);
  uint8_t fd1 = uint8_t(request->fd1);
  uint8_t rm1 = uint8_t(request->rm1);
  uint8_t ra1 = uint8_t(request->ra1);
  uint8_t rd1 = uint8_t(request->rd1);
  uint8_t tm1 = uint8_t(request->tm1);
  uint8_t ta1 = uint8_t(request->ta1);
  uint8_t td1 = uint8_t(request->td1);
  if (IsOutside(s1, 0, 5)) {
    RCLCPP_WARN(this->get_logger(), "s1 must be assingned between 0 - 5");
    return;
  }
  if (IsOutside(fm1, 8, 60)) {
    RCLCPP_WARN(this->get_logger(), "fm1 must be assingned between 8 - 60");
    return;
  }
  if (IsOutside(fa1, 10, 90)) {
    RCLCPP_WARN(this->get_logger(), "fa1 must be assingned between 10 - 90");
    return;
  }
  if (IsOutside(fd1, 40, 160)) {
    RCLCPP_WARN(this->get_logger(), "fd1 must be assingned between 40 - 160");
    return;
  }
  if (IsOutside(rm1, 8, 30)) {
    RCLCPP_WARN(this->get_logger(), "rm1 must be assingned between 8 - 30");
    return;
  }
  if (IsOutside(ra1, 10, 50)) {
    RCLCPP_WARN(this->get_logger(), "ra1 must be assingned between 10 - 50");
    return;
  }
  if (IsOutside(rd1, 40, 90)) {
    RCLCPP_WARN(this->get_logger(), "rd1 must be assingned between 40 - 90");
    return;
  }
  if (IsOutside(tm1, 8, 35)) {
    RCLCPP_WARN(this->get_logger(), "tm1 must be assingned between 8 - 35");
    return;
  }
  if (IsOutside(ta1, 10, 60)) {
    RCLCPP_WARN(this->get_logger(), "ta1 must be assingned between 10 - 60");
    return;
  }
  if (IsOutside(td1, 40, 160)) {
    RCLCPP_WARN(this->get_logger(), "td1 must be assingned between 40 - 160");
    return;
  }

  RCLCPP_INFO(this->get_logger(), "Speed profile is set");
  whill_->SendSetSpeedProfileCommand(s1, fm1, fa1, fd1, rm1, ra1, rd1, tm1, ta1, td1);
  response->result = 1;
}

void WhillNode::OnSetBatteryVoltageOutSrv(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<whill_msgs::srv::SetBatteryVoltageOut::Request> request,
  const std::shared_ptr<whill_msgs::srv::SetBatteryVoltageOut::Response> response)
{
  (void)request_header;
  switch (request->v0) {
    case 0:
      whill_->SendSetBatteryVoltageOutCommand(0);
      RCLCPP_INFO(this->get_logger(), "battery voltage out: disable");
      response->result = 1;
      break;
    case 1:
      whill_->SendSetBatteryVoltageOutCommand(1);
      RCLCPP_INFO(this->get_logger(), "battery voltage out: enable");
      response->result = 1;
      break;
    default:
      RCLCPP_WARN(this->get_logger(), "v0 must be assigned 0 or 1");
      response->result = -1;
      break;
  }
  RCLCPP_WARN(this->get_logger(), "BatteryVoltageOut command is not available on Model CR2!!");
}

void WhillNode::OnSetBatterySavingSrv(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<whill_msgs::srv::SetBatterySaving::Request> request,
  const std::shared_ptr<whill_msgs::srv::SetBatterySaving::Response> response)
{
  (void)request_header;
  response->result = -1;

  uint8_t l0 = uint8_t(request->l0);
  uint8_t b0 = uint8_t(request->b0);
  if (IsOutside(l0, 1, 90)) {
    RCLCPP_WARN(this->get_logger(), "l0 must be assingned between 1 - 90");
    return;
  }
  if (IsOutside(b0, 0, 1)) {
    RCLCPP_WARN(this->get_logger(), "b0 must be assingned between 0 - 1");
    return;
  }

  RCLCPP_INFO(this->get_logger(), "Battery saving settings are set (l0: %d, b0: %d)", l0, b0);
  whill_->SendSetBatterySavingCommand(l0, b0);
  response->result = 1;
}

void WhillNode::OnSetJoystickLockSrv(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<whill_msgs::srv::SetJoystickLock::Request> request,
  const std::shared_ptr<whill_msgs::srv::SetJoystickLock::Response> response)
{
  (void)request_header;
  switch (request->d0) {
    case 0:
      whill_->SendSetJoystickLockCommand(false);
      RCLCPP_INFO(this->get_logger(), "joystick lock: resume (unlock)");
      response->result = 1;
      break;
    case 1:
      whill_->SendSetJoystickLockCommand(true);
      RCLCPP_INFO(this->get_logger(), "joystick lock: pause (lock)");
      response->result = 1;
      break;
    default:
      RCLCPP_WARN(this->get_logger(), "d0 must be assigned 0 (resume) or 1 (pause)");
      response->result = -1;
      break;
  }
}

/**
 * The function ConvertToWhillJoy converts a float value to an integer value between -100 and 100.
 *
 * @param raw_joy The `ConvertToWhillJoy` function takes a float value `raw_joy` as input, which
 * represents the raw joystick input. The function then converts this raw joystick input to an integer
 * value between -100 and 100, ensuring that the value does not exceed these limits.
 *
 * @return The function `ConvertToWhillJoy` takes a float value `raw_joy`, multiplies it by 100, and
 * then checks if the result is less than -100 or greater than 100. If the result is less than -100, it
 * returns -100. If the result is greater than 100, it returns 100. Otherwise, it returns the
 * calculated value `joy`.
 */
int WhillNode::ConvertToWhillJoy(float raw_joy)
{
  int joy = (int)(raw_joy * 100.0f);
  if (joy < -100) {return -100;}
  if (joy > 100) {return 100;}
  return joy;
}

/**
 * The function determines if a target value is outside the range defined by two other values.
 *
 * @param[in] target The value that we want to check if it is outside the range defined by end1 and end2.
 * @param[in] end1 The value of the first end point.
 * @param[in] end2 The "end2" parameter represents one end of a range.
 *
 * @return a boolean value. It returns true if the target value is outside the range defined by the
 * end1 and end2 values, and false otherwise.
 */
bool WhillNode::IsOutside(uint8_t target, uint8_t end1, uint8_t end2)
{
  uint8_t bottom = (end1 < end2) ? end1 : end2;
  uint8_t top = (end1 < end2) ? end2 : end1;
  if (target > top) {return true;}
  if (target < bottom) {return true;}
  return false;
}

}  // namespace whill_driver

RCLCPP_COMPONENTS_REGISTER_NODE(whill_driver::WhillNode)
