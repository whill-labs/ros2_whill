// Copyright (c) 2024 WHILL, Inc.
// Released under the MIT license
// https://opensource.org/licenses/mit-license.php

/**
 * @file    whill.cpp
 * @brief   Functions of WHILL Model CR2 domain
 */
#include "whill_driver/model_cr2/whill.hpp"

#include <math.h>

namespace whill_driver
{
namespace model_cr2
{

/**
 * The index of control commands
 */
enum CommandId : uint8_t
{
  kStartSendingData = 0x00,
  kStopSendingData,
  kSetPower,
  kSetJoystick,
  kSetSpeedProfile,
  kSetBatteryVoltageOut,
  kSetBatterySaving,
  kReserved,
  kSetVelocity,
  kReserved1,
  kSetJoystickLock,
  kMax
};

/**
 * The size of control commands
 */
enum CommandSize : uint8_t
{
  kStartSendingDataCommandSize = 6,
  kStopSendingDataCommandSize = 2,
  kSetPowerCommandSize = 3,
  kSetJoystickCommandSize = 5,
  kSetSpeedProfileCommandSize = 12,
  kSetBatteryVoltageOutCommandSize = 3,
  kSetBatterySavingCommandSize = 4,
  kReservedCommandSize,
  kSetVelocityCommandSize = 7,
  kSetJoystickLockCommandSize = 3,
  kMaxCommandSize = 16
};

/**
 * The value for U0 in SetJoyStick command
 */
enum SetJoyStickCommand : uint8_t
{
  kUserControlDisable = 0x00,
  kUserControlEnable = 0x01,
};

/**
 * The value for converting [0.001rad] to [rad]
 */
constexpr float kMotorAngleFactor = 0.001;

/**
 * The value for converting [0.004km/h] to [km/h]
 */
constexpr float kMotorSpeedFactor = 0.004;

/**
 * The size of the header (Protocol sign and Data length)
 */
constexpr uint8_t kHeaderSize = 2;

Whill::Whill(const std::string & port)
{
  port_ = std::make_shared<hardware::SerialPort>();
  if (port_->Open(port) < 0) {
    fprintf(stderr, "Can't initizalize UART to communicate with WHILL\n");
  }

  parser_ = std::make_shared<Parser>();
}

Whill::~Whill()
{
  port_->Close();
}

int Whill::SendStartSendingDataCommand(
  uint16_t interval_ms, DatasetNumber dataset_number,
  SpeedMode speed_mode)
{
  int idx = 0;
  uint8_t packet[kHeaderSize + CommandSize::kStartSendingDataCommandSize] = {0};

  packet[idx++] = kProtocolSign;
  packet[idx++] = CommandSize::kStartSendingDataCommandSize;
  packet[idx++] = CommandId::kStartSendingData;
  packet[idx++] = dataset_number;
  packet[idx++] = (uint8_t)(interval_ms >> 8);
  packet[idx++] = (uint8_t)interval_ms;
  packet[idx++] = speed_mode;
  packet[idx] = parser_->Checksum(packet, idx);
  return port_->Send(packet, sizeof(packet));
}

int Whill::SendStopSendingDataCommand()
{
  int idx = 0;
  uint8_t packet[kHeaderSize + CommandSize::kStopSendingDataCommandSize] = {0};

  packet[idx++] = kProtocolSign;
  packet[idx++] = CommandSize::kStopSendingDataCommandSize;
  packet[idx++] = CommandId::kStopSendingData;
  packet[idx] = parser_->Checksum(packet, idx);
  return port_->Send(packet, sizeof(packet));
}

int Whill::SendSetPower(bool turn_on)
{
  int idx = 0;
  uint8_t packet[kHeaderSize + CommandSize::kSetPowerCommandSize] = {0};

  packet[idx++] = kProtocolSign;
  packet[idx++] = CommandSize::kSetPowerCommandSize;
  packet[idx++] = CommandId::kSetPower;
  packet[idx++] = turn_on ? 0x01 : 0x00;
  packet[idx] = parser_->Checksum(packet, idx);
  return port_->Send(packet, sizeof(packet));
}

int Whill::SetPowerOn()
{
  return SendSetPower(true);
}

int Whill::SetPowerOff()
{
  return SendSetPower(false);
}

int Whill::SendSetJoystickCommand(uint8_t fb, uint8_t lr)
{
  int idx = 0;
  uint8_t packet[kHeaderSize + CommandSize::kSetJoystickCommandSize] = {0};

  packet[idx++] = kProtocolSign;
  packet[idx++] = CommandSize::kSetJoystickCommandSize;
  packet[idx++] = CommandId::kSetJoystick;
  packet[idx++] = kUserControlDisable;
  packet[idx++] = fb;
  packet[idx++] = lr;
  packet[idx] = parser_->Checksum(packet, idx);
  return port_->Send(packet, sizeof(packet));
}

int Whill::SendSetJoystickCommandWithLocal()
{
  int idx = 0;
  uint8_t packet[kHeaderSize + CommandSize::kSetJoystickCommandSize] = {0};

  packet[idx++] = kProtocolSign;
  packet[idx++] = CommandSize::kSetJoystickCommandSize;
  packet[idx++] = CommandId::kSetJoystick;
  packet[idx++] = kUserControlEnable;
  packet[idx++] = 0;
  packet[idx++] = 0;
  packet[idx] = parser_->Checksum(packet, idx);
  return port_->Send(packet, sizeof(packet));
}

int Whill::SendSetSpeedProfileCommand(
  uint8_t s1, uint8_t fm1, uint8_t fa1, uint8_t fd1, uint8_t rm1, uint8_t ra1,
  uint8_t rd1, uint8_t tm1, uint8_t ta1, uint8_t td1)
{
  int idx = 0;
  uint8_t packet[kHeaderSize + CommandSize::kSetSpeedProfileCommandSize] = {0};

  packet[idx++] = kProtocolSign;
  packet[idx++] = CommandSize::kSetSpeedProfileCommandSize;
  packet[idx++] = CommandId::kSetSpeedProfile;
  packet[idx++] = s1;
  packet[idx++] = fm1;
  packet[idx++] = fa1;
  packet[idx++] = fd1;
  packet[idx++] = rm1;
  packet[idx++] = ra1;
  packet[idx++] = rd1;
  packet[idx++] = tm1;
  packet[idx++] = ta1;
  packet[idx++] = td1;
  packet[idx] = parser_->Checksum(packet, idx);
  return port_->Send(packet, sizeof(packet));
}

int Whill::SendSetBatteryVoltageOutCommand(uint8_t battery_out)
{
  int idx = 0;
  uint8_t packet[kHeaderSize + CommandSize::kSetBatteryVoltageOutCommandSize] = {0};

  packet[idx++] = kProtocolSign;
  packet[idx++] = CommandSize::kSetBatteryVoltageOutCommandSize;
  packet[idx++] = CommandId::kSetBatteryVoltageOut;
  packet[idx++] = battery_out;
  packet[idx] = parser_->Checksum(packet, idx);
  return port_->Send(packet, sizeof(packet));
}

int Whill::SendSetBatterySavingCommand(uint8_t low_battery_level, uint8_t sounds_buzzer)
{
  int idx = 0;
  uint8_t packet[kHeaderSize + CommandSize::kSetBatterySavingCommandSize] = {0};

  packet[idx++] = kProtocolSign;
  packet[idx++] = CommandSize::kSetBatterySavingCommandSize;
  packet[idx++] = CommandId::kSetBatterySaving;
  packet[idx++] = low_battery_level;
  packet[idx++] = sounds_buzzer;
  packet[idx] = parser_->Checksum(packet, idx);
  return port_->Send(packet, sizeof(packet));
}

int Whill::SendSetVelocityCommand(int linear, int angular)
{
  int idx = 0;
  uint8_t packet[kHeaderSize + CommandSize::kSetVelocityCommandSize] = {0};

  packet[idx++] = kProtocolSign;
  packet[idx++] = CommandSize::kSetVelocityCommandSize;
  packet[idx++] = CommandId::kSetVelocity;
  packet[idx++] = kUserControlDisable;
  packet[idx++] = (uint8_t)((linear >> 8) & 0x000000FF);
  packet[idx++] = (uint8_t)(linear & 0x000000FF);
  packet[idx++] = (uint8_t)((angular >> 8) & 0x000000FF);
  packet[idx++] = (uint8_t)(angular & 0x000000FF);
  packet[idx] = parser_->Checksum(packet, idx);
  return port_->Send(packet, sizeof(packet));
}

int Whill::SendSetJoystickLockCommand(bool lock)
{
  int idx = 0;
  uint8_t packet[kHeaderSize + CommandSize::kSetJoystickLockCommandSize] = {0};

  packet[idx++] = kProtocolSign;
  packet[idx++] = CommandSize::kSetJoystickLockCommandSize;
  packet[idx++] = CommandId::kSetJoystickLock;
  packet[idx++] = lock ? 0x01 : 0x00;  // D0: 1:pause(lock) / 0:resume(unlock)
  packet[idx] = parser_->Checksum(packet, idx);
  return port_->Send(packet, sizeof(packet));
}

int Whill::ReceiveDataset0(std::shared_ptr<whill_msgs::msg::SpeedProfile> & msg)
{
  // Read data
  uint8_t buf[kDatasetMaxSize] = {0};
  int received_len = port_->Receive(buf, (int)kDatasetMaxSize);
  if (received_len < 1) {return 0;}

  // Parse data
  uint8_t payload[kDatasetMaxSize] = {0};
  int payload_len = parser_->Parse(buf, received_len, payload);
  if (payload_len < 1) {return 0;}
  if (payload_len >= 32) {return 0;}

  if (payload[0] == 0) {
    // Dataset0
    msg->s1 = payload[1];
    msg->fm1 = payload[2];
    msg->fa1 = payload[3];
    msg->fd1 = payload[4];
    msg->rm1 = payload[5];
    msg->ra1 = payload[6];
    msg->rd1 = payload[7];
    msg->tm1 = payload[8];
    msg->ta1 = payload[9];
    msg->td1 = payload[10];
  }
  return 1;
}

int Whill::ReceiveDataset1(std::shared_ptr<whill_msgs::msg::ModelCr2State> & msg)
{
  // Read data
  uint8_t buf[kDatasetMaxSize] = {0};
  int received_len = port_->Receive(buf, (int)kDatasetMaxSize);
  if (received_len < 1) {return 0;}

  // Parse data
  uint8_t payload[kDatasetMaxSize] = {0};
  int payload_len = parser_->Parse(buf, received_len, payload);
  if (payload_len < 1) {return 0;}
  if (payload_len >= 32) {return 0;}

  if (payload[0] == 1) {
    // Dataset1
    msg->joy_front = int8_t(payload[13]);
    msg->joy_side = int8_t(payload[14]);
    msg->battery_power = int(payload[15] & 0xff);
    msg->battery_current = Calc16BitSignedData(payload[16], payload[17]);
    msg->right_motor_angle =
      Calc16BitSignedData(payload[18], payload[19]) * kMotorAngleFactor;
    msg->left_motor_angle = Calc16BitSignedData(payload[20], payload[21]) * kMotorAngleFactor;
    msg->right_motor_speed =
      Calc16BitSignedData(payload[22], payload[23]) * kMotorSpeedFactor;
    msg->left_motor_speed = Calc16BitSignedData(payload[24], payload[25]) * kMotorSpeedFactor;
    msg->power_on = int(payload[26] & 0xff);
    msg->speed_mode_indicator = int(payload[27] & 0xff);
    msg->error = int(payload[28] & 0xff);
    if (msg->error != 0) {
      fprintf(stderr, "WHILL sends error message. error id: %d", msg->error);
    }
  }
  return 1;
}

/**
 * The function Calc16BitSignedData in C++ calculates a 16-bit signed data value from two 8-bit input
 * values.
 *
 * @param d1 The `Calc16BitSignedData` function takes two `uint8_t` parameters, `d1` and `d2`, which
 * represent the high and low bytes of a 16-bit signed data value respectively. The function combines
 * these two bytes to form a 16-bit signed integer and then
 * @param d2 The `d2` parameter is an 8-bit unsigned integer representing the least significant bits of
 * a 16-bit signed data value.
 *
 * @return The function `Calc16BitSignedData` is returning a floating-point value that represents a
 * 16-bit signed data value. The function takes two 8-bit unsigned integers `d1` and `d2` as input
 * parameters, combines them into a 16-bit value, and then checks if the value is greater than 2^15. If
 * it is, the value is adjusted to be
 */
float Whill::Calc16BitSignedData(uint8_t d1, uint8_t d2)
{
  float d = float(((d1 & 0xff) << 8) | (d2 & 0xff));
  if (d > pow(2, 15)) {
    d -= pow(2, 16);
  }
  return d;
}

}  // namespace model_cr2
}  // namespace whill_driver
