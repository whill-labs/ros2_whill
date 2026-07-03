// Copyright (c) 2024 WHILL, Inc.
// Released under the MIT license
// https://opensource.org/licenses/mit-license.php

/**
 * @file    whill.hpp
 * @brief   Definitions of WHILL Model CR2 domain
 */
#ifndef WHILL_DRIVER_MODEL_CR2_WHILL_H_
#define WHILL_DRIVER_MODEL_CR2_WHILL_H_

#include <memory>
#include <string.h>

#include "whill_msgs/msg/model_cr2_state.hpp"
#include "whill_msgs/msg/speed_profile.hpp"

#include "hardware/serial_port.hpp"
#include "parser.hpp"

namespace whill_driver
{
namespace model_cr2
{

/**
 * The number of WHILL State Dataset
 */
enum DatasetNumber : uint8_t
{
  kDatasetNumber0 = 0x00,  /// Speed profiles
  kDatasetNumber1 = 0x01,  /// Model CR2 status
};

/**
 * The mode of speed profiles of WHILL
 */
enum SpeedMode : uint8_t
{
  kSpeedMode0 = 0x00,
  kSpeedMode1,
  kSpeedMode2,
  kSpeedMode3,
  kSpeedMode4,  /// for serial port
  kSpeedMode5,  /// for app
  kSpeedModeMax
};

class Whill
{
public:
  Whill(const std::string & port);
  ~Whill();

  int SendStartSendingDataCommand(
    uint16_t interval_ms, DatasetNumber dataset_number,
    SpeedMode speed_mode);
  int SendStopSendingDataCommand();
  int SendSetPower(bool turn_on);
  int SetPowerOn();
  int SetPowerOff();
  int SendSetJoystickCommand(uint8_t fb, uint8_t lr);
  int SendSetJoystickCommandWithLocal();
  int SendSetSpeedProfileCommand(
    uint8_t s1,
    uint8_t fm1, uint8_t fa1, uint8_t fd1,
    uint8_t rm1, uint8_t ra1, uint8_t rd1,
    uint8_t tm1, uint8_t ta1, uint8_t td1);
  int SendSetBatteryVoltageOutCommand(uint8_t battery_out);
  int SendSetBatterySavingCommand(uint8_t low_battery_level, uint8_t sounds_buzzer);
  int SendSetVelocityCommand(int linear, int angular);
  int SendSetJoystickLockCommand(bool lock);
  int ReceiveDataset0(std::shared_ptr<whill_msgs::msg::SpeedProfile> & msg);
  int ReceiveDataset1(std::shared_ptr<whill_msgs::msg::ModelCr2State> & msg);

private:
  std::shared_ptr<hardware::SerialPort> port_;
  std::shared_ptr<Parser> parser_;

  float Calc16BitSignedData(uint8_t d1, uint8_t d2);
};

}  // namespace model_cr2
}  // namespace whill_driver

#endif  // WHILL_DRIVER_MODEL_CR2_WHILL_H_
