/* 

MIT License

Copyright (c) 2018 WHILL inc.

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

/*
Note that this file instantiates a rclcpp::Node without subclassing it. 
This was the typical usage model in ROS 1, but this style of coding is 
not compatible with composing multiple nodes into a single process. 
Thus, it is no longer the recommended style for ROS 2.
*/

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"

#include "whill_modelc/com_whill.h"
#include "ros2_whill_interfaces/srv/set_speed_profile.hpp"
#include "ros2_whill_interfaces/srv/set_power.hpp"
#include "ros2_whill_interfaces/srv/set_battery_voltage_out.hpp"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <fcntl.h>
#include <sys/epoll.h>
#include <unistd.h>
#include <time.h>
#include <sys/time.h>
#include <math.h>

#define POWERON_RESPONSE_DATA_IDX (0)
#define POWERON_RESPONSE_DATA (0x52)

int whill_fd; // file descriptor for UART to communicate with WHILL
rclcpp::Node::SharedPtr node = nullptr;

// Set Speed Profile
bool set_speed_profile_srv(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<ros2_whill_interfaces::srv::SetSpeedProfile::Request> request, 
    const std::shared_ptr<ros2_whill_interfaces::srv::SetSpeedProfile::Response> response)
{
    (void)request_header;

    // value check
    if(0 <= request->s1 && request->s1 <= 5
        && 8 <= request->fm1 && request->fm1 <= 80
        && 10 <= request->fa1 && request->fa1 <= 90
        && 40 <= request->fd1 && request->fd1 <= 160
        && 8 <= request->rm1 && request->rm1 <= 30
        && 10 <= request->ra1 && request->ra1 <= 50
        && 40 <= request->rd1 && request->rd1 <= 80
        && 8 <= request->tm1 && request->tm1 <= 35
        && 10 <= request->ta1 && request->ta1 <= 100
        && 40 <= request->td1 && request->td1 <= 160)
    {
        RCLCPP_INFO(node->get_logger(), "Speed profile is set");
        sendSetSpeed(whill_fd, request->s1, request->fm1, request->fa1, request->fd1, request->rm1, request->ra1, request->rd1, request->tm1, request->ta1, request->td1);
        
        response->result = 1;
        return true;
    }
    else
    {
        RCLCPP_WARN(node->get_logger(), "wrong parameter is assigned.");
        RCLCPP_WARN(node->get_logger(), "s1 must be assingned between 0 - 5");
        RCLCPP_WARN(node->get_logger(), "fm1 must be assingned between 8 - 60");
        RCLCPP_WARN(node->get_logger(), "fa1 must be assingned between 10 - 90");
        RCLCPP_WARN(node->get_logger(), "fd1 must be assingned between 10 - 160");
        RCLCPP_WARN(node->get_logger(), "rm1 must be assingned between 8 - 60");
        RCLCPP_WARN(node->get_logger(), "ra1 must be assingned between 10 - 90");
        RCLCPP_WARN(node->get_logger(), "rd1 must be assingned between 10 - 160");
        RCLCPP_WARN(node->get_logger(), "tm1 must be assingned between 8 - 60");
        RCLCPP_WARN(node->get_logger(), "ta1 must be assingned between 10 - 90");
        RCLCPP_WARN(node->get_logger(), "td1 must be assingned between 10 - 160");
        response->result = -1;
        return false;
    }    
}


// Set Power
bool set_power_srv(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<ros2_whill_interfaces::srv::SetPower::Request> request,
    const std::shared_ptr<ros2_whill_interfaces::srv::SetPower::Response> response)
{
    (void)request_header;
    // power off
    if(request->p0 == 0)
    {
        sendPowerOff(whill_fd);
        RCLCPP_INFO(node->get_logger(), "WHILL wakes down\n");
        response->result = 1;
        return true;
    }
    // power on
    else if (request->p0 == 1)
    {
        char recv_buf[128];
        int len;

        sendPowerOn(whill_fd);
        usleep(2000);
        
        RCLCPP_INFO(node->get_logger(), "WHILL wakes up");
        response->result = 1;
        return true;
    }
    else
    {
        RCLCPP_WARN(node->get_logger(), "p0 must be assinged 0 or 1");
        response->result = -1;
        return false;
    }

}


// Set Battery Voltage out
bool set_battery_voltage_out_srv(
    const std::shared_ptr<rmw_request_id_t> request_header,    
    const std::shared_ptr<ros2_whill_interfaces::srv::SetBatteryVoltageOut::Request> request,
    const std::shared_ptr<ros2_whill_interfaces::srv::SetBatteryVoltageOut::Response> response)
{
    (void)request_header;
    if(request->v0 == 0 || request->v0 == 1)
    {
        sendSetBatteryOut(whill_fd, request->v0);
        if(request->v0 == 0) RCLCPP_INFO(node->get_logger(), "battery voltage out: disable");
        if(request->v0 == 1) RCLCPP_INFO(node->get_logger(), "battery voltage out: enable");
        response->result = 1;
        return true;
    }
    else
    {
        RCLCPP_INFO(node->get_logger(), "v0 must be assigned 0 or 1");
        response->result = -1;
        return false;
    }

}


// Set Joystick

void whillSetJoyMsgCallback(const sensor_msgs::msg::Joy::SharedPtr joy)
{
    int joy_side  = -joy->axes[0] * 100.0f;
    int joy_front = joy->axes[1] * 100.0f;

    // value check
    if(joy_front < -100) joy_front = -100;
    if(joy_front > 100)  joy_front = 100;
    if(joy_side < -100) joy_side = -100;
    if(joy_side > 100)  joy_side = 100;

    sendJoystick(whill_fd, joy_front, joy_side);
}


int main(int argc, char **argv)
{
    // ROS setup
    rclcpp::init(argc, argv);
    node = rclcpp::Node::make_shared("whill_modelc_controller");
    
    std::string serialport = "/dev/ttyUSB0";
    node->get_parameter("serialport", serialport);
    RCLCPP_INFO(node->get_logger(), "=========================");
    RCLCPP_INFO(node->get_logger(), "WHILL CR Controller:");
    RCLCPP_INFO(node->get_logger(), "    serialport: %s", serialport.c_str());
    RCLCPP_INFO(node->get_logger(), "=========================");

    // Services
    auto set_speed_profile = node->create_service<ros2_whill_interfaces::srv::SetSpeedProfile>("/whill/set_speed_profile_srv", set_speed_profile_srv);
    auto set_power = node->create_service<ros2_whill_interfaces::srv::SetPower>("/whill/set_power_srv", set_power_srv);
    auto set_battery_voltage_out = node->create_service<ros2_whill_interfaces::srv::SetBatteryVoltageOut>("/whill/set_battery_voltage_out_srv", set_battery_voltage_out_srv);

    // Subscribers
    auto whill_setjoy_sub = node->create_subscription<sensor_msgs::msg::Joy>("/whill/controller/joy", whillSetJoyMsgCallback, rmw_qos_profile_sensor_data);

    initializeComWHILL(&whill_fd, serialport);
    rclcpp::spin(node);

    closeComWHILL(whill_fd);
    rclcpp::shutdown();
    node = nullptr;

    return 0;
}


