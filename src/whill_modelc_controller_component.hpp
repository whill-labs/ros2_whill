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

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/joy.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <std_srvs/srv/empty.hpp>

#include "ros2_whill_interfaces/msg/whill_model_c.hpp"
#include "ros2_whill_interfaces/msg/whill_speed_profile.hpp"
#include "ros2_whill_interfaces/srv/set_speed_profile.hpp"
#include "ros2_whill_interfaces/srv/set_power.hpp"
#include "ros2_whill_interfaces/srv/set_battery_voltage_out.hpp"
#include "whill_modelc/com_whill.h"

#include "./odom.h"

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
#include <math.h>
#include <limits>
#include <chrono>

#define MAX_EVENTS (10)
#define DATASET_NUM_ZERO (0)
#define DATASET_NUM_ONE (1)
#define SPEED_MODE (0)
#define SEND_INTERVAL (10)

#define DATASET_LEN_OLD (30)
#define DATASET_LEN_NEW (31)

#define ACC_CONST (0.000122)
#define GYR_CONST (0.004375)
#define MANGLE_CONST (0.001)
#define MSPEED_CONST (0.004)

class WhillController : public rclcpp::Node
{
private:
    rclcpp::Publisher<ros2_whill_interfaces::msg::WhillModelC>::SharedPtr state_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr joy_pub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr battery_state_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<ros2_whill_interfaces::msg::WhillSpeedProfile>::SharedPtr speed_prof_pub_;

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;

    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr clear_odom_srv_;
    rclcpp::Service<ros2_whill_interfaces::srv::SetSpeedProfile>::SharedPtr set_speed_prof_srv_;
    rclcpp::Service<ros2_whill_interfaces::srv::SetPower>::SharedPtr set_power_srv_;
    rclcpp::Service<ros2_whill_interfaces::srv::SetBatteryVoltageOut>::SharedPtr set_battery_voltage_out_srv_;

    tf2_ros::TransformBroadcaster odom_broadcaster_;

    rclcpp::TimerBase::SharedPtr main_timer_;

    double wheel_radius_;
    std::string serial_port_;
    int send_interval_;
    int whill_fd_;

    Odometry odom_;

public:
    WhillController(const rclcpp::NodeOptions &options) : WhillController("", options) {}
    WhillController(const std::string &name_space = "", const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
        : Node("whill_modelc_controller_node", name_space, options), odom_broadcaster_(this)
    {
        using namespace std::chrono_literals;

        // parameter setting
        declare_parameter("wheel_radius", 0.135);
        wheel_radius_ = get_parameter("wheel_radius").as_double();
        declare_parameter("serial_port", "/dev/ttyUSB0");
        serial_port_ = get_parameter("serial_port").as_string();
        declare_parameter("send_interval", 10);
        send_interval_ = get_parameter("send_interval").as_int();

        RCLCPP_INFO(this->get_logger(), "=========================");
        RCLCPP_INFO(this->get_logger(), "WHILL CR Publisher:");
        RCLCPP_INFO(this->get_logger(), "    serialport: %s", serial_port_.c_str());
        RCLCPP_INFO(this->get_logger(), "    wheel_radius: %f", wheel_radius_);
        RCLCPP_INFO(this->get_logger(), "    send_interval: %d", send_interval_);
        RCLCPP_INFO(this->get_logger(), "=========================");

        // open uart communication
        char recv_buf[128];
        size_t len, idx;
        initializeComWHILL(&whill_fd_, serial_port_);
        for(int i = 0; i < 6; ++i){
            sendStopSendingData(whill_fd_);
            usleep(2000);
            sendStartSendingData(whill_fd_, 25, DATASET_NUM_ZERO, i);
            usleep(2000);
            len = recvDataWHILL(whill_fd_, recv_buf);
            if(recv_buf[0] == DATASET_NUM_ZERO && len == 12){
                ros2_whill_interfaces::msg::WhillSpeedProfile speed_prof_msg;
                speed_prof_msg.s1  = int(recv_buf[1] & 0xff);
			    speed_prof_msg.fm1 = int(recv_buf[2] & 0xff);
			    speed_prof_msg.fa1 = int(recv_buf[3] & 0xff);
			    speed_prof_msg.fd1 = int(recv_buf[4] & 0xff);
			    speed_prof_msg.rm1 = int(recv_buf[5] & 0xff);
			    speed_prof_msg.ra1 = int(recv_buf[6] & 0xff);
			    speed_prof_msg.rd1 = int(recv_buf[7] & 0xff);
			    speed_prof_msg.tm1 = int(recv_buf[8] & 0xff);
			    speed_prof_msg.ta1 = int(recv_buf[9] & 0xff);
			    speed_prof_msg.td1 = int(recv_buf[10] & 0xff);
                speed_prof_pub_->publish(speed_prof_msg);
			    RCLCPP_INFO(this->get_logger(), "Speed profile %ld is published", speed_prof_msg.s1);
            }
        }

        // send StopSendingData command
	    sendStopSendingData(whill_fd_);
	    usleep(2000);
	    // Send StartSendingData command: dataset 1
	    sendStartSendingData(whill_fd_, send_interval_, DATASET_NUM_ONE, SPEED_MODE);
	    usleep(2000);

        // pub/sub/srv initialize
        state_pub_ = this->create_publisher<ros2_whill_interfaces::msg::WhillModelC>("/whill/modelc_state", rclcpp::QoS(10));
        joy_pub_ = this->create_publisher<sensor_msgs::msg::Joy>("/whill/states/joy", rclcpp::QoS(10));
        joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/whill/states/joint_state", rclcpp::QoS(10));
        imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("/whill/states/imu", rclcpp::QoS(10));
        battery_state_pub_ = this->create_publisher<sensor_msgs::msg::BatteryState>("/whill/states/battery_state", rclcpp::QoS(10));
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/whill/odom", rclcpp::QoS(10));

        speed_prof_pub_ = this->create_publisher<ros2_whill_interfaces::msg::WhillSpeedProfile>("/whill/speed_profile", rclcpp::QoS(10));

        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>("/whill/controller/joy", rclcpp::QoS(10), [this](const sensor_msgs::msg::Joy::SharedPtr joy)
                                                                    {
                                                                        int joy_side = -joy->axes[0] * 100.0f;
                                                                        int joy_front = joy->axes[1] * 100.0f;

                                                                        // value check
                                                                        if (joy_front < -100)
                                                                            joy_front = -100;
                                                                        if (joy_front > 100)
                                                                            joy_front = 100;
                                                                        if (joy_side < -100)
                                                                            joy_side = -100;
                                                                        if (joy_side > 100)
                                                                            joy_side = 100;
                                                                        // sendJoystick(this->whill_fd_, joy_front, joy_side);
                                                                    });

        clear_odom_srv_ = this->create_service<std_srvs::srv::Empty>("/whill/odom/clear", [this](const std::shared_ptr<rmw_request_id_t> req_header, const std::shared_ptr<std_srvs::srv::Empty::Request> req, const std::shared_ptr<std_srvs::srv::Empty::Response> res)
                                                                     {
            RCLCPP_INFO(this->get_logger(), "Clear Odometry");
	        this->odom_.reset();
            return true; });

        set_speed_prof_srv_ = this->create_service<ros2_whill_interfaces::srv::SetSpeedProfile>("/whill/set_speed_profile_srv", [this](const std::shared_ptr<rmw_request_id_t> req_header, const ros2_whill_interfaces::srv::SetSpeedProfile::Request::SharedPtr req, const ros2_whill_interfaces::srv::SetSpeedProfile::Response::SharedPtr res)
                                                                                                {
            (void)req_header;
                // value check
                if(0 <= req->s1 && req->s1 <= 5
                    && 8 <= req->fm1 && req->fm1 <= 80
                    && 10 <= req->fa1 && req->fa1 <= 90
                    && 40 <= req->fd1 && req->fd1 <= 160
                    && 8 <= req->rm1 && req->rm1 <= 30
                    && 10 <= req->ra1 && req->ra1 <= 50
                    && 40 <= req->rd1 && req->rd1 <= 80
                    && 8 <= req->tm1 && req->tm1 <= 35
                    && 10 <= req->ta1 && req->ta1 <= 100
                    && 40 <= req->td1 && req->td1 <= 160){
                    RCLCPP_INFO(this->get_logger(), "Speed profile is set");
                    // sendSetSpeed(this->whill_fd_, req->s1, req->fm1, req->fa1, req->fd1, req->rm1, req->ra1, req->rd1, req->tm1, req->ta1, req->td1);
                    // releaseJoystick->result = 1;
                    return true;
                }else{
                    RCLCPP_WARN(this->get_logger(), "wrong parameter is assigned.");
                    RCLCPP_WARN(this->get_logger(), "s1 must be assingned between 0 - 5");
                    RCLCPP_WARN(this->get_logger(), "fm1 must be assingned between 8 - 60");
                    RCLCPP_WARN(this->get_logger(), "fa1 must be assingned between 10 - 90");
                    RCLCPP_WARN(this->get_logger(), "fd1 must be assingned between 10 - 160");
                    RCLCPP_WARN(this->get_logger(), "rm1 must be assingned between 8 - 60");
                    RCLCPP_WARN(this->get_logger(), "ra1 must be assingned between 10 - 90");
                    RCLCPP_WARN(this->get_logger(), "rd1 must be assingned between 10 - 160");
                    RCLCPP_WARN(this->get_logger(), "tm1 must be assingned between 8 - 60");
                    RCLCPP_WARN(this->get_logger(), "ta1 must be assingned between 10 - 90");
                    RCLCPP_WARN(this->get_logger(), "td1 must be assingned between 10 - 160");
                    // releaseJoystick->result = -1;
                    return false;
                } });

        set_power_srv_ = this->create_service<ros2_whill_interfaces::srv::SetPower>("/whill/set_power_srv", [this](const std::shared_ptr<rmw_request_id_t> req_header, const ros2_whill_interfaces::srv::SetPower::Request::SharedPtr req, const ros2_whill_interfaces::srv::SetPower::Response::SharedPtr res)
                                                                                    {
            (void)req_header;

            // power off
            if(req->p0 == 0){
                // sendPowerOff(this->whill_fd_);
                RCLCPP_INFO(this->get_logger(), "WHILL wakes down\n");
                res->result = 1;
                return true;
            }else if(req->p0 == 1){
                char recv_buf[128];
                int len;

                //After firmware update of Model C, need to send 2times power on command.
                //sendPowerOn(this->whill_fd_);
                //usleep(10000);
                //sendPowerOn(this->whill_fd_);
                //usleep(2000);
                RCLCPP_INFO(this->get_logger(), "WHILL wakes up");
                res->result = 1;
                return true;
            }else{
                RCLCPP_WARN(this->get_logger(), "p0 must be assinged 0 or 1");
                res->result = -1;
                return false;
            } });

        set_battery_voltage_out_srv_ = this->create_service<ros2_whill_interfaces::srv::SetBatteryVoltageOut>("/whill/set_battery_voltage_out_srv", [this](const std::shared_ptr<rmw_request_id_t> req_header, const ros2_whill_interfaces::srv::SetBatteryVoltageOut::Request::SharedPtr req, const ros2_whill_interfaces::srv::SetBatteryVoltageOut::Response::SharedPtr res)
                                                                                                              {
            (void)req_header;
            if(req->v0 == 0 || req->v0 == 1){
                // sendSetBatteryOut(this->whill_fd_, request->v0);
                if(req->v0 == 0) RCLCPP_INFO(this->get_logger(), "battery voltage out: disable");
                if(req->v0 == 1) RCLCPP_INFO(this->get_logger(), "battery voltage out: enable");
                res->result = 1;
                return true;
            }else{
                RCLCPP_INFO(this->get_logger(), "v0 must be assigned 0 or 1");
                res->result = -1;
                return false;
            } });

        main_timer_ = this->create_wall_timer(100ms, [&](){
            
        });
    }
};