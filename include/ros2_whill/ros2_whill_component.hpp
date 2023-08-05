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

    char recv_buf_[128];

    Odometry odom_;

    // Helper

    float calc_16bit_signed_data(char d1, char d2)
    {
        float d = 0;
        d = float(((d1 & 0xff) << 8) | (d2 & 0xff));
        if (d > pow(2, 15))
        {
            d = d - pow(2, 16);
        }

        return d;
    }

    float calc_8bit_signed_data(char d1)
    {
        float d = 0;
        d = float(d1 & 0xff);
        if (d > pow(2, 7))
        {
            d = d - pow(2, 8);
        }

        return d;
    }

    double calc_rad_diff(double past, double current)
    {
        double diff = past - current;
        if (past * current < 0 && fabs(diff) > M_PI)
        {                                                      // Crossed +PI/-PI[rad] border
            diff = M_PI - fabs(past) + (M_PI - fabs(current)); // - to +
            if (past > 0 && current < 0)
            { // Case of + to -
                diff = -diff;
            }
        }
        return diff;
    }

    unsigned int calc_time_diff(unsigned int past, unsigned int current)
    {
        int diff = current - past;
        if (abs(diff) >= 100)
        {
            diff = (201 - past) + current;
        }
        return (unsigned int)diff;
    }

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
        size_t len, idx;
        initializeComWHILL(&whill_fd_, serial_port_);
        for (int i = 0; i < 6; ++i)
        {
            sendStopSendingData(whill_fd_);
            usleep(2000);
            sendStartSendingData(whill_fd_, 25, DATASET_NUM_ZERO, i);
            usleep(2000);
            len = recvDataWHILL(whill_fd_, recv_buf_);
            if (recv_buf_[0] == DATASET_NUM_ZERO && len == 12)
            {
                ros2_whill_interfaces::msg::WhillSpeedProfile speed_prof_msg;
                speed_prof_msg.s1 = int(recv_buf_[1] & 0xff);
                speed_prof_msg.fm1 = int(recv_buf_[2] & 0xff);
                speed_prof_msg.fa1 = int(recv_buf_[3] & 0xff);
                speed_prof_msg.fd1 = int(recv_buf_[4] & 0xff);
                speed_prof_msg.rm1 = int(recv_buf_[5] & 0xff);
                speed_prof_msg.ra1 = int(recv_buf_[6] & 0xff);
                speed_prof_msg.rd1 = int(recv_buf_[7] & 0xff);
                speed_prof_msg.tm1 = int(recv_buf_[8] & 0xff);
                speed_prof_msg.ta1 = int(recv_buf_[9] & 0xff);
                speed_prof_msg.td1 = int(recv_buf_[10] & 0xff);
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
                                                                        sendJoystick(this->whill_fd_, joy_front, joy_side);
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
                    sendSetSpeed(this->whill_fd_, req->s1, req->fm1, req->fa1, req->fd1, req->rm1, req->ra1, req->rd1, req->tm1, req->ta1, req->td1);
                    res->result = 1;
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
                    res->result = -1;
                    return false;
                } });

        set_power_srv_ = this->create_service<ros2_whill_interfaces::srv::SetPower>("/whill/set_power_srv", [this](const std::shared_ptr<rmw_request_id_t> req_header, const ros2_whill_interfaces::srv::SetPower::Request::SharedPtr req, const ros2_whill_interfaces::srv::SetPower::Response::SharedPtr res)
                                                                                    {
            (void)req_header;

            // power off
            if(req->p0 == 0){
                sendPowerOff(this->whill_fd_);
                RCLCPP_INFO(this->get_logger(), "WHILL wakes down\n");
                res->result = 1;
                return true;
            }else if(req->p0 == 1){
                char recv_buf[128];
                int len;

                //After firmware update of Model C, need to send 2times power on command.
                sendPowerOn(this->whill_fd_);
                usleep(10000);
                sendPowerOn(this->whill_fd_);
                usleep(2000);
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
                sendSetBatteryOut(this->whill_fd_, req->v0);
                if(req->v0 == 0) RCLCPP_INFO(this->get_logger(), "battery voltage out: disable");
                if(req->v0 == 1) RCLCPP_INFO(this->get_logger(), "battery voltage out: enable");
                res->result = 1;
                return true;
            }else{
                RCLCPP_INFO(this->get_logger(), "v0 must be assigned 0 or 1");
                res->result = -1;
                return false;
            } });

        // main timer config
        main_timer_ = this->create_wall_timer(100ms, [&]()
                                              {
                                                  ros2_whill_interfaces::msg::WhillModelC state_msg;
                                                  sensor_msgs::msg::Imu imu_msg;
                                                  sensor_msgs::msg::BatteryState battery_state_msg;
                                                  sensor_msgs::msg::Joy joy_msg;
                                                  sensor_msgs::msg::JointState joint_state_msg;
                                                  geometry_msgs::msg::TransformStamped odom_trans;
                                                  nav_msgs::msg::Odometry odom_msg;

                                                  // receive data from whill
                                                  unsigned int time_diff_ms = receive_whill_data(state_msg);
                                                  construct_imu_msg(imu_msg, state_msg);
                                                  construct_battery_state_msg(battery_state_msg, state_msg);
                                                  construct_joy_msg(joy_msg, state_msg);
                                                  construct_joint_state_msg(joint_state_msg, state_msg, time_diff_ms);
                                                  
                                                  odom_.update(joint_state_msg, time_diff_ms / 1000.0f);

                                                  // publish data
                                                  imu_pub_->publish(imu_msg);
                                                  battery_state_pub_->publish(battery_state_msg);
                                                  joy_pub_->publish(joy_msg);
                                                  joint_state_pub_->publish(joint_state_msg);

                                                  odom_trans.header.stamp = this->get_clock()->now();
                                                  odom_trans.header.frame_id = "odom";
                                                  odom_trans.child_frame_id = "base_footprint";
                                                  odom_broadcaster_.sendTransform(odom_trans);

                                                  odom_msg = odom_.getROSOdometry();
                                                  odom_msg.header.stamp = this->get_clock()->now();
                                                  odom_msg.header.frame_id = "odom";
                                                  odom_msg.child_frame_id = "base_footprint";
                                                  odom_pub_->publish(odom_msg); });
    }

    /**
     * @brief Receive data from whill, and parse it. This data is stored in state_msg.
     * 
     * @param state_msg [in] ros2_whill_interfaces::msg::WhillModelC
     * @return unsigned int elapsed time
     */
    unsigned int receive_whill_data(ros2_whill_interfaces::msg::WhillModelC &state_msg)
    {
        size_t len = recvDataWHILL(whill_fd_, recv_buf_);
        if (recv_buf_[0] == DATASET_NUM_ONE && (len == DATASET_LEN_OLD || len == DATASET_LEN_NEW))
        {
            unsigned char check_sum = 0x00;
            for (int i = 0; i <= len - 1; i++)
            {
                check_sum ^= static_cast<unsigned char>(recv_buf_[i]);
            }
            unsigned char cs = static_cast<unsigned char>(recv_buf_[len - 1]);

            rclcpp::Time cr_time_stamp = this->get_clock()->now();

            state_msg.acc_x = calc_16bit_signed_data(recv_buf_[1], recv_buf_[2]) * ACC_CONST;
            state_msg.acc_y = calc_16bit_signed_data(recv_buf_[3], recv_buf_[4]) * ACC_CONST;
            state_msg.acc_z = calc_16bit_signed_data(recv_buf_[5], recv_buf_[6]) * ACC_CONST;

            state_msg.gyr_x = calc_16bit_signed_data(recv_buf_[7], recv_buf_[8]) * GYR_CONST;
            state_msg.gyr_y = calc_16bit_signed_data(recv_buf_[9], recv_buf_[10]) * GYR_CONST;
            state_msg.gyr_z = calc_16bit_signed_data(recv_buf_[11], recv_buf_[12]) * GYR_CONST;

            state_msg.joy_front = (int)calc_8bit_signed_data(recv_buf_[13]);
            state_msg.joy_side = (int)calc_8bit_signed_data(recv_buf_[14]);

            state_msg.battery_power = int(recv_buf_[15] & 0xff);
            state_msg.battery_current = calc_16bit_signed_data(recv_buf_[16], recv_buf_[17]); // TODO -> fixed point

            state_msg.right_motor_angle = calc_16bit_signed_data(recv_buf_[18], recv_buf_[19]) * MANGLE_CONST;
            state_msg.left_motor_angle = calc_16bit_signed_data(recv_buf_[20], recv_buf_[21]) * MANGLE_CONST;
            state_msg.right_motor_speed = calc_16bit_signed_data(recv_buf_[22], recv_buf_[23]) * MSPEED_CONST;
            state_msg.left_motor_speed = calc_16bit_signed_data(recv_buf_[24], recv_buf_[25]) * MSPEED_CONST;

            state_msg.power_on = int(recv_buf_[26] && 0xff);
            state_msg.speed_mode_indicator = int(recv_buf_[27] && 0xff);

            unsigned int time_diff_ms = 0;
            if (len == DATASET_LEN_NEW)
            {
                unsigned int time_ms = (unsigned int)(recv_buf_[29] && 0xff);
                static unsigned int past_time_ms = 0;
                time_diff_ms = calc_time_diff(past_time_ms, time_ms);
                past_time_ms = time_ms;
            }

            state_msg.error = int(recv_buf_[28] & 0xff);
            if (state_msg.error != 0)
            {
                RCLCPP_WARN(this->get_logger(), "WHILL sends error message. error id: %d", state_msg.error);
            }
            return time_diff_ms;
        }
    }

    /**
     * @brief Construct ros imu message from state msg
     * 
     * @param imu_msg [in/out] sensor_msgs::msg::Imu
     * @param state_msg [in] ros2_whill_interfaces::msg::WhillModelC
     */
    void construct_imu_msg(sensor_msgs::msg::Imu &imu_msg, ros2_whill_interfaces::msg::WhillModelC &state_msg)
    {
        imu_msg.header.stamp = this->get_clock()->now();
        imu_msg.header.frame_id = "imu";
        imu_msg.orientation_covariance[0] = -1;
        imu_msg.angular_velocity.x = state_msg.gyr_x / 180 * M_PI; // deg per sec to rad/s
        imu_msg.angular_velocity.y = state_msg.gyr_y / 180 * M_PI; // deg per sec to rad/s
        imu_msg.angular_velocity.z = state_msg.gyr_z / 180 * M_PI; // deg per sec to rad/s
        imu_msg.linear_acceleration.x = state_msg.acc_x * 9.80665; // G to m/ss
        imu_msg.linear_acceleration.y = state_msg.acc_y * 9.80665; // G to m/ss
        imu_msg.linear_acceleration.z = state_msg.acc_z * 9.80665; // G to m/ss
    }

    /**
     * @brief Construct ros BatteryState message from state msg
     * 
     * @param battery_state_msg [in/out] sensor_msgs::msg::BatteryState
     * @param state_msg [in] ros2_whill_interfaces::msg::WhillModelC
     */
    void construct_battery_state_msg(sensor_msgs::msg::BatteryState &battery_state_msg, ros2_whill_interfaces::msg::WhillModelC &state_msg)
    {
        battery_state_msg.voltage = 25.2;                                 //[V]
        battery_state_msg.current = -state_msg.battery_current / 1000.0f; // mA -> A
        battery_state_msg.charge = std::numeric_limits<float>::quiet_NaN();
        battery_state_msg.design_capacity = 10.04;                       //[Ah]
        battery_state_msg.percentage = state_msg.battery_power / 100.0f; // Percentage
        battery_state_msg.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_UNKNOWN;
        battery_state_msg.power_supply_health = sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_UNKNOWN;
        battery_state_msg.power_supply_technology = sensor_msgs::msg::BatteryState::POWER_SUPPLY_TECHNOLOGY_LION;
        battery_state_msg.present = true;
        battery_state_msg.location = "0";
    }

    /**
     * @brief Construct ros joy message from state msg
     * 
     * @param joy_msg [in/out] sensor_msgs::msg::Joy
     * @param state_msg [in] ros2_whill_interfaces::msg::WhillModelC
     */
    void construct_joy_msg(sensor_msgs::msg::Joy &joy_msg, ros2_whill_interfaces::msg::WhillModelC &state_msg)
    {
        joy_msg.header.stamp = this->get_clock()->now();
        joy_msg.axes.resize(2);
        joy_msg.axes[0] = -state_msg.joy_side / 100.0f;
        joy_msg.axes[1] = state_msg.joy_front / 100.0f;
    }

    /**
     * @brief Construct ros JointState message from state msg
     * 
     * @param joint_state_msg [in/out] sensor_msgs::msg::JointState
     * @param state_msg [in] ros2_whill_interfaces::msg::WhillModelC
     * @param time_diff_ms [in] elapsed time
     */
    void construct_joint_state_msg(sensor_msgs::msg::JointState &joint_state_msg, ros2_whill_interfaces::msg::WhillModelC &state_msg, unsigned int time_diff_ms)
    {
        joint_state_msg.header.stamp = this->get_clock()->now();
        joint_state_msg.name.resize(2);
        joint_state_msg.position.resize(2);
        joint_state_msg.velocity.resize(2);

        joint_state_msg.name[0] = "leftWheel";
        joint_state_msg.position[0] = state_msg.left_motor_angle; // Rad

        static double past[2] = {0.0f, 0.0f};

        if (time_diff_ms != 0)
            joint_state_msg.velocity[0] = calc_rad_diff(past[0], joint_state_msg.position[0]) / (double(time_diff_ms) / 1000.0f);
        else
            joint_state_msg.velocity[0] = 0;

        past[0] = joint_state_msg.position[0];

        joint_state_msg.name[1] = "rightWheel";
        joint_state_msg.position[1] = state_msg.right_motor_angle; // Rad

        if (time_diff_ms != 0)
            joint_state_msg.velocity[1] = calc_rad_diff(past[1], joint_state_msg.position[1]) / (double(time_diff_ms) / 1000.0f);
        else
            joint_state_msg.velocity[0] = 0;
        past[1] = joint_state_msg.position[1];
    }

    /**
     * @brief Shutdown WHILL
     * 
     * @todo does it work correctly???
     * @todo implement stopping whill process
     */
    void ShutdownWHILL(void)
    {
        RCLCPP_INFO(this->get_logger(), "Request Stop Sending Data.");
        sendStopSendingData(whill_fd_);
        usleep(1000);
        RCLCPP_INFO(this->get_logger(), "Closing Serial Port.");
        closeComWHILL(whill_fd_);
    }
};