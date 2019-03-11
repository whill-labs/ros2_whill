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
#include "sensor_msgs/msg/joint_state.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.hpp"
#include "std_srvs/srv/empty.hpp"

#include "ros2_whill_interfaces/msg/whill_model_c.hpp"
#include "ros2_whill_interfaces/msg/whill_speed_profile.hpp"
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

#define MAX_EVENTS (10)
#define DATASET_NUM_ZERO (0)
#define DATASET_NUM_ONE (1)
#define SPEED_MODE (0)
#define SEND_INTERVAL (10)

#define ACC_CONST (0.000122)
#define GYR_CONST (0.004375)
#define MANGLE_CONST (0.001)
#define MSPEED_CONST (0.004)

Odometry odom;

int registerFdToEpoll(struct epoll_event *ev, int epollfd, int fd)
{
	/* register socket to epoll */
	ev->events = EPOLLIN;
	ev->data.fd = fd;
	if (epoll_ctl(epollfd, EPOLL_CTL_ADD, fd, ev) == -1) {
		RCLCPP_INFO(node->get_logger(), "fail epoll_ctl\n");
		return -1;
	}
	return 1;
}

float calc_16bit_signed_data(char d1, char d2)
{
	float d = 0;
	d = float(((d1 & 0xff) << 8) | (d2 & 0xff));
	if(d > pow(2, 15)){
		d = d - pow(2, 16);
	}

	return d;
}

float calc_8bit_signed_data(char d1)
{
	float d = 0;
	d = float(d1 & 0xff);
	if(d > pow(2, 7)){
		d = d - pow(2, 8);
	}

	return d;
}

double calc_rad_diff(double past,double current){
	double diff = past - current;
	if(past * current < 0 && fabs(diff) > M_PI){  // Crossed +PI/-PI[rad] border
        diff = M_PI-fabs(past) + (M_PI-fabs(current));  // - to +
        if(past > 0 && current < 0){  // Case of + to -
            diff = -diff;
        }
    }
	return diff;
}

unsigned int calc_time_diff(unsigned int past,unsigned int current){
	int diff = current - past;
	if(abs(diff) >= 100){
		diff = (201 - past) + current;
	}
	return (unsigned int)diff;
}

bool clearOdom(
	const std::shared_ptr<rmq_request_id_t> request_header,
	const std::shared_ptr<std_srvs::srv::Empty::Request> request,
	const std::shared_ptr<std_srvs::srv::Empty::Response> response){

	RCLCPP_INFO(node->get_logger(), "Clear Odometry");
	odom.reset();
	return true;
}


int main(int argc, char **argv)
{
	// ROS setup
	ros::init(argc, argv);
	node = rclcpp::Node::make_shared("whill_modelc_publisher");

	auto whill_modelc_pub         = node->create_publisher<ros2_whill_interfaces::msg::WhillModelC>("/whill/modelc_state");
	auto whill_modelc_joy         = node->create_publisher<sensor_msgs::msg::Joy>("/whill/states/joy");
	auto whill_modelc_joint_state = node->create_publisher<sensor_msgs::msg::JointState>("/whill/states/joint_state");
	auto whill_modelc_imu         = node->create_publisher<sensor_msgs::msg::Imu>("/whill/states/imu");
	auto whill_modelc_battery     = node->create_publisher<sensor_msgs_msg::Battery>("/whill/states/batttery_state");
	auto whill_modelc_odom        = node->create_publisher<nav_msgs::msg::Odometry>("/whill/odom");

	auto clear_odom_srv           = node->create_service<std_srvs::srv::Empty>("/whill/odom/clear", clearOdom);
	
	auto whill_speed_profile      = node->create_publisher<ros2_whill_interfaces::msg::WhillSpeedProfile>("/whill/speed_profile");

	tf2_ros::TransformBroadcaster odom_broadcaster_(node);

	double wheel_radius = 0.135;
	node->get_parameter("wheel_radius", wheel_radius);
	std::string serialport = "/dev/ttyUSB0";
	node->get_parameter("serialport", serialport);
	int send_interval = SEND_INTERVAL;
	node->get_parameter("send_interval", send_interval);

	// Node Param
	if(send_interval < 10){
	RCLCPP_WARN(node->get_logger(), "Too short interval. Set interval > 10");
		//nh.setParam("/whill_modelc_publisher/send_interval", 10);
		send_interval = 10;
	}
	RCLCPP_INFO(node->get_logger(), "param: send_interval=%d", send_interval);

	// WHILL setup
	struct epoll_event ev;
	struct epoll_event events[MAX_EVENTS];
	int epollfd, nfds;
	int whill_fd; // file descriptor for UART to communicate with WHILL
	char recv_buf[128];
	int len, idx;
	int i;

	initializeComWHILL(&whill_fd,serialport);
	if((epollfd = epoll_create1(0)) < 0)
	{
		RCLCPP_INFO(node->get_logger(), "can't creare epoll\n");
		return -1;
	}

	// register Fd to epoll
	registerFdToEpoll(&ev, epollfd, whill_fd);

	// Send StartSendingData command: dataset 0 for all speed mode
	for(int i = 0; i < 6; i ++)
	{
		sendStopSendingData(whill_fd);
		usleep(2000);
		sendStartSendingData(whill_fd, 25, DATASET_NUM_ZERO, i);
		usleep(2000);
		len = recvDataWHILL(whill_fd, recv_buf);
		if(recv_buf[0] == DATASET_NUM_ZERO && len == 12)
		{
			ros_whill::msgWhillSpeedProfile msg_sp;
			msg_sp.s1  = int(recv_buf[1] & 0xff);
			msg_sp.fm1 = int(recv_buf[2] & 0xff);
			msg_sp.fa1 = int(recv_buf[3] & 0xff);
			msg_sp.fd1 = int(recv_buf[4] & 0xff);
			msg_sp.rm1 = int(recv_buf[5] & 0xff);
			msg_sp.ra1 = int(recv_buf[6] & 0xff);
			msg_sp.rd1 = int(recv_buf[7] & 0xff);
			msg_sp.tm1 = int(recv_buf[8] & 0xff);
			msg_sp.ta1 = int(recv_buf[9] & 0xff);
			msg_sp.td1 = int(recv_buf[10] & 0xff);
			whill_speed_profile_pub.publish(msg_sp);
			RCLCPP_INFO(node->get_logger(), "Speed profile %ld is published", msg_sp.s1);
		}
	}


	// send StopSendingData command
	sendStopSendingData(whill_fd);
	usleep(2000);
	// Send StartSendingData command: dataset 1
	sendStartSendingData(whill_fd, send_interval, DATASET_NUM_ONE, SPEED_MODE);
	usleep(2000);

	// loop
	int msg_cnt = 0;
	RCLCPP_INFO(node->get_logger(), "Start WHILL message reporting");
	while(rclcpp::ok())
 	{
		//RCLCPP_INFO(node->get_logger(), "WHILL message %d is published", msg_cnt ++);
		auto msg = std::make_shared<ros2_whill_interfaces::msg::WhillModelC>();
		auto joy = std::make_shared<sensor_msgs::msg::Joy>();
		auto jointState = std::make_shared<sensor_msgs::msg::JointState>();
		auto Imu = std::make_shared<sensor_msgs::msg::Imu>();
		auto batteryState = std::make_shared<sensor_msgs::msg::BatteryState>();
		auto odomMsg = std::make_shared<nav_msgs::msg::Odometry>();

		// WHILL data
		// Process epoll
		nfds = epoll_wait(epollfd, events, MAX_EVENTS, -1);
		if (nfds == -1) {
			RCLCPP_INFO(node->get_logger(), "Error : epoll wait");
			break;
		}
		
		for (i = 0; i < nfds; i++) {
			// Receive Data From WHILL
			if(events[i].data.fd == whill_fd) {
				len = recvDataWHILL(whill_fd, recv_buf);
				if(recv_buf[0] == DATASET_NUM_ONE && len == 31)
				{
					unsigned char checksum = 0x00;
					for(int i = 0; i<=29; i++){
						checksum ^= static_cast<unsigned char>(recv_buf[i]);
					}
					unsigned char cs = static_cast<unsigned char>(recv_buf[30]);

					//printf("%d %d",recv_buf[30],cs);
					//if(checksum != cs){
					//	RCLCPP_WARN(node->get_logger(), "Checksum Failed 0x%02x:0x%02x",checksum,cs-checksum);
					//	continue;
					//}


					rclcpp::Time currentTime = rclcpp::Time::now();
					joy->header.stamp = currentTime;
					jointState->header.stamp = currentTime;
					imu->header.stamp = currentTime;

					msg->acc_x = calc_16bit_signed_data(recv_buf[1], recv_buf[2]) * ACC_CONST;
					msg->acc_y = calc_16bit_signed_data(recv_buf[3], recv_buf[4]) * ACC_CONST;
					msg->acc_z = calc_16bit_signed_data(recv_buf[5], recv_buf[6]) * ACC_CONST;

					msg->gyr_x = calc_16bit_signed_data(recv_buf[7], recv_buf[8]) * GYR_CONST;
					msg->gyr_y = calc_16bit_signed_data(recv_buf[9], recv_buf[10]) * GYR_CONST;
					msg->gyr_z = calc_16bit_signed_data(recv_buf[11], recv_buf[12]) * GYR_CONST;

					msg->joy_front = (int)calc_8bit_signed_data(recv_buf[13]);
					msg->joy_side = (int)calc_8bit_signed_data(recv_buf[14]);

					msg->battery_power = int(recv_buf[15] & 0xff);
					msg->battery_current = calc_16bit_signed_data(recv_buf[16], recv_buf[17]); //TODO -> fixed point

					msg->right_motor_angle = calc_16bit_signed_data(recv_buf[18], recv_buf[19]) * MANGLE_CONST;
					msg->left_motor_angle = calc_16bit_signed_data(recv_buf[20], recv_buf[21]) * MANGLE_CONST;
					msg->right_motor_speed = calc_16bit_signed_data(recv_buf[22], recv_buf[23]) * MSPEED_CONST;
					msg->left_motor_speed = calc_16bit_signed_data(recv_buf[24], recv_buf[25]) * MSPEED_CONST;

					msg->power_on = int(recv_buf[26] & 0xff);

					msg->speed_mode_indicator = int(recv_buf[27] & 0xff);

					
					unsigned int time_ms = (unsigned int)(recv_buf[29] & 0xff);
					static unsigned int past_time_ms = 0;
					unsigned int time_diff_ms = calc_time_diff(past_time_ms,time_ms);
					past_time_ms = time_ms;


					// IMU message
					imu->header.frame_id = "imu";

					imu->orientation_covariance[0] = -1;   // Orientation is unknown

					imu->angular_velocity.x = msg->gyr_x / 180 * M_PI;   // deg per sec to rad/s
					imu->angular_velocity.y = msg->gyr_y / 180 * M_PI;   // deg per sec to rad/s
					imu->angular_velocity.z = msg->gyr_z / 180 * M_PI;   // deg per sec to rad/s

					imu->linear_acceleration.x = msg->acc_x * 9.80665;  // G to m/ss
					imu->linear_acceleration.y = msg->acc_y * 9.80665;  // G to m/ss
					imu->linear_acceleration.z = msg->acc_z * 9.80665;  // G to m/ss

					// Battery State

					batteryState->voltage                 = 25.2; //[V]
					batteryState->current                 = -msg->battery_current / 1000.0f; // mA -> A
					batteryState->charge                  = std::numeric_limits<float>::quiet_NaN();
					batteryState->design_capacity         = 10.04;//[Ah]
					batteryState->percentage              = msg->battery_power / 100.0f; // Percentage
					batteryState->power_supply_status     = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_UNKNOWN;
					batteryState->power_supply_health     = sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_UNKNOWN;
					batteryState->power_supply_technology = sensor_msgs::msg::BatteryState::POWER_SUPPLY_TECHNOLOGY_LION;
					batteryState->present                 = true;
					batteryState->location                = "0";


					// Joy message
					
					joy->axes.resize(2);
					joy->axes[0] = -msg->joy_side / 100.0f; //X
					joy->axes[1] = msg->joy_front  / 100.0f; //Y


					// JointState message
					jointState->name.resize(2);
					jointState->position.resize(2);
					jointState->velocity.resize(2);

					jointState->name[0]     = "leftWheel";
					jointState->position[0] = msg->left_motor_angle;  //Rad

					static double past[2] = {0.0f,0.0f};
					
					if(time_diff_ms != 0)jointState->velocity[0] = calc_rad_diff(past[0],jointState->position[0]) / (double(time_diff_ms) / 1000.0f);
					else jointState->velocity[0] = 0;

					past[0] = jointState->position[0];

					jointState->name[1]     = "rightWheel";
					jointState->position[1] = msg->right_motor_angle;  //Rad

					if(time_diff_ms != 0)jointState->velocity[1] = calc_rad_diff(past[1],jointState->position[1]) / (double(time_diff_ms) / 1000.0f);
					else jointState->velocity[0] = 0;
					past[1] = jointState->position[1];


					odom.update(*jointState, time_diff_ms/1000.0f);
					

					msg->error = int(recv_buf[28] & 0xff);
					if(msg->error != 0)
					{
						RCLCPP_WARN(node->get_logger(), "WHILL sends error message. error id: %d", msg->error);
					}

					// publish
					whill_modelc_joy->publish(joy);
					whill_modelc_jointState->publish(jointState);
					whill_modelc_imu->publish(imu);
					whill_modelc_battery->publish(batteryState);

					// Publish Odometry
					auto odom_trans = std::make_shared<geometry_msgs::msg::TransformStamped>();
					*odom_trans = odom.getROSTransformStamped();
					odom_trans->header.stamp = currentTime;
					odom_trans->header.frame_id = "odom";
					odom_trans->child_frame_id = "base_footprint";
					odom_broadcaster_->sendTransform(odom_trans);

					auto odom_msg = std::make_shared<nav_msgs::msg::Odometry>();
					*odom_msg = odom.getROSOdometry();
					odom_msg->header.stamp = currentTime;
					odom_msg->header.frame_id = "odom";
					odom_msg->child_frame_id = "base_footprint";
					whill_modelc_odom->publish(odom_msg);
				}
			}
		}

		rclcpp::spinOnce();
		

	}

	// send StopSendingData command
	RCLCPP_INFO(node->get_logger(), "Request Stop Sending Data.");
	sendStopSendingData(whill_fd);
	usleep(1000);
	RCLCPP_INFO(node->get_logger(), "Closing Serial Port.");
	closeComWHILL(whill_fd);
	return 0;
}


