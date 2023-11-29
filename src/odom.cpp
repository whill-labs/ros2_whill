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

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/transform_broadcaster.h"

#include "ros2_whill/odom.h"

#include <arpa/inet.h>
#include <fcntl.h>
#include <limits>
#include <math.h>
#include <netdb.h>
#include <netinet/in.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/epoll.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

Odometry::Odometry() {
  pose.x = pose.y = pose.theta = 0.0;
  velocity.x = velocity.y = velocity.theta = 0.0;
}

long double Odometry::confineRadian(long double rad) {
  if (rad >= M_PI) {
    rad -= 2.0 * M_PI;
  }
  if (rad <= -M_PI) {
    rad += 2.0 * M_PI;
  }
  return rad;
}

void Odometry::update(sensor_msgs::msg::JointState jointState, double dt) {
  // std::cout << dt << std::endl;
  if (dt == 0)
    return;

  double angle_vel_r = jointState.velocity[1];
  double angle_vel_l = -jointState.velocity[0];

  long double vr = angle_vel_r * wheel_radius_;
  long double vl = angle_vel_l * wheel_radius_;

  long double delta_L = (vr + vl) / 2.0;
  long double delta_theta = (vr - vl) / (2.0 * wheel_tread_);

  pose.x += delta_L * dt * cosl(pose.theta + delta_theta * dt / 2.0);
  pose.y += delta_L * dt * sinl(pose.theta + delta_theta * dt / 2.0);

  velocity.x = delta_L;
  velocity.y = 0.0;
  velocity.theta = delta_theta;

  double theta = pose.theta + delta_theta * dt;
  pose.theta = confineRadian(theta);

  return;
}

void Odometry::reset() {
  Space2D poseZero = {0, 0, 0};
  set(poseZero);
  velocity = poseZero;
}

void Odometry::set(Space2D pose) { this->pose = pose; }

Odometry::Space2D Odometry::getOdom() { return this->pose; }

nav_msgs::msg::Odometry Odometry::getROSOdometry() {
  nav_msgs::msg::Odometry odom;

  tf2::Quaternion quat_tf;
  quat_tf.setRPY(0.0, 0.0, pose.theta);
  geometry_msgs::msg::Quaternion odom_quat = tf2::toMsg(quat_tf);
  // geometry_msgs::msg::Quaternion odom_quat;
  // tf2::convert(quat_tf, odom_quat);

  // position
  odom.pose.pose.position.x = pose.x;
  odom.pose.pose.position.y = pose.y;
  odom.pose.pose.position.z = 0.0;
  odom.pose.pose.orientation = odom_quat;

  // velocity
  odom.twist.twist.linear.x = velocity.x;
  odom.twist.twist.linear.y = velocity.y;
  odom.twist.twist.linear.z = 0.0;
  odom.twist.twist.angular.x = 0.0;
  odom.twist.twist.angular.y = 0.0;
  odom.twist.twist.angular.z = velocity.theta;

  return odom;
}

geometry_msgs::msg::TransformStamped Odometry::getROSTransformStamped() {

  geometry_msgs::msg::TransformStamped odom_trans;

  tf2::Quaternion quat_tf;
  quat_tf.setRPY(0.0, 0.0, pose.theta);
  geometry_msgs::msg::Quaternion odom_quat = tf2::toMsg(quat_tf);
  // tf2::convert(quat_tf, odom_quat);

  odom_trans.transform.translation.x = pose.x;
  odom_trans.transform.translation.y = pose.y;
  odom_trans.transform.translation.z = 0.0;
  odom_trans.transform.rotation = odom_quat;

  return odom_trans;
}
