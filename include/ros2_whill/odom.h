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

#ifndef __ODOM_H__
#define __ODOM_H__

#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

class Odometry {
private:
  long double confineRadian(long double rad);

  typedef struct {
    double x;
    double y;
    double theta;
  } Space2D;

  static constexpr double wheel_radius_ = 0.1325;
  static constexpr double wheel_tread_ = 0.248;

  Space2D pose;
  Space2D velocity;

public:
  Odometry();
  void update(sensor_msgs::msg::JointState joint, double dt);
  void set(Space2D pose);
  void reset();

  nav_msgs::msg::Odometry getROSOdometry();
  geometry_msgs::msg::TransformStamped getROSTransformStamped();
  Space2D getOdom();
};

#endif
