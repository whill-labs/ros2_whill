# ros2_whill
<img src="https://user-images.githubusercontent.com/2618822/44189349-e4f39800-a15d-11e8-9261-79edac310e6a.png" width="100px">

ROS2 package for WHILL Model CR

## ROS API

### Subscribed Topics

#### /whill/controller/joy [(sensor_msgs/Joy)](http://docs.ros.org/api/sensor_msgs/html/msg/Joy.html)
- Virtual WHILL joystick input. You can controll WHILL via this topic.

### Published Topics

#### /whill/states/joy [(sensor_msgs/Joy)](http://docs.ros.org/api/sensor_msgs/html/msg/Joy.html)
- Joystick status

#### /whill/states/joint_state [(sensor_msgs/JointState)](http://docs.ros.org/api/sensor_msgs/html/msg/JointState.html)
- Wheel rotate position(rad) and rotation velocity(rad/s)

#### /whill/states/imu [(sensor_msgs/Imu)](http://docs.ros.org/api/sensor_msgs/html/msg/Imu.html)
- IMU measured data. 
- 値バグってそう

#### /whill/states/battery_state [(sensor_msgs/BatteryState)](http://docs.ros.org/api/sensor_msgs/html/msg/BatteryState.html)
- Battery information


## Requirements
- Ubuntu 22.04
- ROS 2 Humble
- [ros2_whill_interfaces](https://github.com/WHILL/ros2_whill_interfaces)

## Build
In your shell:
```sh
cd ~/<your_ros2_ws>/src
git clone https://github.com/WHILL/ros2_whill_interfaces.git
git clone https://github.com/nanoshimarobot/ros2_whill.git
cd ~/<your_ros2_ws>
colcon build --packages-up-to ros2_whill
source install/local_setup.bash

```

## Launch nodes
### Controller node
```sh
ros2 launch ros2_whill modelc.launch.py
```

### In the case of opening serial port failed

Edit
```
/lib/udev/rules.d/50-udev-default.rules
```

And add:
```
KERNEL=="ttyUSB[0-9]*", MODE="0666"
```

## Call services
### Change Speed Profile
```sh
ros2 service call /whill/set_speed_profile_srv ros2_whill_interfaces/SetSpeedProfile '{s1: 4, fm1: 15, fa1: 16, fd1: 64, rm1: 10, ra1: 16, rd1: 56, tm1: 10, ta1: 56, td1: 72}'
```

### Power on
```sh
ros2 service call /whill/set_power_srv ros2_whill_interfaces/SetPower '{p0: 1}'
```

### Power off
```sh
ros2 service call /whill/set_power_srv ros2_whill_interfaces/SetPower'{p0: 0}'
```