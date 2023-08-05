# ros2_whill
<img src="https://user-images.githubusercontent.com/2618822/44189349-e4f39800-a15d-11e8-9261-79edac310e6a.png" width="100px">

ROS2 package for WHILL Model CR

## humble-dev UNDER CONSTRUCTION!!!!
## crystal-dev -> humble-dev

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

#### /whill/states/battery_state [(sensor_msgs/BatteryState)](http://docs.ros.org/api/sensor_msgs/html/msg/BatteryState.html)
- Battery information


## Requirements
- Ubuntu 18.04 (Windows & Mac OS support is currently under development)
- ROS2 Crystal
- [ros2_whill_interfaces](https://github.com/WHILL/ros2_whill_interfaces)

## Build
In your shell:
```sh
cd ~/<your_ros2_ws>/src
git clone https://github.com/WHILL/ros2_whill_interfaces.git
git clone https://github.com/WHILL/ros2_whill.git
cd ~/<your_ros2_ws>
colcon build --packages-up-to ros2_whill
source install/local_setup.bash

```


### Build only ros_whill package
```sh
colcon build --packages-select ros2_whill
```

## SerialPort Settings

### Set

Edit your `~/.bashrc` (bash) or `~/.zshrc` (zsh) to add this line:

```sh
export TTY_WHILL=/dev/[YOUR SERIAL PORT DEVICE]
```
Setting will be applied automatically from next shell starting or booting up.

#### Apply setting immediately

In your shell:

(bash)
```bash
source ~/.bashrc
```

(zsh)
```zsh
source ~/.zshrc
```

### Check the current setting
```sh
echo $TTY_WHILL  # -> Should be /dev/[YOUR SERIAL PORT DEVICE]
```
### Set serial port as an argument of the launch file
Edit `serialport` value in the following parameter file and load the param file when you launch nodes.
```sh
vim ~/<your_ros2_ws>/src/ros2_whill/params/sample_param.yaml
```

## Launch nodes
### Controller node
```sh
ros2 run ros2_whill whill_modelc_controller
```
### Publisher node
```sh
ros2 run ros2_whill whill_modelc_publisher
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