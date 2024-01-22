# tello_ros
Tello ROS-Gazebo Framework

## Installation

### 1. Set up your Linux environment

Set up a Ubuntu 20.04 box or VM.

Also install asio:
~~~
sudo apt install libasio-dev
~~~

### 2. Set up your ROS environment

[Install ROS2 Foxy](https://docs.ros.org/) with the `ros-foxy-desktop` option.

If you install binaries, be sure to also install the 
[development tools and ROS tools](https://docs.ros.org/en/foxy/Installation/Ubuntu-Development-Setup.html)
from the source installation instructions.

Install these additional packages:
~~~
sudo apt install ros-foxy-cv-bridge ros-foxy-camera-calibration-parsers
~~~

### 3. Install `tello_ros`

Download, compile and install `tello_ros`:
~~~
mkdir -p ~/tello_ros_ws/src
cd ~/tello_ros_ws/src
git clone https://github.com/clydemcqueen/tello_ros.git
git clone https://github.com/ptrmu/ros2_shared.git
cd ..
source /opt/ros/foxy/setup.bash
# If you didn't intall Gazebo, skip tello_gazebo while building:
colcon build --event-handlers console_direct+
