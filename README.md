# tello_ros
Tello ROS-Gazebo Framework, using Jason BDI Agents to control the drone.

## ROS Topics of Interest
<pre>
- /detectRed	std_msgs/Int16		- Amount of red pixels the drone camera is seeing
- /detectBlue	std_msgs/Int16		- Amount of blue pixels the drone camera is seeing
- /battery	std_msgs/Int16		- Battery percentage of the drone
- /drone1/cmd_vel	geometry_msgs/Twist		- Used to command tello velocity
- /agLand	std_msgs/String		- Topic where Agent publishes landing info
- /agentReact	std_msgs/String		- Topic where Agent publishes reaction to Image processing Node


    
</pre>
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
sudo apt install ros-foxy-cv-bridge ros-foxy-camera-calibration-parsers ros-foxy-gazebo-ros-pkgs
~~~

If you run into the **No namespace found** error re-set `GAZEBO_MODEL_PATH`:

    export GAZEBO_MODEL_PATH=${PWD}/install/tello_gazebo/share/tello_gazebo/models
    source /usr/share/gazebo/setup.sh

### 3. Install `tello_ros`

Download, compile and install `tello_ros`:
~~~
mkdir -p ~/tello_ros_ws/src
cd ~/tello_ros_ws/src
git clone https://github.com/iagosilvestre/tello_ros.git
git clone https://github.com/ptrmu/ros2_shared.git
cd ..
source /opt/ros/foxy/setup.bash
colcon build --event-handlers console_direct+
~~~

## Run Simulation
Start the gazebo simulation by running the launch script:
~~~
ros2 launch tello_gazebo simple_launch.py 
~~~
Start the python rclpy node :
~~~
ros2 run py_pubsub listener-gz
~~~
Navigate to the BDI Agent folder(/EB2A-Tello/Agent-tellopy-gz/..) and initiate the agent:
~~~
./gradlew
~~~


