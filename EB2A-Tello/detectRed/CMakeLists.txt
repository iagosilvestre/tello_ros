cmake_minimum_required(VERSION 3.3.0)
project(myproject)

# Build the talker and listener. Each one uses the following ROS packages,
# which we need to find_package() individually:
#   roscpp (the client library)
#   std_msgs (contains the std_msgs/String message type)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
# We've found them; now use their variables in the usual way to configure
# the compile and link steps.
# Note: we skip calling link_directories() because ROS packages follow the
# recommended CMake practice of returning absolute paths to libraries
include_directories(${roscpp_INCLUDE_DIRS})
include_directories(${std_msgs_INCLUDE_DIRS})

add_executable(detect_redPixel_image detect_redPixel_image.cpp)
ament_target_dependencies(detect_redPixel_image rclcpp geometry_msgs std_msgs sensor_msgs)
target_link_libraries(detect_redPixel_image ${roscpp_LIBRARIES} ${std_msgs_LIBRARIES})

# (optional) Install the executables.
install(TARGETS detect_redPixel_image
        DESTINATION bin)

