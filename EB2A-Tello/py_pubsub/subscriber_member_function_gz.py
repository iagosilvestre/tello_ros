# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


import pprint
import math
import rclpy
import threading
import numpy
import time
import av
import tf2_ros
import tf2_msgs.msg
import cv2
import time
import yaml
import os
import sys
from ament_index_python.packages import get_package_share_directory

from djitellopy import Tello

from rclpy.node import Node
from std_msgs.msg import Empty, UInt8, UInt8, Bool, String, Int8,Float32,Int16
from sensor_msgs.msg import Image, Imu, BatteryState, Temperature, CameraInfo
from geometry_msgs.msg import Twist, TransformStamped, PoseStamped, Pose
from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
import ament_index_python
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from std_msgs.msg import Float64MultiArray 
from gazebo_msgs.srv import SetEntityState,GetWorldProperties,GetEntityState,DeleteEntity,SpawnEntity
from geometry_msgs.msg import Vector3
from gazebo_msgs.msg import ModelState

class MinimalSubscriber(Node):

    def spin_until_service_complete(node, response):
        rclpy.spin_until_future_complete(node, response)
        if response.result() is not None:
            node.get_logger().info('SERVICE COMPLETE! RESULT:\n{}'.format(response.result()))
            return response.result()

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.declare_parameter('target_frame', 'telloCamera')
        self.target_frame = self.get_parameter(
            'target_frame').get_parameter_value().string_value
        self.subscription = self.create_subscription(String,'cmd_tello',self.listener_callback,1)
        self.subscription  # prevent unused variable warning

        #self.sub_drone_pose = self.create_subscription(tf2_msgs.msg.TFMessage,'tf',self.pose_callback,1)
        #self.subscription  # prevent unused variable warning
        # Declare parameters

        self.sub_drone_goto = self.create_subscription(String,'go_to',self.goto_callback,1)
        self.subscription  # prevent unused variable warning
        # Declare parameters

        self.sub_drone_goto = self.create_subscription(String,'color',self.color_callback,1)
        self.subscription  # prevent unused variable warning
        # Declare parameters

        


        Tello.RESPONSE_TIMEOUT = int(10.0)
        
        #self.tello = Tello()
        #self.tello.connect()
        #self.tello.set_speed(70)
        self.pub_conclude = self.create_publisher(String, 'conclude', 10)
        self.pub_image_raw = self.create_publisher(Image, '/drone1/image_raw', 1)
        self.pub_camera_info = self.create_publisher(CameraInfo, '/drone1/camera_info', 1)
        self.pub_reached_goal = self.create_publisher(Int8, 'reached_goal', 1)
        self.pub_agt_x = self.create_publisher(Float32, 'agt_x', 1)
        self.pub_agt_y = self.create_publisher(Float32, 'agt_y', 1)
        self.pub_agt_w = self.create_publisher(Float32, 'agt_w', 1)
        self.pub_cmd_reset = self.create_publisher(String, 'cmd_tello', 1)
        self.pub_det_red = self.create_publisher(Int16, 'detectRed', 1)
        self.pub_det_blue = self.create_publisher(Int16, 'detectBlue', 1)
        self.pub_det_fire_ext = self.create_publisher(Int16, 'fireExt', 1)
        self.pub_batt = self.create_publisher(Int16, 'battery', 1)

        self.pub_cmd_vel = self.create_publisher(Twist, '/drone1/cmd_vel', 1)
        self.pub_cmd_vel_dummy = self.create_publisher(Twist, '/cmd_vel', 1)
        self.pub_cmd_velY_dummy = self.create_publisher(Int16, '/cmd_velY', 1)

        # Publisher to control the model's pose
        self.publisher = self.create_publisher(
            ModelState,
            '/gazebo/set_model_state',
            10
        )
         # Create a service client to delete entities in Gazebo
        self.delete_entity_client = self.create_client(DeleteEntity, '/delete_entity')
        #self.tf_buffer = Buffer()
        #self.tf_listener = TransformListener(self.tf_buffer, self)
        #self.spawn_entity_client = self.create_client(SpawnEntity, '/spawn_entity')

        self.spawn_entity_client = self.create_client(SpawnEntity, '/spawn_entity')

        while not self.spawn_entity_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for "/spawn_entity" service...')
        self.get_logger().info('"/spawn_entity" service is available.')

        sdf_file_path = os.path.join(
        get_package_share_directory("tello_gazebo"), "models",
        "red_target", "model.sdf")

        print(sdf_file_path)
        
        
        self.current_x=0.0
        self.current_y=0.0
        self.current_w=0.0

        self.goal_x=0.0
        self.goal_y=0.0
        self.goal_w=0.0

        self.startgoto=0
        self.spawn=0

        self.ignCam=1

        self.thrs=0.1
        self.cameraMsg=Image()

        self.countFire=0

        self.start_video_capture()
        self.start_goto_pose()
        self.sub_cam_vrd = self.create_subscription(Image,'/drone1/image_raw',self.camVerdict_callback,1)

        
        #self.resize_object(Vector3(x=1.0, y=1.0, z=2.0))

        #self.delete_object('red_target')

        #self.combatFire()
        #time.sleep(5)
        #self.combatFire()
        #time.sleep(5)
        #self.combatFire()
        #self.delete_object('red_target2')
        #self.spawn_object('red_target2')


        #self.start_goto_pose()
        #self.start_tello_status()
        #self.start_tello_odom()
        #tello.takeoff()

        #tello.move_left(100)
        #tello.rotate_counter_clockwise(90)
        #tello.move_forward(100)
        #tello.land()
        #self.timer = self.create_timer(0.5, self.on_timer)
        self.model_name = 'tello_1'

        # Create a subscriber to the /gazebo/model_states topic
        self.getState = self.create_subscription(
            ModelStates,
            '/gazebo/model_states',
            self.model_states_callback,
            10  # QoS profile depth
        )
        self.getState  # prevent unused variable warning

    def delete_object(self, object_name):
        # Ensure the service client is available
        while not self.delete_entity_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        request = DeleteEntity.Request()
        request.name = object_name
        future = self.delete_entity_client.call_async(request)
        #future.add_done_callback(self.callback)
    
    def combatFire(self):
        # Ensure the service client is available
        fireExtinguished=Int16()
        target_fire="red_target_"+str(self.countFire)
        self.delete_object(target_fire)
        self.countFire=self.countFire+1
        if(self.countFire==3):
            fireExtinguished.data=1
            self.pub_det_fire_ext.publish(fireExtinguished)
    
    
    def spawn_object(self, object_name):
        # Ensure the service client is available
        while not self.spawn_entity_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        if(object_name=="red_target"):
            sdf_file_path = os.path.join(
        get_package_share_directory("tello_gazebo"), "models",
        "red_target", "model.sdf")
        elif(object_name=="blue_target"):
            sdf_file_path = os.path.join(
        get_package_share_directory("tello_gazebo"), "models",
        "blue_target", "model.sdf")
        elif(object_name=="white_target"):
            sdf_file_path = os.path.join(
        get_package_share_directory("tello_gazebo"), "models",
        "white_target", "model.sdf")
        

        # Set data for request
        request = SpawnEntity.Request()
        request.name = object_name
        request.xml = open(sdf_file_path, 'r').read()
        #request.robot_namespace = argv[1]
        #request.initial_pose.position.x = float(argv[2])
        #request.initial_pose.position.y = float(argv[3])
        #request.initial_pose.position.z = float(argv[4])

        #request.initial_pose.position.x = -2.0
        request.initial_pose.position.x = 1.7
        request.initial_pose.position.y = 0.0
        request.initial_pose.position.z = 1.3
        request.initial_pose.orientation.x = 0.001250731431127177
        request.initial_pose.orientation.y = 0.6816700348249676
        request.initial_pose.orientation.z = 0.001250731431127177
        request.initial_pose.orientation.w = 0.7316575940721876

        #request.initial_pose.orientation.x = 0.5262733023599449
        #request.initial_pose.orientation.y = 0.5339706903029998
        #request.initial_pose.orientation.z = -0.4805629580935845
        #request.initial_pose.orientation.w = 0.454940607583935

        future = self.spawn_entity_client.call_async(request)
        #rclpy.spin_until_future_complete(self, future)

        #if future.result() is not None:
        #    self.get_logger().info('Object %s inserted successfully' % object_name)
        #else:
        #    self.get_logger().warning('Failed to insert object %s' % object_name)

    def callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info('Object "%s" deleted successfully' % response.status_message)
            else:
                self.get_logger().warning('Failed to delete object "%s": %s' % (response.status_message))
        except Exception as e:
            self.get_logger().error('Service call failed %r' % (e,))

    def resize_object(self, size):
        print(" Trying to resize object")
        # Construct message to set model state
        model_state = ModelState()
        model_state.model_name = 'red_target'  # Replace with the name of your object
        model_state.reference_frame = 'world'  # Assuming resizing w.r.t world frame

        # Set the desired size
        model_state.pose.position.x = size.x
        model_state.pose.position.y = size.y
        model_state.pose.position.z = size.z

        # Publish the command to Gazebo
        self.publisher.publish(model_state)

    def color_callback(self, msg):
        # Find the index of the specified model in the ModelStates message
        if msg.data=="red":
            if self.spawn==1:
                #print("spawn = 1")
                self.delete_object('blue_target')
            self.spawn_object('red_target')
            self.spawn=1
            #print("spawned red")
        elif msg.data=="blue":
            if self.spawn==1:
                #print("spawn = 1")
                self.delete_object('red_target')
            self.spawn_object('blue_target')
            self.spawn=1
            #print("spawned blue")
        elif msg.data=="white":
            if self.spawn==1:
                #print("spawn = 1")
                self.delete_object('blue_target')
                self.delete_object('red_target')
            self.spawn_object('white_target')
            self.spawn=1
            #print("spawned blue")
        elif msg.data=="del":
            if self.spawn==1:
                #print("spawn = 1")
                self.delete_object('blue_target')
                self.delete_object('red_target')
            #print("spawned blue")


    def model_states_callback(self, msg):
        # Find the index of the specified model in the ModelStates message
        try:
            model_index = msg.name.index(self.model_name)
        except ValueError:
            self.get_logger().warning(f"Model '{self.model_name}' not found in model_states message.")
            return

        # Get the pose information for the specified model
        model_pose = msg.pose[model_index]
        self.current_x=model_pose.position.x
        self.current_y=model_pose.position.y
        self.current_w=model_pose.orientation.w
        # Print the localization information
        #self.get_logger().info(f"Model '{self.model_name}' localization:")
        #self.get_logger().info(f"Position: x={model_pose.position.x}, y={model_pose.position.y}, z={model_pose.position.z}")
        #self.get_logger().info(f"Orientation: x={model_pose.orientation.x}, y={model_pose.orientation.y}, z={model_pose.orientation.z}, w={model_pose.orientation.w}")

     
    def listener_callback(self, msg):
        baro=1
        x = msg.data.split(";")
        self.get_logger().info('I heard: "%s"' % x[0])
        if x[0] == "\"takeoff":
            self.tello.takeoff()
            #time.sleep(3)
            #self.tello.move_down(20)
        elif x[0] == "\"move_left":
            self.tello.move_left(int(x[1].strip("\"")))
        elif x[0] == "\"move_right":
            self.tello.move_right(int(x[1].strip("\"")))
        elif x[0] == "\"move_forward":
            self.tello.move_forward(int(x[1].strip("\"")))
        elif x[0] == "\"move_back":
            self.tello.move_back(int(x[1].strip("\"")))
        elif x[0] == "\"up":
            self.tello.move_up(int(x[1].strip("\"")))
        elif x[0] == "\"down":
            self.tello.move_down(int(x[1].strip("\"")))
        elif x[0] == "\"land":
            self.tello.land()
        elif x[0] == "\"rotate_ccw":
            self.tello.rotate_counter_clockwise(int(x[1].strip("\"")))
        elif x[0] == "\"rotate_cw":
            self.tello.rotate_clockwise(int(x[1].strip("\"")))
        elif x[0] == "\"keepalive":
            self.ignCam=0
        elif x[0] == "\"ign":
            self.ignCam=1
        elif x[0] == "\"combatFire":
            self.combatFire()
          
            #teste=1
            #teste=self.tello.query_speed()

    #def pose_callback(self, msg):
        #drone_x=msg.transforms.transform.translation.x
        #drone_y=msg.transforms.transform.translation.y
        #print("I believe I am at x = %.2f and y=  %.2f "  % (drone_x, drone_y))

    def camVerdict_callback(self, msg):
        self.cameraMsg=msg
    

    def goto_callback(self, msg):
        #if(self.startgoto==0):
        self.startgoto=1
        #    self.start_goto_pose()
        x = msg.data
        x = x.strip('"')
        x = x.strip('[]')
        # Split the string by comma and convert values to floats
        vector = [float(x) for x in x.split(',')]
        self.ignCam=1

        

        
        self.get_logger().info('x goal: "%s"' % vector[0])
        self.get_logger().info('y goal: "%s"' % vector[1])
        self.get_logger().info('w goal: "%s"' % vector[2])

        self.goal_x=float(vector[0])
        self.goal_y=float(vector[1])
        self.goal_w=float(vector[2])
        #x = x.replace("[", "") 
        #x = x.replace("]", "") 
        #x = msg.data.split(",")
        #self.goal_x=float(x[0].strip("\""))
        #self.goal_y=float(x[1].strip("\""))
        #self.goal_w=float(x[2].strip("\""))
        
        

    # Start goal pose thread.
    def start_goto_pose(self, rate=1.0/60.0):
        def goto_pose_thread():
            print("Starting goto thread")
            msg=Twist()
            msg_x=Float32()
            msg_y=Float32()
            msg_w=Float32()
            batt=Int16()
            velY=Int16()
            while True:
                if(self.startgoto==1):
                    msg.linear.x = 0.0
                    msg.linear.y = 0.0
                    msg.angular.z = 0.0
                    velY.data=0
                    fast=0.05
                    slow=0.01
                    speed=0.0
                    if((abs(self.goal_x-self.current_x)+abs(self.goal_y-self.current_y))>self.thrs*5):
                        speed=fast
                    else:
                        speed=slow
                    if(abs(self.goal_x-self.current_x)>self.thrs):
                        #print("front/back")
                        if(self.goal_x>self.current_x):
                            msg.linear.x = speed
                        else:
                            msg.linear.x = -speed
                    if(abs(self.goal_y-self.current_y)>self.thrs):
                        #print("left/right")
                        if(self.goal_y>self.current_y):
                            msg.linear.y = speed
                            velY.data=int(speed*100)
                        else:
                            msg.linear.y = -speed
                            velY.data=int(-speed*100)
                    if(abs(self.goal_w-self.current_w)>0.001):
                        #print("rotation")
                        if(self.goal_w>self.current_w):
                            msg.angular.z = 0.03

                        else:
                            msg.angular.z = -0.03  
                    if(abs(self.goal_w-self.current_w)>0.5):
                        #print("rotation")
                        if(self.goal_w>self.current_w):
                            msg.angular.z = 0.05

                        else:
                            msg.angular.z = -0.05  
                #if(self.startgoto==1):                
                    self.pub_cmd_vel.publish(msg) #Only for ROS-Gazebo Simulation
                     #Only for ROS-Gazebo Simulation- dummmy for RosMonitoring check
                msg_x.data=self.current_x
                msg_y.data=self.current_y
                msg_w.data=self.current_w
                batt.data=99
                self.pub_agt_x.publish(msg_x)
                self.pub_agt_y.publish(msg_y)
                self.pub_agt_w.publish(msg_w)

                self.pub_cmd_vel_dummy.publish(msg)
                self.pub_cmd_velY_dummy.publish(velY)
                #self.pub_batt.publish(batt)
                #print("Goal_x = %.2f and Drone_x=  %.2f "  % (self.goal_x, self.current_x))
                #print("Goal_y = %.2f and Drone_y=  %.2f "  % (self.goal_y, self.current_y))
                #print("Goal_w = %.2f and Drone_w=  %.2f "  % (self.goal_w, self.current_w))
                #print("Speed = %.2f \n"  % (speed))

                time.sleep(rate)
                

        # We need to run the recorder in a seperate thread
        thread = threading.Thread(target=goto_pose_thread)
        thread.start()
        return thread

    
    
    # Start video capture thread.
    def start_video_capture(self, rate=1.0/15.0):
        # Enable tello stream
        #self.tello.streamon()
        #self.tello.set_video_bitrate(1)
        #self.tello.set_video_resolution("Tello.RESOLUTION_480P")
        # OpenCV bridge
        self.bridge = CvBridge()
        redData=Int16()
        blueData=Int16()
        def video_capture_thread():
            time.sleep(2)
            while True:
                # Count red and blue pixels in the received image
                bridge = CvBridge()
                cv_image = bridge.imgmsg_to_cv2(self.cameraMsg, desired_encoding="rgb8")
                

                # Define the lower and upper bounds for the red color in RGB
                #lower_red = numpy.array([200, 50, 50])
                #upper_red = numpy.array([255, 100, 100])
                lower_red = numpy.array([155, 0, 0])
                upper_red = numpy.array([255, 150, 150])

                # Define the lower and upper bounds for the blue color in RGB
                lower_blue = numpy.array([30, 30, 150])
                upper_blue = numpy.array([150, 150, 255])

                # Create masks to extract only the red and blue pixels
                red_mask = cv2.inRange(cv_image, lower_red, upper_red)
                blue_mask = cv2.inRange(cv_image, lower_blue, upper_blue)

                # Count the number of red and blue pixels
                red_pixel_count = numpy.sum(red_mask == 255)
                blue_pixel_count = numpy.sum(blue_mask == 255)

                if(self.ignCam==1):
                    redData.data=0
                else:
                    #redData.data=1
                    redData.data=int(red_pixel_count)
                if(self.ignCam==1):
                    blueData.data=0   
                elif(blue_pixel_count>200):
                    #blueData.data=1
                    blueData.data=int(blue_pixel_count)
                
                self.pub_det_red.publish(redData)
                self.pub_det_blue.publish(blueData)
                print(f'The number of red pixels in the image is: {redData.data}')
                print(f'The number of blue pixels in the image is: {blueData.data}')

                time.sleep(rate)
        # We need to run the recorder in a seperate thread, otherwise blocking options would prevent frames from getting added to the video
        thread = threading.Thread(target=video_capture_thread)
        thread.start()
        return thread 	
      
    

def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
