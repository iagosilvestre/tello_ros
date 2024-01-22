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

from djitellopy import Tello

from rclpy.node import Node
from std_msgs.msg import Empty, UInt8, UInt8, Bool, String, Int8,Float32,Int16
from sensor_msgs.msg import Image, Imu, BatteryState, Temperature, CameraInfo
from geometry_msgs.msg import Twist, TransformStamped, PoseStamped
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
import ament_index_python
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from std_msgs.msg import Float64MultiArray 

class MinimalSubscriber(Node):

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

        Tello.RESPONSE_TIMEOUT = int(10.0)
        
        self.tello = Tello()
        self.tello.connect()
        self.tello.set_speed(70)
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
        self.pub_batt = self.create_publisher(Int16, 'battery', 1)

        self.pub_cmd_vel = self.create_publisher(Twist, '/drone1/cmd_vel', 1)
        
        #self.tf_buffer = Buffer()
        #self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.current_x=0.0
        self.current_y=0.0
        self.current_w=0.0

        self.goal_x=0.0
        self.goal_y=0.0
        self.goal_w=0.0

        self.startgoto=0

        self.thrs=0.03

        self.start_video_capture()
        self.start_goto_pose()
        #self.sub_cam_vrd = self.create_subscription(Image,'/drone1/image_raw',self.camVerdict_callback,1)
        
        #self.start_goto_pose()
        #self.start_tello_status()
        #self.start_tello_odom()
        #tello.takeoff()

        #tello.move_left(100)
        #tello.rotate_counter_clockwise(90)
        #tello.move_forward(100)
        #tello.land()
        #self.timer = self.create_timer(0.5, self.on_timer)


    def on_timer(self):
        """
        Callback function.
        This function gets called at the specific time interval.
        """
        # Store frame names in variables that will be used to
        # compute transformations
        from_frame_rel = self.target_frame
        to_frame_rel = 'world'
    
        trans = None
        
        try:
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform(
                        'world',
                        'telloCamera',
                        now)
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')
            return
        
        # Publish the 2D pose
        #drone_x = trans.transform.translation.x
        #drone_y = trans.transform.translation.y 
        self.current_x = trans.transform.translation.x
        self.current_y = trans.transform.translation.y   
        self.current_w = trans.transform.rotation.w
        #print("I believe I am at x = %.3f and y=  %.3f "  % (trans.transform.translation.x, trans.transform.translation.y))
        roll, pitch, yaw = self.euler_from_quaternion(
        trans.transform.rotation.x,
        trans.transform.rotation.y,
        trans.transform.rotation.z,
        trans.transform.rotation.w)      
        self.current_yaw = yaw    
        msg = Float64MultiArray()
        msg.data = [self.current_x, self.current_y, self.current_yaw]   
        #self.publisher_2d_pose.publish(msg) 
   
    def euler_from_quaternion(self, x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
        
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
        
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
        
        return roll_x, pitch_y, yaw_z # in radians
        



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
            self.tello.set_speed(70)
          
            #teste=1
            #teste=self.tello.query_speed()

    #def pose_callback(self, msg):
        #drone_x=msg.transforms.transform.translation.x
        #drone_y=msg.transforms.transform.translation.y
        #print("I believe I am at x = %.2f and y=  %.2f "  % (drone_x, drone_y))

    def count_red_and_blue_pixels(self,image):
        # Convert the ROS image message to an OpenCV image
        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(image, desired_encoding="rgb8")

        # Define the lower and upper bounds for the red color in RGB
        lower_red = numpy.array([100, 0, 0])
        upper_red = numpy.array([255, 100, 100])

        # Define the lower and upper bounds for the blue color in RGB
        lower_blue = numpy.array([0, 0, 100])
        upper_blue = numpy.array([100, 100, 255])

        # Create masks to extract only the red and blue pixels
        red_mask = cv2.inRange(cv_image, lower_red, upper_red)
        blue_mask = cv2.inRange(cv_image, lower_blue, upper_blue)

        # Count the number of red and blue pixels
        red_pixel_count = numpy.sum(red_mask == 255)
        blue_pixel_count = numpy.sum(blue_mask == 255)

        return red_pixel_count, blue_pixel_count

    def camVerdict_callback(self, msg):
        redData=Int8()
        blueData=Int8()
        # Count red and blue pixels in the received image
        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")

        # Define the lower and upper bounds for the red color in RGB
        lower_red = numpy.array([200, 50, 50])
        upper_red = numpy.array([255, 100, 100])

        # Define the lower and upper bounds for the blue color in RGB
        lower_blue = numpy.array([50, 50, 200])
        upper_blue = numpy.array([100, 100, 255])

        # Create masks to extract only the red and blue pixels
        red_mask = cv2.inRange(cv_image, lower_red, upper_red)
        blue_mask = cv2.inRange(cv_image, lower_blue, upper_blue)

        # Count the number of red and blue pixels
        red_pixel_count = numpy.sum(red_mask == 255)
        blue_pixel_count = numpy.sum(blue_mask == 255)

        if(red_pixel_count>4000):
            redData.data=1
        else:
            redData.data=0
        if(blue_pixel_count>2000):
            blueData.data=1
        else:
            blueData.data=0   
        self.pub_det_red.publish(redData)
        self.pub_det_blue.publish(blueData)
        print(f'The number of red pixels in the image is: {red_pixel_count}')
        print(f'The number of blue pixels in the image is: {blue_pixel_count}')

    #def camVerdict_callback(self, msg):
    #    print("Received Cam img ")
    #    image = cv2.imread(msg.data)

    def goto_callback(self, msg):
        #if(self.startgoto==0):
        self.startgoto=1
        #    self.start_goto_pose()
        x = msg.data
        x = x.replace("[", "") 
        x = x.replace("]", "") 
        x = msg.data.split(",")
        #self.get_logger().info('x goal: "%s"' % x[0])
        #self.get_logger().info('y goal: "%s"' % x[1])
        self.goal_x=float(x[0].strip("\"["))
        self.goal_y=float(x[1].strip("\""))
        #self.goal_w=0.49
        self.goal_w=float(x[2].strip("\"]"))
        
        

    # Start goal pose thread.
    def start_goto_pose(self, rate=1.0/30.0):
        def goto_pose_thread():
            print("Starting goto thread")
            msg=Twist()
            msg_x=Float32()
            msg_y=Float32()
            msg_w=Float32()
            while True:
                msg.linear.x = 0.0
                msg.linear.y = 0.0
                msg.angular.z = 0.0
                if(abs(self.goal_x-self.current_x)>self.thrs):
                    print("front/back")
                    if(self.goal_x>self.current_x):
                        msg.linear.x = -0.03
                    else:
                        msg.linear.x = 0.03
                if(abs(self.goal_y-self.current_y)>self.thrs):
                    print("left/right")
                    if(self.goal_y>self.current_y):
                        msg.linear.y = -0.03
                    else:
                        msg.linear.y = 0.03 
                if(abs(self.goal_w-self.current_w)>0.02):
                    print("rotation")
                    if(self.goal_w>self.current_w):
                        msg.angular.z = -0.03
                    else:
                        msg.angular.z = 0.03   
                if(self.startgoto==1):                
                    self.pub_cmd_vel.publish(msg) #Only for ROS-Gazebo Simulation
                msg_x.data=self.current_x
                msg_y.data=self.current_y
                msg_w.data=self.current_w
                self.pub_agt_x.publish(msg_x)
                self.pub_agt_y.publish(msg_y)
                self.pub_agt_w.publish(msg_w)
                #print("Desired velocity x = %.2f and y=  %.2f "  % (msg.linear.x, msg.linear.y))
                #print("Goal_x = %.2f and Drone_x=  %.2f \n"  % (self.goal_x, self.current_x))
                #print("Goal_y = %.2f and Drone_y=  %.2f \n"  % (self.goal_y, self.current_y))
                #print("Goal_w = %.2f and Drone_w=  %.2f \n"  % (self.goal_w, self.current_w))
                #print("Vel_x = %.2f and Vel_y=  %.2f \n"  % (msg.linear.x , msg.linear.y))
                time.sleep(rate)
                

        # We need to run the recorder in a seperate thread
        thread = threading.Thread(target=goto_pose_thread)
        thread.start()
        return thread
    
    # Start video capture thread.
    def start_video_capture(self, rate=1.0/15.0):
        # Enable tello stream
        self.tello.streamon()
        #self.tello.set_video_bitrate(1)
        #self.tello.set_video_resolution("Tello.RESOLUTION_480P")
        # OpenCV bridge
        self.bridge = CvBridge()
        redData=Int16()
        blueData=Int16()
        battData=Int16()
        
        def video_capture_thread():
            frame_read = self.tello.get_frame_read()

            while True:
                # Get frame from drone
                frame = frame_read.frame

                # Publish opencv frame using CV bridge
                msg = self.bridge.cv2_to_imgmsg(numpy.array(frame), 'rgb8')
                msg.header.frame_id = 'drone'
                self.pub_image_raw.publish(msg)

                

                cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")
                lower_red = numpy.array([180, 0, 0])
                upper_red = numpy.array([255, 150, 150])

                # Define the lower and upper bounds for the blue color in RGB
                lower_blue = numpy.array([0, 0, 180])
                upper_blue = numpy.array([150, 150, 255])

                # Create masks to extract only the red and blue pixels
                red_mask = cv2.inRange(cv_image, lower_red, upper_red)
                blue_mask = cv2.inRange(cv_image, lower_blue, upper_blue)

                # Count the number of red and blue pixels
                red_pixel_count = numpy.sum(red_mask == 255)
                blue_pixel_count = numpy.sum(blue_mask == 255)

                if(red_pixel_count>200):
                    #redData.data=1
                    redData.data=int(red_pixel_count)
                else:
                    redData.data=0
                if(blue_pixel_count>200):
                    #blueData.data=1
                    blueData.data=int(blue_pixel_count)
                else:
                    blueData.data=0   
                self.pub_det_red.publish(redData)
                self.pub_det_blue.publish(blueData)
                battData.data=int(self.tello.get_battery())
                self.pub_batt.publish(battData)
                print(f'Battery : {battData}')
                print(f'The number of red pixels in the image is: {red_pixel_count}')
                print(f'The number of blue pixels in the image is: {blue_pixel_count}')

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
