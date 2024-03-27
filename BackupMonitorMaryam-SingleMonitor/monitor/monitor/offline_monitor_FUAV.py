#!/usr/bin/env python

# begin imports
import json
import yaml
import websocket
import sys
import rclpy
import rosidl_runtime_py
from rclpy.node import Node
from threading import *
from rosmonitoring_interfaces.msg import *
from std_msgs.msg import *
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from geometry_msgs.msg import Twist
# done import

class ROSMonitor_offline_monitor_FUAV(Node):


	def callbackdetectRed(self,data):
		self.get_logger().info("monitor has observed "+ str(data))
		dict= rosidl_runtime_py.message_to_ordereddict(data)
		dict['topic']='detectRed'
		dict['time']=float(self.get_clock().now().to_msg().sec)
		self.ws_lock.acquire()
		self.logging(dict)
		self.ws_lock.release()
		self.get_logger().info("event successfully logged")

	def callbackagentReact(self,data):
		self.get_logger().info("monitor has observed "+ str(data))
		dict= rosidl_runtime_py.message_to_ordereddict(data)
		dict['topic']='agentReact'
		dict['time']=float(self.get_clock().now().to_msg().sec)
		self.ws_lock.acquire()
		self.logging(dict)
		self.ws_lock.release()
		self.get_logger().info("event successfully logged")

	def callbackbattery(self,data):
		self.get_logger().info("monitor has observed "+ str(data))
		dict= rosidl_runtime_py.message_to_ordereddict(data)
		dict['topic']='battery'
		dict['time']=float(self.get_clock().now().to_msg().sec)
		self.ws_lock.acquire()
		self.logging(dict)
		self.ws_lock.release()
		self.get_logger().info("event successfully logged")

	def callbackagLand(self,data):
		self.get_logger().info("monitor has observed "+ str(data))
		dict= rosidl_runtime_py.message_to_ordereddict(data)
		dict['topic']='agLand'
		dict['time']=float(self.get_clock().now().to_msg().sec)
		self.ws_lock.acquire()
		self.logging(dict)
		self.ws_lock.release()
		self.get_logger().info("event successfully logged")

	def callbackcmd_vel(self,data):
		self.get_logger().info("monitor has observed "+ str(data))
		dict= rosidl_runtime_py.message_to_ordereddict(data)
		dict['topic']='cmd_vel'
		dict['time']=float(self.get_clock().now().to_msg().sec)
		self.ws_lock.acquire()
		self.logging(dict)
		self.ws_lock.release()
		self.get_logger().info("event successfully logged")

	def __init__(self,monitor_name,log,actions):
		self.monitor_publishers={}
		self.config_publishers={}
		self.config_subscribers={}
		self.config_client_services={}
		self.config_server_services={}
		self.services_info={}
		self.dict_msgs={}
		self.ws_lock=Lock()
		self.name=monitor_name
		self.actions=actions
		self.logfn=log
		self.topics_info={}
		super().__init__(self.name)
		# creating the verdict and error publishers for the monitor
		self.monitor_publishers['error']=self.create_publisher(topic=self.name+'/monitor_error',msg_type=MonitorError,qos_profile=1000)

		self.monitor_publishers['verdict']=self.create_publisher(topic=self.name+'/monitor_verdict',msg_type=String,qos_profile=1000)

		# done creating monitor publishers

		self.publish_topics=False
		self.topics_info['detectRed']={'package': 'std_msgs.msg', 'type': 'Int16'}
		self.topics_info['agentReact']={'package': 'std_msgs.msg', 'type': 'String'}
		self.topics_info['battery']={'package': 'std_msgs.msg', 'type': 'Int16'}
		self.topics_info['agLand']={'package': 'std_msgs.msg', 'type': 'String'}
		self.topics_info['cmd_vel']={'package': 'geometry_msgs.msg', 'type': 'Twist'}
		self.config_subscribers['detectRed']=self.create_subscription(topic='detectRed',msg_type=Int16,callback=self.callbackdetectRed,qos_profile=1000)

		self.config_subscribers['agentReact']=self.create_subscription(topic='agentReact',msg_type=String,callback=self.callbackagentReact,qos_profile=1000)

		self.config_subscribers['battery']=self.create_subscription(topic='battery',msg_type=Int16,callback=self.callbackbattery,qos_profile=1000)

		self.config_subscribers['agLand']=self.create_subscription(topic='agLand',msg_type=String,callback=self.callbackagLand,qos_profile=1000)

		self.config_subscribers['cmd_vel']=self.create_subscription(topic='cmd_vel',msg_type=Twist,callback=self.callbackcmd_vel,qos_profile=1000)

		self.get_logger().info('Monitor' + self.name + ' started and ready' )
		self.get_logger().info('Logging at' + self.logfn )



	def logging(self,json_dict):
		try:
			with open(self.logfn,'a+') as log_file:
				log_file.write(json.dumps(json_dict)+'\n')
			self.get_logger().info('Event logged')
		except:
			self.get_logger().info('Unable to log the event')

def main(args=None):
	rclpy.init(args=args)
	log = './log_FUAV_offline.txt'
	actions = {}
	actions['detectRed']=('log',0)
	actions['agentReact']=('log',0)
	actions['battery']=('log',0)
	actions['agLand']=('log',0)
	actions['cmd_vel']=('log',0)
	monitor = ROSMonitor_offline_monitor_FUAV('offline_monitor_FUAV',log,actions)
	rclpy.spin(monitor)
	monitor.ws.close()
	monitor.destroy_node()
	rclpy.shutdown()

if __name__ == '__main__':
	main()
