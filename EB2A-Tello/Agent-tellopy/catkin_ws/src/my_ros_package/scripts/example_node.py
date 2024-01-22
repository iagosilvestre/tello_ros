#!/usr/bin/env python
import time
import math
import rospy
from std_msgs.msg import String
from std_msgs.msg import Bool
from std_msgs.msg import Int32


class ExampleNode:
    def __init__(self):
        # Initialize node
        rospy.init_node('example_node')

        #critical things
        self.reaction_times = []
        self.perception_times = []

        # Create publisher
        self.pubVal1 = rospy.Publisher('value1', Int32, queue_size=10)
        self.pubVal2 = rospy.Publisher('value2', Int32, queue_size=10)

        self.ctd = 0
        self.isFinished = False
        self.file = open("ariacReactionTimes.log", "w")
        self.startTime = time.time()

        # Create subscriber
        rospy.Subscriber('current_time', String, self.callback1)
        rospy.Subscriber('finish', Bool, self.callback2)

    def run(self):
        rate = rospy.Rate(10) # 10 Hz
        while not (rospy.is_shutdown() or self.isFinished):
            # Publish message
            message1 = Int32()
            message1.data = self.ctd
            self.pubVal1.publish(message1) 
            
            if self.ctd % 20 == 0:
                message2 = Int32()
                message2.data = int(self.ctd/20)
                self.pubVal2.publish(message2)
                self.perception_times.append(time.perf_counter())


            self.ctd = self.ctd + 1
            rate.sleep()
        self.recordTimes()
        rospy.signal_shutdown('Node is shutting down.')

    def recordTimes(self):
        if len(self.perception_times) == len(self.reaction_times):
            for pTime, rTime in zip(self.perception_times, self.reaction_times):
                elapsed_time = (rTime - pTime) * 1000 
                self.file.write(f"{pTime}\t{rTime}\t{elapsed_time}\n")
        else:
            pIter = iter(self.perception_times)
            next(pIter)
            for rTime in self.reaction_times:
                pTime = next(pIter)
                elapsed_time = (rTime - pTime) * 1000 
                self.file.write(f"{pTime}, {rTime}, {elapsed_time}\n")
        total_time = time.time()-self.startTime        
        if not self.file.closed:
            self.file.write(f"Sent msgs: {len(self.perception_times)}\n")
            self.file.write(f"Received msgs: {len(self.reaction_times)}\n")
            self.file.write(f"Elapsed_time(s): {total_time}\n")
            self.file.close()


    def callback1(self, message):
        # Print received message
        self.reaction_times.append(time.perf_counter())
        rospy.loginfo("Received msg: %s", message.data)

    def callback2(self, message):
        # Print received message
        self.isFinished = True
        rospy.loginfo("FINISHED")

if __name__ == '__main__':
    node = ExampleNode()
    node.run()
