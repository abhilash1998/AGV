#!/usr/bin/env python

import rospy
import os 
import time
from geometry_msgs.msg import Vector3
import madgwick 
import kalman

filter=madgwick.Madgwick()
filter_k=kalman.Kalman()


class RosNode:
    def __init__(self):
        self.acc_x=0
        self.acc_y=0
        self.acc_z=0
        self.gyro_x=0
        self.gyro_y=0
        self.gyro_z=0
        self.mag_x=0
        self.mag_y=0
        self.mag_z=0
        self.then=0
        rospy.init_node("ros_node")
        rospy.loginfo("Starting RosNode.")
        
        pass
    
    def acc_callback(self,msg):
        self.acc_x=msg.x
        self.acc_y=msg.y
        self.acc_z=msg.z
    def gyro_callback(self,msg):
        self.gyro_x=msg.x
        self.gyro_y=msg.y
        self.gyro_z=msg.z
        #print(self.gyro_x)
    def mag_callback(self,msg):
        self.mag_x=msg.x
        self.mag_y=msg.y
        self.mag_z=msg.z
        now=int(round(time.time() * 1000))
        if (now-self.then>10):
            dt=now-self.then
            filter.updateRollPitchYaw(self.acc_x,self.acc_y,self.acc_z,self.gyro_x,self.gyro_y,self.gyro_z,self.mag_x,self.mag_y,self.mag_z,dt)
            filter_k.computeAndUpdateRollPitchYaw(self.acc_x,self.acc_y,self.acc_z,self.gyro_x,self.gyro_y,self.gyro_z,self.mag_x,self.mag_y,self.mag_z,dt)
            #filter.computeAndUpdateRollPitch()
            print(filter_k.yaw)
            self.then=now

# if __name__ == "__main__":

ros_node = RosNode()
rospy.Subscriber("imu/acc_data", Vector3, ros_node.acc_callback)
rospy.Subscriber("imu/gyro_data", Vector3, ros_node.gyro_callback)
rospy.Subscriber("imu/mag_data", Vector3, ros_node.mag_callback)

rospy.spin()
