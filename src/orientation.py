#!/usr/bin/env python
import kalman 
import os
import sys
import time
#import smbus
import numpy as np
import rospy
from sensor_msgs.msg import Imu
global acc_x,acc_y,acc_z,groll,gpitch,gyaw,then,now
now=0
then=0


def compute_orientation(msg,args):
    then=args
    acc_x=msg.linear_acceleration.x
    acc_y=msg.linear_acceleration.y
    acc_z=msg.linear_acceleration.z
    groll=msg.angular_velocity.x
    gpitch=msg.angular_velocity.y
    gyaw=msg.angular_velocity.z
    now=int(round(time.time() * 1000))
    
    #print(gyaw)

    if (now-then>10):
        dt=now-then
        filter.computeAndUpdateRollPitch(acc_x,acc_y,acc_z,groll,gpitch,dt)
        #filter.computeAndUpdateRollPitch()
        print(filter.roll)
        then=now

filter=kalman.Kalman()

rospy.init_node("orientation")
rospy.Subscriber("/imu/data_raw", Imu, compute_orientation,(then))
rospy.spin()
