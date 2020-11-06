#!/usr/bin/env python
import rospy
from sensor_msgs.msg  import Imu
import tf
def convert(msg):
    (r,p,y)=tf.transformations.euler_from_quaternion([msg.orientation.x,msg.orientation.y, msg.orientation.z, msg.orientation.w])
    print(msg.orientation.x,msg.orientation.y, msg.orientation.z, msg.orientation.w)
    print(r,p,y)



rospy.init_node('transform')
rospy.Subscriber("/imu/data",Imu , convert)

rospy.spin()