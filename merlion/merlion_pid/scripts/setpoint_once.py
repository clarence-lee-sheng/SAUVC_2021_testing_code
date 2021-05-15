#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64

if __name__ == "__main__":
    rospy.init_node('setpoint_once')

    time = 0
    rate = rospy.Rate(10)
    while not rospy.is_shutdown() and time < 3:
        rospy.Publisher('pitch/setpoint', Float64, queue_size=10).publish(Float64(0))
        rospy.Publisher('roll/setpoint',  Float64, queue_size=10).publish(Float64(0))
        rospy.Publisher('depth/setpoint', Float64, queue_size=10).publish(Float64(0.2))
        rospy.Publisher('speed/setpoint', Float64, queue_size=10).publish(Float64(0))
        rate.sleep()
        time += 0.1
