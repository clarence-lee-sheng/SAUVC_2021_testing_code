#!/usr/bin/env python3
import serial
import time
import rospy
import numpy as np
from geometry_msgs.msg import Twist
from sensor_msgs.msg import FluidPressure


class ThrusterManagerNode:
    def __init__(self):
        self.node_name = 'thruster_manager'
        self.tranform_matrix = np.array([
            # x  y   z roll pitch yaw
            [0,  0,  1,  1, -1,  0],  # left front motor
            [0,  0, -1,  1,  1,  0],  # right front motor
            [-1, 0,  0,  0,  0,  1],  # left rear motor
            [1,  0,  0,  0,  0,  1],  # right rear motor
            [0,  0, -1,  0, -1,  0]   # rear motor
        ])
        self.output = np.array([1488.0] * len(self.tranform_matrix))

        rospy.init_node(self.node_name)
        rospy.Subscriber(self.node_name + '/input', Twist, self.input_callback)
        pressure_pub = rospy.Publisher('pressure', FluidPressure, queue_size=10)

        serial_port = rospy.get_param('~port')

        while True:
            try:
                ser = serial.Serial(serial_port, 19200, timeout=1)
                rate = rospy.Rate(100)
                while not rospy.is_shutdown():
                    ser.write((','.join(str(int(n)) for n in self.output.ravel()) + '\n').encode('utf-8'))
                    message = ser.readline().decode('utf-8').strip()
                    if message.startswith('message'):
                        print(message)
                    elif message.startswith('pressure'):
                        try:
                            data = float(message.split(':')[1])  # in mBar
                            message = FluidPressure()
                            message.fluid_pressure = data / 10  # in kPa
                            pressure_pub.publish(message)
                        except Exception as e:
                            print(e)
                    rate.sleep()
            except serial.serialutil.SerialException as e:
                print(e)
                try:
                    ser.close()
                except Exception:
                    pass
                time.sleep(0.3)

    def input_callback(self, msg: Twist):
        control_vector = np.array([
            [msg.linear.x], [msg.linear.y], [msg.linear.z],
            [msg.angular.x], [msg.angular.y], [msg.angular.z]
        ])
        self.output = (self.tranform_matrix @ control_vector) * 400 + 1488


if __name__ == '__main__':
    ThrusterManagerNode()
    rospy.spin()
