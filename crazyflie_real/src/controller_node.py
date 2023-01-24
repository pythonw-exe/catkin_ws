#!/usr/bin/env python3

import rospy
from sys import argv
from math import degrees
import numpy as np
from crazyflie_real.msg import CMD_Flat

class controller_class():
    def __init__(self, u):
        self.pub_cmd = rospy.Publisher("/controller/cmd_flat", CMD_Flat, queue_size=1)
        self.u = u

    def send_control(self):
        self.pub_cmd.publish(self.u)

    def update_u(self):
        self.u.position.x = -2.0
        self.u.position.y = 0.1
        self.u.position.z = 0.2
        self.u.psi = 0

    def cleanup(self):
        self.pub_cmd.publish(self.msg_stop)

if __name__ == "__main__":
    rospy.init_node("controller_node", anonymous=False)
    ros_rate = rospy.Rate(0.1)
    controller = controller_class(CMD_Flat())
    controller.update_u()
    while not rospy.is_shutdown():
        controller.send_control()
        ros_rate.sleep()
    #controller.cleanup()
