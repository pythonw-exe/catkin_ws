#!/usr/bin/env python3

import rospy
import roslib
import tf

from geometry_msgs.msg import PoseStamped, Point
from simulation.msg import car_cmd
from tf import TransformBroadcaster

from tf.transformations import quaternion_from_euler

import numpy as np

def normailize_angle(angle):
    while angle > np.pi:
        angle = angle - 2.0 * np.pi

    while angle < -np.pi:
        angle = angle + 2.0 * np.pi

    return angle

class linear_bicycle_model():
    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
        #ROS
        self.tf = TransformBroadcaster()
        self.frame = "cf"
        self.sub_feeder = rospy.Subscriber("/simulation/car_cmd", car_cmd, self.process_msg)

        #parameters
        self.max_steer = np.radians(30.0)
        self.L = 2.9
        self.dt = 0.1

        #variable initialization
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v

        #control variable initialization
        self.throttle = 0
        self.delta = 0


    def process_msg(self, msg):
        self.delta = msg.delta
        self.throttle = msg.throttle
        print("[car_simulator-linear_bicycle_model] process_msg-msg:", msg)

    def update(self):
        #Linear kinematic bicyle model
        self.delta = np.clip(self.delta, -self.max_steer, self.max_steer)
        self.x = self.x + self.v*np.cos(self.yaw)*self.dt
        self.y = self.y + self.v*np.sin(self.yaw)*self.dt
        self.yaw = self.yaw + self.v/self.L*np.tan(self.delta)*self.dt
        self.yaw = normailize_angle(self.yaw)
        self.v = self.v + self.throttle*self.dt

        translation = (self.x,
                       self.y,
                       0)
        
        #Rotations should be 0
        roll = 0
        pitch = 0
        yaw = 0
        q = quaternion_from_euler(roll, pitch, yaw)
        rotation = (q[0],
                    q[1],
                    q[2],
                    q[3])
        
        print("[car_simulator-linear_bicycle_model] update-tf:", translation, rotation)
        self.tf.sendTransform(translation, rotation, rospy.Time.now(), self.frame, "world")


if __name__ == "__main__":
    rospy.init_node("car_simulator", anonymous=False)
    ros_rate = rospy.Rate(1)
    car = linear_bicycle_model()
    while not rospy.is_shutdown():
        car.update()
        ros_rate.sleep()

