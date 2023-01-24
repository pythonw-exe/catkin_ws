#!/usr/bin/env python3

#System
import logging
import sys
import time
from threading import Event
from math import radians, degrees, sqrt, sin, cos, tan

#Crazyflie
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.positioning.position_hl_commander import PositionHlCommander
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie

from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncLogger import SyncLogger

from cflib.positioning.motion_commander import MotionCommander

from cflib.utils import uri_helper

#ROS
import rospy
from geometry_msgs.msg import PoseStamped, Point

#ARClab
from crazyflie_real.msg import CMD_Flat

# URI to the Crazyflie to connect to
uri = "radio://0/80/2M/E7E7E7E7E7"

logging.basicConfig(level=logging.ERROR)


class crazyflie_class():
    def __init__(self, link_uri):
        #crazyflie
        self.cf = Crazyflie(rw_cache = None)
        print("[debug] here1")
        self.cf.disconnected.add_callback(self._disconnected)
        self.cf.connection_failed.add_callback(self._connection_failed)

        #pub and sub objects
        self.sub_feeder = rospy.Subscriber("/mocap_node/crazyflie/pose", PoseStamped, self._send_mocap_pose)
        self.sub_cmd_flat = rospy.Subscriber('/controller/cmd_flat',CMD_Flat, self._send_cmd_flat)

    def init2(self):
        print("[debug] here2")
        self.pc = PositionHlCommander(self.cf)

    def _connection_failed(self, link_uri, msg):
        print('Connection to %s failed: %s' % (link_uri,msg))
        self.stop()
        rospy.signal_shutdown("Disconnected from Crazyflie, killing rospy...")

    def _connection_lost(self, link_uri, msg):
        print('Connection to %s lost: %s' % (link_uri,msg))
        self.stop()
        rospy.signal_shutdown("Disconnected from Crazyflie, killing rospy...")

    def _disconnected(self, link_uri):
        rospy.signal_shutdown("Disconnected from Crazyflie, killing rospy...")
        print('Disconnected from %s' % link_uri)

    def _send_mocap_pos(self, msg):
        x = msg.pose.position.x
        y = msg.pose.position.y
        z = msg.pose.position.z
        self.cf.extpos.send_extpos(x, y, z)

    def _send_mocap_pose(self, msg):
        x = msg.pose.position.x
        y = msg.pose.position.y
        z = msg.pose.position.z
        qx = msg.pose.orientation.x
        qy = msg.pose.orientation.y
        qz = msg.pose.orientation.z
        qw = msg.pose.orientation.w
        #print("[crazyflie_node]: extpose ", x, y, z, qx, qy, qz, qw)
        self.cf.extpos.send_extpose(x, y, z, qx, qy, qz, qw)

        (temp_x, temp_y, temp_z) = self.pc.get_position()
        #print("[crazyflie_node]: get_position ", temp_x, temp_y, temp_z)

    def _send_cmd_flat(self, msg):
        x = msg.position.x
        y = msg.position.y
        z = msg.position.z
        yaw = degrees(msg.psi)
        print("[crazyflie_node]: setpoint ", x, y, z, yaw)
        self.cf.commander.send_position_setpoint(x,y,z,yaw)

    #connect crazyflie with given link_uri
    def connect(self, link_uri):
        self.cf.open_link(link_uri)

    #send stop setpoint and disconnect crazyflie
    def stop(self):
        self.sub_cmd_flat.unregister()
        self.cf.commander.send_stop_setpoint()
        rospy.set_param("crazyflie_ready", False)
        self.cf.close_link()

if __name__ =="__main__":
    #Python ROS initialization
    rospy.init_node("crazyflie_node",anonymous=False)
    rospy.set_param("crazyflie_ready", False)
    ros_rate = rospy.Rate(10)

    #Crazyflie initialization
    cflib.crtp.init_drivers()

    crazyflie = crazyflie_class(uri)

    if not rospy.is_shutdown():
        print("connecting...")
        crazyflie.connect(uri)
        crazyflie.init2()

    while not rospy.get_param("crazyflie_ready") and not rospy.is_shutdown():
        ros_rate.sleep()
    
    #spinning for callback function
    try:
        rospy.spin()
    except KeyboardInterrupt:
        crazyflie.stop()

    #stop crazyflie
    if not rospy.is_shutdown():
        crazyflie.stop()

    if not rospy.is_shutdown():
        rospy.signal_shutdown("Flight finished")
