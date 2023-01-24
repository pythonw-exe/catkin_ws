#!/usr/bin/env python3

import rospy
import roslib
import tf

from geometry_msgs.msg import PoseStamped, Point
from tf import TransformBroadcaster

from crazyflie_real.msg import CMD_Flat

class visualization_class():
    def __init__(self):
        # TF Broadcaster
        self.tf = TransformBroadcaster()
        self.frame = "cf"
        
        self.sub_feeder = rospy.Subscriber("/mocap_node/crazyflie/pose", PoseStamped, self.update_pose)
        print("[visualization_node]: init completed")

    # Broadcast a new TF with updated pose
    def update_pose(self, msg_pose):
        translation = (msg_pose.pose.position.x,
                       msg_pose.pose.position.y,
                       msg_pose.pose.position.z)
        rotation = (msg_pose.pose.orientation.x,
                    msg_pose.pose.orientation.y,
                    msg_pose.pose.orientation.z,
                    msg_pose.pose.orientation.w)
        
        self.tf.sendTransform(translation, rotation,
                     rospy.Time.now(), self.frame, "world")
        
        print("[visualization_node]: update_pose ", translation, rotation)
        
if __name__ == "__main__":
    rospy.init_node("visualization_node", anonymous=False)
    visualization = visualization_class()
    rospy.spin()
