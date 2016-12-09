#!/usr/bin/env python
import argparse
import struct
import sys
import rospy
import cv2
import numpy as np
import baxter_interface

from baxter_core_msgs.msg import EndpointState
from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)

from std_msgs.msg import (
    Header,
    String,
)

from sensor_msgs.msg import (
    JointState,
)

from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)

def getGripperPose(data):
    receive_pose = data.pose
    # only want the position x,y,z
    current_position = receive_pose.position
    # rospy.loginfo("Printing position... ")
    # rospy.loginfo(current_position)

def getPoint(vector):
    receive_vector = vector
    # rospy.loginfo("Printing vector... ")
    # rospy.loginfo(receive_vector)

def main():

    rospy.init_node('limbs_contrller', anonymous = True)
    global receive_pose
    rospy.Subscriber("/convertPixeltoCoordinate",Point,getPoint)
    global receive_vector
    rospy.Subscriber("/robot/limb/left/endpoint_state",EndpointState,getGripperPose)
    global pub_point
    pub_point = rospy.Publisher('pointGeneratedFromController',Point,queue_size=10)


    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
