#!/usr/bin/env python
import argparse
import struct
import sys
import rospy
import cv2
import numpy as np
import baxter_interface

from cv_bridge import CvBridge, CvBridgeError
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

def controllerCallBack(vector):
    rospy.loginfo(vector)

    newPose = Pose()

    # newPose.orientation.x = -0.366894936773
    # newPose.orientation.y =  0.885980397775
    # newPose.orientation.z =  0.108155782462
    # newPose.orientation.w =  0.262162481772

def main():

    rospy.init_node('limbs_contrller', anonymous = True)
    rospy.Subscriber("/pinHoleCameraVector",Point,controllerCallBack)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
