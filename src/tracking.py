#!/usr/bin/env python
import argparse
import struct
import sys
import rospy
import cv2
import numpy as np
import baxter_interface
from baxter_core_msgs.msg import EndpointState
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

def control_baxter_arm(limb,point,quaternion):
    rospy.loginfo("Received pose... ")
    check_version = baxter_interface.CHECK_VERSION
    baxter = baxter_interface.RobotEnable(check_version)
    init_state = baxter.state().enabled
    baxter.enable()
    limb_left = baxter_interface.Limb('left')
    limb_right = baxter_interface.Limb('right')
    # grippers
    gripper_left = baxter_interface.Gripper('left')
    gripper_left.calibrate() # must calibrate at first run
    gripper_left.open()

    # Adopted from rethinkrobotics.com/wiki/IK_Pick_and_Place_Demo code walkthrough
    # IK Service
    ns = "ExternalTools/" + limb + "/PositionKinematicsNode/IKService"
    iksvc = rospy.ServiceProxy(ns, SolvePositionIK)

    hdr = Header(stamp=rospy.Time.now(), frame_id='base')
    poses = {
        'left': PoseStamped(header=hdr,pose=Pose(position=point,orientation=quaternion)),
        'right': PoseStamped(header=hdr,pose=Pose(position=point,orientation=quaternion))
    }

    ikreq = SolvePositionIKRequest()
    ikreq.pose_stamp.append(poses[limb])
    try:
        rospy.wait_for_service(ns, 5.0)
        resp = iksvc(ikreq)
    except (rospy.ServiceException, rospy.ROSException), e:
        rospy.logerr("Service call failed: %s" % (e,))
        return False

    # Check if result valid, and type of seed ultimately used to get solution
    # convert rospy's string representation of uint8[]'s to int's

    solved = False
    while(solved == False):
        resp_seeds = struct.unpack('<%dB' % len(resp.result_type),
                                   resp.result_type)

        if (resp_seeds[0] != resp.RESULT_INVALID):
            seed_str = {
                        ikreq.SEED_USER: 'User Provided Seed',
                        ikreq.SEED_CURRENT: 'Current Joint Angles',
                        ikreq.SEED_NS_MAP: 'Nullspace Setpoints',
                       }.get(resp_seeds[0], 'None')
            rospy.loginfo("SUCCESS - Valid Joint Solution Found from Seed Type: %s" %
                  (seed_str,))
            # Format solution into Limb API-compatible dictionary
            limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
            solved = True
        else:
            rospy.loginfo("INVALID POSE - Inupt Different Seed Angles + Random Noise.")

            # Yunchu suggsted add a random noise to seed if IK has no solution
            random= np.random.normal(0,0.25,7)
            js = JointState()
            js.header = hdr

            if limb == "left":
                i = 0
                for key,val in limb_left.joint_angles().iteritems():
                    js.name.append(key)
                    js.position.append(val+random[i])
                    i += 1
            elif limb == "right":
                i = 0
                for key,val in limb_right.joint_angles().iteritems():
                    js.name.append(key)
                    js.position.append(val+random[i])
                    i += 1

            ikreq.seed_angles = [js]
            resp = iksvc(ikreq)
    global pick

    if limb == "left":
        # limb_left = baxter_interface.Limb('left')
        limb_left.move_to_joint_positions(limb_joints)
        if pick == True:
            rospy.loginfo("Gripper closing... ")
            gripper_left.close()
        else:
            gripper_left.open()
    elif limb == "right":
        # limb_right = baxter_interface.Limb('right')
        limb_right.move_to_joint_positions(limb_joints)

def getGripperPose(data):
    global receive_pose
    receive_pose = data.pose

def getPoint(point):
    global receive_pose
    # flags
    global at_home
    global stop_receiving
    global pick
    global retract
    if stop_receiving == False and at_home == True:
        rospy.loginfo("This is the point received...")
        # gripper orientation does not change
        home_q = Quaternion(x=0.999589232154,y=-0.0245830850312,z=0.00968817562479,w=0.0110985650154)

        current_position = receive_pose.position
        move_down = Point()
        move_down.x = current_position.x
        move_down.y = current_position.y
        move_down.z = -0.186592194012
        control_baxter_arm('left',point,home_q)
        stop_receiving = True

    if pick == False:
        # move down
        # only want the position x,y,z
        current_position = receive_pose.position
        move_down = Point()
        move_down.x = current_position.x
        move_down.y = current_position.y
        move_down.z = -0.186592194012
        rospy.loginfo("Move down... ")
        control_baxter_arm('left',move_down,home_q)
        pick = True

def main():

    rospy.init_node('limbs_tracking', anonymous = True)
    check_version = baxter_interface.CHECK_VERSION
    baxter = baxter_interface.RobotEnable(check_version)
    init_state = baxter.state().enabled
    # rospy.loginfo("Enabling robot... ")
    baxter.enable()

    global receive_pose
    rospy.Subscriber("/robot/limb/left/endpoint_state",EndpointState,getGripperPose)

    global stop_receiving # flag: move to block
    stop_receiving = False

    global pick # flag: move down and close gripper
    pick = False

    global retract # flag: move up and move somewhere
    retract = False

    # home position
    global at_home
    at_home = False
    if at_home == False:
        rospy.loginfo("Move to home position... ")
        home_start = Point(x=0.548196579393,y= 0.671666419104,z=0.102095131063)
        home_q = Quaternion(x=0.998980611394,y=-0.0323088935291,z=0.0193939908084,w=-0.0248545081974)
        control_baxter_arm('left',home_start,home_q)
        at_home = True

    rospy.Subscriber("/convertPixeltoCoordinate",Point,getPoint)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
