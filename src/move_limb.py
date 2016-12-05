#!/usr/bin/env python
import argparse
import struct
import sys
import rospy
import baxter_interface
import numpy as np

from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)
from std_msgs.msg import Header

from sensor_msgs.msg import JointState


from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)

def control_baxter_arm(limb,point,quaternion):
    rospy.loginfo("Received pose... ")
    check_version = baxter_interface.CHECK_VERSION
    baxter = baxter_interface.RobotEnable(check_version)
    init_state = baxter.state().enabled
    rospy.loginfo("Enabling robot... ")
    baxter.enable()
    limb_left = baxter_interface.Limb('left')
    limb_right = baxter_interface.Limb('right')

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

            # Yuchun suggsted add a random noise to seed if IK has no solution
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

    if limb == "left":
        # limb_left = baxter_interface.Limb('left')
        limb_left.move_to_joint_positions(limb_joints)
    elif limb == "right":
        # limb_right = baxter_interface.Limb('right')
        limb_right.move_to_joint_positions(limb_joints)

def main():

    rospy.init_node('control_baxter_left_arm',anonymous = True)

    check_version = baxter_interface.CHECK_VERSION
    baxter = baxter_interface.RobotEnable(check_version)
    init_state = baxter.state().enabled
    rospy.loginfo("Enabling robot... ")
    baxter.enable()

    # grippers
    # gripper_left = baxter_interface.Gripper('left')
    # gripper_right = baxter_interface.Gripper('right')
    # gripper_left.calibrate() # must calibrate at first run
    # gripper_right.calibrate()
    # rospy.loginfo("Gripper calibrated... ")
    # gripper_left.open()
    # gripper_right.open()

    # left
    test_left_p = Point(x=0.657579481614,y=0.851981417433,z=0.0388352386502)
    test_left_q = Quaternion(x=-0.366894936773,y=0.885980397775,z=0.108155782462,w=0.262162481772)

    # right
    test_right_p = Point(x=0.656982770038,y=-0.852598021641,z=0.0388609422173)
    test_right_q = Quaternion(x=-0.367048116303,y=0.885911751787,z=-0.108908281936,w=0.261868353356)

    control_baxter_arm('left',test_left_p,test_left_q)
    control_baxter_arm('right',test_right_p,test_right_q)

    rospy.spin()

if __name__ == '__main__':
    main()
