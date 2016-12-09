#!/usr/bin/env python
import rospy
import cv2
import numpy as np
import baxter_interface

from std_msgs.msg import String
from geometry_msgs.msg import Point
from cv_bridge import CvBridge, CvBridgeError
from image_geometry import PinholeCameraModel
from geometry_msgs.msg import Point
from sensor_msgs.msg import (
    Image,
    CameraInfo,
)

def callback(data):
    # conversion from image pixel to baxter coordinates
    # newPoint = Point()
    # newPoint.x = 0.615534177063
    # newPoint.y = 0.567440498728
    # newPoint.z = -0.0683162101765
    #
    # pub_point.publish(newPoint)


    bridge = CvBridge()
    try:
        cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
        print(e)

    height,width,depth = cv_image.shape
    hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

    greenLower = np.array([60, 85, 0])
    greenUpper = np.array([90, 175, 255])

    mask = cv2.inRange(hsv, greenLower, greenUpper)
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)

    ret,thresh = cv2.threshold(mask,127,255,cv2.THRESH_BINARY)
    contours,hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    # draw contours
    cv2.drawContours(cv_image,contours,-1,(255,0,0),3)
    # find center
    M = cv2.moments(thresh)
    cx = int(M['m10']/M['m00'])
    cy = int(M['m01']/M['m00'])
    rospy.loginfo("width= %d, height= %d",width,height)
    rospy.loginfo("Cx= %d, Cy= %d",cx,cy)
    # draw center
    cv2.circle(cv_image,(cx,cy),4,(0,0,255),2)
    # show process image
    cv2.imshow("Camera Image window", cv_image)

    homeCx = 340
    homeCy = 167

    xFactor = 0.0006616
    yFactor = 0.0006779
    # conversion from image pixel to baxter coordinates
    newPoint = Point()
    if cx-homeCx > 0:
        newPoint.y = 0.364276075804 - (cx-homeCx)*yFactor
    else:
        newPoint.y = 0.364276075804 + (cx-homeCx)*yFactor

    if cy-homeCy > 0:
        newPoint.x = 0.667494608765 + (cy-homeCy)*xFactor
    else:
        newPoint.x = 0.667494608765 - (cy-homeCy)*xFactor

    # newPoint.x = 0.667494608765
    # newPoint.y = 0.364276075804
    newPoint.z = -0.0045560019772
    # newPoint= Point()
    # newPoint.x = 0.795210036117
    # newPoint.y = 0.572176283352
    # newPoint.z = -0.175121752188
    # rospy.loginfo(newPoint)
    pub_point.publish(newPoint)

    cv2.waitKey(3)

def main():
    rospy.init_node('left_hand_camera_detection', anonymous = True)

    rospy.Subscriber("/cameras/left_hand_camera/image",Image,callback)
    cv2.namedWindow("Camera Image window")

    global pub_point
    pub_point = rospy.Publisher('convertPixeltoCoordinate', Point, queue_size=1)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
