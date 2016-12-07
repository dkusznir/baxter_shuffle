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

    M = cv2.moments(thresh)
    cx = int(M['m10']/M['m00'])
    cy = int(M['m01']/M['m00'])
    # rospy.loginfo("width= %d, height= %d",width,height)
    # rospy.loginfo("Cx= %d, Cy= %d",cx,cy)
    # draw center
    cv2.circle(cv_image,(cx,cy),4,(0,0,255),2)

    # show process image
    cv2.imshow("Camera Image window", cv_image)

    # determine unit vector from camera to object
    cam = PinholeCameraModel()
    cam.fromCameraInfo(cam_info)
    (x, y, z) = cam.projectPixelTo3dRay((cx, cy))
    # rospy.loginfo("Printing vector... ")
    rospy.loginfo("Vector to block center:x= %f, y= %f, z= %f",x,y,z)

    newVector = Point()
    newVector.x = x
    newVector.y = y
    newVector.z = z
    # publish a vector actually
    pub_vector.publish(newVector)



    cv2.waitKey(3)

def main():
    rospy.init_node('left_hand_camera_detection', anonymous = True)

    global cam_info
    cam_info = None

    rospy.loginfo("Waiting for camera_info topic...")
    try:
        cam_info = rospy.wait_for_message('/cameras/left_hand_camera/camera_info',CameraInfo,timeout=5)
    except(rospy.ROSException), e:
        rospy.loginfo("Fail - no cam_info received")
        return False

    rospy.Subscriber("/cameras/left_hand_camera/image",Image,callback)
    cv2.namedWindow("Camera Image window")

    global pub_vector
    pub_vector = rospy.Publisher('pinHoleCameraVector', Point, queue_size=10)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
