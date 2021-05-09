#!/usr/bin/env python
import rospy
import cv2
import cv2.aruco as aruco
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Bool

class Detector:
    def __init__(self):
        self.sub = rospy.Subscriber("/ardrone/bottom/image_raw", Image, self.callback)
        self.pub = rospy.Publisher('/drone_control/image_raw', Image, queue_size=1)
        self.pub_aruco = rospy.Publisher('/drone_control/aruco_detected', Bool, queue_size=1)
        self.bridge = CvBridge()
        self.aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
        self.arucoParams = aruco.DetectorParameters_create()
        self.detected = Bool()
        self.detected.data = False

    def callback(self, image_message):
        cv_image = self.bridge.imgmsg_to_cv2(image_message, desired_encoding='passthrough')
        (corners, ids, rejected) = aruco.detectMarkers(cv_image, self.aruco_dict, parameters=self.arucoParams)
        try:
            if len(ids) > 0:
                self.detected.data = True
                aruco.drawDetectedMarkers(cv_image, corners, ids)
        except: pass
        image_message = self.bridge.cv2_to_imgmsg(cv_image, encoding="passthrough")
        self.pub.publish(image_message)
        self.pub_aruco.publish(self.detected)


if __name__ == '__main__':
    rospy.init_node('detection_node')
    t = Detector()
    while not rospy.is_shutdown():
        rospy.spin()
