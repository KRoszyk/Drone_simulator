#!/usr/bin/env python
import rospy
import cv2
import cv2.aruco as aruco
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Bool, Float32MultiArray, Int8

class Detector:
    def __init__(self):
        self.sub = rospy.Subscriber("/ardrone/bottom/image_raw", Image, self.callback)
        self.pub = rospy.Publisher('/drone_control/image_raw', Image, queue_size=1)
        self.pub_aruco = rospy.Publisher('/drone_control/aruco_detected', Bool, queue_size=1)
        self.pub_center = rospy.Publisher('/drone_control/aruco_center', Float32MultiArray, queue_size=1)
        self.pub_id = rospy.Publisher('/drone_control/aruco_id', Int8, queue_size=1)
        self.bridge = CvBridge()
        self.aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
        self.arucoParams = aruco.DetectorParameters_create()
        self.detected = Bool()
        self.detected.data = False
        self.aruco_center = Float32MultiArray()
        self.aruco_id = Int8()

    def callback(self, image_message):
        cv_image = self.bridge.imgmsg_to_cv2(image_message, desired_encoding='passthrough')
        (corners, ids, rejected) = aruco.detectMarkers(cv_image, self.aruco_dict, parameters=self.arucoParams)
        try:
            if len(ids) > 0:
                self.detected.data = True
                aruco.drawDetectedMarkers(cv_image, corners, ids)
                vertexes = corners[0][0]
                center_x = (vertexes[0][1] + vertexes[1][1] + vertexes[2][1] + vertexes[3][1]) / 4
                center_y = (vertexes[0][0] + vertexes[1][0] + vertexes[2][0] + vertexes[3][0]) / 4
                center_coords = Float32MultiArray()
                center_coords.data = [center_x, center_y]
                self.aruco_center = center_coords
                self.aruco_id.data = ids[0][0]
        except: pass
        image_message = self.bridge.cv2_to_imgmsg(cv_image, encoding="passthrough")
        self.pub.publish(image_message)
        self.pub_aruco.publish(self.detected)
        self.pub_center.publish(self.aruco_center)
        self.pub_id.publish(self.aruco_id)

if __name__ == '__main__':
    rospy.init_node('detection_node')
    t = Detector()
    while not rospy.is_shutdown():
        rospy.spin()
