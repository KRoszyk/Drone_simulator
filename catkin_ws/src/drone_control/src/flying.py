#!/usr/bin/env python
import rospy
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from drone_controller import BasicDroneController as BDC
from std_msgs.msg import Bool, Float32, Float32MultiArray, Int8
from drone_status import DroneStatus
from collections import OrderedDict

class Fly:
    def __init__(self):
        self.sub_odom = rospy.Subscriber("/drone_control/odom_converted", Odometry, self.callback_odom)
        self.sub_aruco = rospy.Subscriber("/drone_control/aruco_detected", Bool, self.callback_detected)
        self.sub_aruco_center = rospy.Subscriber("/drone_control/aruco_center", Float32MultiArray, self.callback_center)
        self.sub_aruco_id = rospy.Subscriber("/drone_control/aruco_id", Int8, self.callback_id)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.pub_angle = rospy.Publisher('/drone_control/drone_angle', Float32, queue_size=1)
        self.controller = BDC()
        self.twist = Twist()
        self.detected = False
        self.drone_x = 0
        self.drone_y = 0
        self.drone_z = 0
        self.angle = 0
        self.diff_angle = 0
        self.aruco_center = (320,180)
        self.Image_center = (320,180)
        self.old_aruco_center = None
        self.detection_counter = 0
        self.dest_dict = OrderedDict()
        self.dest_dict['1'] = (3, -14)
        self.dest_dict['10'] = (-14, 2)
        self.dest_dict['20'] = (-4, 13)
        self.dest_dict['50'] = (11, 4)
        self.dest_dict['100'] = (0,0)
        self.dest_id = self.dest_dict.keys()[0]
        self.marker_coords = self.dest_dict[self.dest_id]
        self.aruco_id = None
        self.rotated = False
        self.on_target = False

    def callback_id(self, message):
        self.aruco_id = message.data

    def callback_center(self, message):
        self.aruco_center = message.data

    def rotate_2_angle_and_fly(self):
        if self.diff_angle >= 5:
            self.controller.SetCommand(yaw_velocity=0.3, pitch=0)
        elif self.diff_angle <= -5:
            self.controller.SetCommand(yaw_velocity=-0.3, pitch=0)
        else: 
            self.controller.SetCommand(pitch=1)

    def land_on_target(self):
        x_diff = self.Image_center[0] - self.aruco_center[0]
        y_diff = self.Image_center[1] - self.aruco_center[1]
        x_diff = x_diff / (self.Image_center[0])
        y_diff = y_diff / (self.Image_center[1])
        
          
        if self.detection_counter <= 2: self.controller.SetCommand(pitch = x_diff*0.3, roll = y_diff*0.3, z_velocity=-0.5)
        else: self.controller.SetCommand(pitch = 0.5*x_diff, roll = 0.5*y_diff, z_velocity=-0.5)
        
        if (abs(x_diff) <= 0.05 and abs(y_diff) <= 0.05 and self.drone_z <= 1.2) or self.drone_z <= 1: 
            self.controller.SetCommand(pitch = 0, roll = 0, z_velocity=0)
            self.controller.SendLand()
            if self.dest_dict and self.controller.status == DroneStatus.Landed:
                self.on_target = True
                self.dest_dict.popitem(last=False)
                self.dest_id = self.dest_dict.keys()[0]
                self.marker_coords = self.dest_dict[self.dest_id]
                self.detection_counter = 0
                rospy.sleep(3)
                self.on_target = False
            else: print("Finished flying!")

    def callback_odom(self, odom):  
        self.drone_x = odom.pose.pose.position.x
        self.drone_y = odom.pose.pose.position.y
        self.drone_z = odom.pose.pose.position.z
        angle = math.degrees(self.calculate_angle((self.drone_x, self.drone_y), self.marker_coords))
        self.diff_angle = angle - self.controller.rotation

        print("os z drona:", self.drone_z)
        if self.controller.status == DroneStatus.Landed and self.dest_dict and not self.on_target:
            self.controller.SendTakeoff()
        if self.controller.status == DroneStatus.Flying and self.drone_z < 10: 
            self.controller.SetCommand(z_velocity=2)
        elif self.controller.status == DroneStatus.Flying and self.drone_z >= 10: 
            self.rotate_2_angle_and_fly()

    def callback_detected(self, message):
        self.detected =  message.data
        if self.detected and self.aruco_id == int(self.dest_id):
            self.detection_counter += 1
            self.land_on_target()

    def calculate_angle(self, point1, point2):
        angle = math.atan2((point2[1]-point1[1]), (point2[0]-point1[0]))     
        if point2[0] < point1[0] and point1[1] > point2[1]:
            angle = -1 * angle
        return angle

if __name__ == '__main__':
    rospy.init_node('flying')
    t = Fly()
    while not rospy.is_shutdown():
        rospy.spin()




