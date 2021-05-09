#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from drone_controller import BasicDroneController as BDC
from std_msgs.msg import Bool

class Fly:
    def __init__(self):
        self.sub = rospy.Subscriber("/nswr_gps/odom_gps", Odometry, self.callback)
        self.sub_aruco = rospy.Subscriber("/drone_control/aruco_detected", Bool, self.callback_detected)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.controller = BDC()
        self.twist = Twist()
        self.detected = False

    def callback(self, odom):
        z = odom.pose.pose.position.z
        if not self.detected:
            self.controller.SendTakeoff()
            self.controller.SetCommand(pitch=0)
            if z < 15: self.controller.SetCommand(z_velocity=1)
            else:
                self.controller.SetCommand(z_velocity=0)
                if self.controller.rotation < 163:
                    self.controller.SetCommand(yaw_velocity=0.3)
                else:
                    self.controller.SetCommand(pitch=1)

        else:
             self.controller.SetCommand(pitch = 0.1, z_velocity=-0.3)
             if z <= 10:
                self.controller.SendLand()

    def callback_detected(self, message):
        self.detected =  message.data


if __name__ == '__main__':
    rospy.init_node('flying')
    t = Fly()
    while not rospy.is_shutdown():
        rospy.spin()




