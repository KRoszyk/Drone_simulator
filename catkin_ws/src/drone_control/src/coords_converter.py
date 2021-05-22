#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from drone_controller import BasicDroneController as BDC
from std_msgs.msg import Bool

class Converter:
    def __init__(self):
        self.sub_odom = rospy.Subscriber("/nswr_gps/odom_gps", Odometry, self.callback)
        self.pub = rospy.Publisher('/drone_control/odom_converted', Odometry, queue_size=1)
        self.drone_x = 0
        self.drone_y = 0
        self.drone_z = 0
        self.offset_x = 492822.082084
        self.offset_y = 5527525.61889
        self.offset_z = 8.75770364218

    def callback(self, odom):
        odom_z = odom.pose.pose.position.z
        odom_y = odom.pose.pose.position.y
        odom_x = odom.pose.pose.position.x
         
        self.drone_x = odom_y - self.offset_y
        self.drone_y = -1 * (odom_x - self.offset_x)
        self.drone_z = odom_z - self.offset_z
        
        message = Odometry()
        message.pose.pose.position.x = self.drone_x
        message.pose.pose.position.y = self.drone_y
        message.pose.pose.position.z = self.drone_z
        self.pub.publish(message)

if __name__ == '__main__':
    rospy.init_node('odom_converter')
    t = Converter()
    while not rospy.is_shutdown():
        rospy.spin()




