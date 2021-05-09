#include <ros/ros.h>
#include <sensor_msgs/NavSatStatus.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>
#include <nswr_gps/conversions.h>
#include <string>
#include <sstream>

static ros::Publisher odom_pub;
int counter = 0;

namespace patch
{
    template < typename T > std::string to_string( const T& n )
    {
        std::ostringstream stm ;
        stm << n ;
        return stm.str() ;
    }
}

void callback(const sensor_msgs::NavSatFixConstPtr &fix)
{
  double northing, easting;
  double latitude, longitude;
  std::string zone;
  nav_msgs::Odometry odom;
  sensor_msgs::NavSatStatus status = fix->status;
  if (fix->header.stamp == ros::Time(0) || fix->status.status == sensor_msgs::NavSatStatus::STATUS_NO_FIX)
  {
    return;
  }
  latitude = fix->latitude;
  longitude = fix->longitude;
  int ZoneNumber = int((longitude + 180)/6) + 1;
  zone = patch::to_string(ZoneNumber);

  gps_common::LLtoUTM(latitude, longitude, northing, easting, zone);
  odom.header.stamp = fix->header.stamp;
  odom.header.frame_id = "map";
  odom.child_frame_id = fix->header.frame_id;

  odom.pose.pose.position.x = easting;
  odom.pose.pose.position.y = northing;
  odom.pose.pose.position.z = fix->altitude;
  // easting: the x coordinate
  // northing: the y coordinate
  // fix->altitude: the z coordinate

  // Publish odometry message
  odom_pub.publish(odom);
  counter++;
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "nswr_gps");
  ros::NodeHandle node("~");
  // Odometry publishers
  odom_pub = node.advertise<nav_msgs::Odometry>("odom_gps", 10);
  // GPS Fix subscribers
  ros::Subscriber fix_sub = node.subscribe("/fix", 10, callback);

  ros::spin();
}
