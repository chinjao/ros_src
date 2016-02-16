#include <ros/ros.h>
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/Quaternion.h"
#include "tf/transform_broadcaster.h"

#include <signal.h>
#include <termios.h>

using namespace std;

int main(int argc, char **argv){
  float x,y,z,theta;
  ros::init(argc,argv,"initial");

  ros::NodeHandle n;
  ros::Rate loop_rate(10);
  ros::Publisher initial_pose_pub = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose",1000);

  while (ros::ok())
  {
    tf::Vector3 result;
    geometry_msgs::PoseWithCovarianceStamped initialPose;
    initialPose.header.frame_id = "/map";
    initialPose.header.stamp = ros::Time::now();// - ros::Duration(0.1);//triangle.laserm_A.stamp + ros::Duration(0.03);
    initialPose.pose.pose.position.x = 3;
    initialPose.pose.pose.position.y = 3;
    
    tf::Quaternion q = tf::createQuaternionFromYaw(theta);
    geometry_msgs::Quaternion qMsg;
    tf::quaternionTFToMsg(q, qMsg);
    initialPose.pose.pose.orientation = qMsg;
    
    initialPose.pose.covariance[6*0+0] = 0.5 * 0.5;
    initialPose.pose.covariance[6*1+1] = 0.5 * 0.5;
    initialPose.pose.covariance[6*5+5] = M_PI/12.0 * M_PI/12.0;
    //   ROS_INFO("Setting pose: %.3f %.3f %.3f [frame=%s]", result.getX(), result.getY(), theta, "/World");
  
    initial_pose_pub.publish(initialPose);
    ros::spinOnce();
    
    loop_rate.sleep();
    
  }
  return 0;

}
