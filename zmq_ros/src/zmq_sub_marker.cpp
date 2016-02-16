#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Imu.h"
#include "zmq_ros/zmq_time2.h"
#include "humans_msgs/HumanSrv.h"
#include "humans_msgs/Human.h"
#include <iostream>
#include <math.h>
#include "zmq.hpp"
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string>
#include <assert.h>
#include "msgpack.hpp"
#include "visualization_msgs/Marker.h"

using namespace std;

struct b_angle{
  float beam_angle;
  float confidence;
  int speech;
  MSGPACK_DEFINE(beam_angle,confidence,speech);
}angle;

int main(int argc, char **argv){
  ros::init(argc, argv, "zmq_angle");

  ros::NodeHandle n;
  zmq::context_t context(1);
  zmq::socket_t subscriber(context,ZMQ_SUB);
  msgpack::unpacked unpack_angle;
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);

  //  ros::Publisher publisher = n.advertise<sensor_msgs::Imu>("test_imu",100);
  subscriber.connect("tcp://133.19.23.48:5511");
  subscriber.setsockopt(ZMQ_SUBSCRIBE,"",0);
  int i = 0;
  while(1){
    visualization_msgs::Marker marker;
    marker.header.frame_id = "camera_link";
    marker.header.stamp = ros::Time::now();
    zmq::message_t update;
    subscriber.recv(&update);
    msgpack::unpack(&unpack_angle,static_cast<const char*>(update.data()),update.size());
    marker.ns = "basic_shapes";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;
    //    sensor_msgs::Imu imu;
    //imu.header.stamp = ros::Time::now();
    //imu.header.frame_id = "/camera_link";
    unpack_angle.get().convert(&angle); //ここでmsgpack受け取り
    marker.scale.x = 0.9*angle.confidence;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    
    marker.pose.orientation.x = cos(angle.beam_angle);
    marker.pose.orientation.y = sin(angle.beam_angle);
    //   cout << angle.speech << endl;
    if(angle.speech == 1){
      marker.color.r = 1.0f;
      marker.color.g = 0.0f;
      marker.color.b = 0.0f;
      marker.color.a = 1.0;

    }
    else{
      marker.color.r = 0.0f;
      marker.color.g = 1.0f;
      marker.color.b = 0.0f;
      marker.color.a = 1.0;
    }
    marker_pub.publish(marker);
    //    imu.linear_acceleration.z = sin(40*i);
    //  cout << angle.beam_angle << endl;
    //imu.linear_acceleration.x = angle.confidence*cos(angle.beam_angle);
    //imu.linear_acceleration.y = angle.confidence*sin(angle.beam_angle);
    //   imu.linear_acceleration.z = angle.beam_angle;
    /*  visualization_msgs::Marker marker;
  uint32_t shape = visualization_msgs::Marker::ARROW;
  marker.header.frame_id = "/my_frame";
  marker.id = 0;
  marker.ns = "basic_shapes";
  marker.type = shape;
  marker.action = visualization_msgs::Marker::ADD;
  
  //  marker.pose.orientation.x = 0.5;
  marker.pose.orientation.y = 0.1;
  //marker.pose.orientation.z = 0.0;
  //marker.pose.orientation.w = 0.00;
  marker.scale.x = 1.0;
  marker.scale.y = 0.2;
  marker.scale.z = 0.2;
  marker.color.r = 0.0f;
  marker.color.g = 1.0f;
  marker.color.b = 0.0f;
  marker.color.a = 1.0;
    
  marker_pub.publish(marker);
    */
    // publisher.publish(imu);

    //cout << angle.beam_angle << endl;
  }
  return 0;
}
