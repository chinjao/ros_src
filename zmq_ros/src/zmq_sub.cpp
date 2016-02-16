#include "ros/ros.h"
#include "std_msgs/String.h"
#include "zmq_ros/zmq_time2.h"
#include "humans_msgs/HumanSrv.h"
#include "humans_msgs/Human.h"
#include <iostream>
#include "zmq.hpp"
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string>
#include <assert.h>
#include "msgpack.hpp"

using namespace std;

struct speech{
  string tracking_ID;
  string speech_dictation;
  string day_time;
  MSGPACK_DEFINE(tracking_ID,speech_dictation,day_time);
}speech_dic;

int main(int argc, char **argv){
  ros::init(argc, argv, "zmq_pub");

  ros::NodeHandle n;

  ros::Publisher zmq_pub = n.advertise<zmq_ros::zmq_time2>("zmq_publish", 1000);
  zmq_ros::zmq_time2 zmq_msg;
  
  zmq::context_t context(1);
  zmq::socket_t subscriber(context,ZMQ_SUB);
  msgpack::unpacked unpack_speech;
   subscriber.connect("tcp://133.19.23.205:4413");
     subscriber.setsockopt(ZMQ_SUBSCRIBE,"",0);
  while(1){
    zmq::message_t update;
    subscriber.recv(&update);
    msgpack::unpack(&unpack_speech,static_cast<const char*>(update.data()),update.size());
    unpack_speech.get().convert(&speech_dic); //ここでmsgpack受け取り
    ros::ServiceClient client = n.serviceClient<humans_msgs::HumanSrv>("track_srv");
    humans_msgs::HumanSrv trackingID;
    long long tracking = std::atoll(speech_dic.tracking_ID.c_str());
    trackingID.request.src.body.tracking_id = tracking;
    cout << "tracking_id: " << tracking << endl;
    trackingID.request.src.header.frame_id = "map";  
    
    /*  if(client.call(trackingID))
	{*/
    /*cout << trackingID.response.dst.face.persons[0] << endl;
	cout << trackingID.response.dst.p << endl;
    */
	zmq_msg.trackingID = speech_dic.tracking_ID;
	zmq_msg.day_time = speech_dic.day_time;
	zmq_msg.speech_dic = speech_dic.speech_dictation;
	zmq_pub.publish(zmq_msg);
	cout << speech_dic.tracking_ID << "," << speech_dic.speech_dictation << "," << speech_dic.day_time << endl;
	//}
  }
  return 0;
}
