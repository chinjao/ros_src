#include "ros/ros.h"
#include "std_msgs/String.h"
#include "zmq_ros/zmq_time2.h"
#include "speech_msgs/Speech.h"
#include "humans_msgs/HumanSrv.h"
#include <iostream>
#include "zmq.hpp"
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string>
#include <assert.h>
#include "msgpack.hpp"
#include <fstream>
#include <sys/stat.h>
#include <sys/types.h>
#include <map>

using namespace std;

struct speech{
  string tracking_ID;
  string speech_dictation;
  string day_time;
  int month;
  int day;
  int hour;
  int minute;
  int second;
  string place;
  MSGPACK_DEFINE(tracking_ID,speech_dictation, day_time,month,day,hour,minute,second,place);
}speech_dic;

int main(int argc, char **argv){
  ros::init(argc, argv, "zmq_pub");

  ros::NodeHandle n;
  struct stat st;
  const char* dir;
  int ret;
  ros::Publisher zmq_pub = n.advertise<speech_msgs::Speech>("zmq_publish", 1000);
  speech_msgs::Speech zmq_msg;
  
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
    msgpack::sbuffer buffer;

      string text_file,dir_place;
      
      text_file = dir_place + "/" + speech_dic.day_time + ".txt";
      dir = dir_place.c_str();
      ret = stat(dir,&st);
      cout << text_file << endl;
      if(ret != 0){
	mkdir(dir,0777);
      }	    
      /*   ofstream ofs(text_file.c_str());
      ofs << "TrackingID : " << speech_dic.tracking_ID << endl;
      ofs << "音声認識結果 : " << speech_dic.speech_dictation << endl;*/
      zmq_msg.TrackingID = std::atoll(speech_dic.tracking_ID.c_str());
      zmq_msg.time = speech_dic.day_time;
      zmq_msg.month = speech_dic.month;
      zmq_msg.day = speech_dic.day;
      zmq_msg.hour = speech_dic.hour;
      zmq_msg.minute = speech_dic.minute;
      zmq_msg.second = speech_dic.second;
      zmq_msg.sentence = speech_dic.speech_dictation;
      //zmq_msg.emotion = speech_dic.emotion;
      zmq_pub.publish(zmq_msg);
      cout << "音声認識結果:" << speech_dic.speech_dictation << endl;
      cout << "取得時間:" << speech_dic.day_time << endl;
      cout << "取得場所:" << speech_dic.place << endl;
      cout << endl;
  }
  return 0;
}
