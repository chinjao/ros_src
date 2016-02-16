#include "ros/ros.h"
#include "std_msgs/String.h"
#include "zmq_ros/zmq_time.h"

#include <iostream>
#include "zmq.hpp"
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string>
#include <assert.h>
#include "msgpack.hpp"

using namespace std;

int main(int argc,char **argv){
  ros::init(argc,argv,"zmq_sub");
  map<string, string> dict;
  ros::NodeHandle n;
  zmq::context_t context(1);
  zmq::socket_t receiver(context,ZMQ_PULL);
  receiver.connect("tcp://localhost:8081");
  msgpack::unpacked unpacked;
  while(1){
    zmq::message_t message;
    receiver.recv(&message);
    msgpack::unpack(&unpacked,static_cast<const char*>(message.data()),message.size());
    unpacked.get().convert(&dict);
    cout << dict["name"] << endl;
  }
}
