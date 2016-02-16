#include "picojson-master/picojson.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "messanger/noun.h"
#include <fstream>
#include <iostream>
#include <cassert>
#include <memory>
#include <string>
#include <algorithm>
using namespace std;
using namespace picojson;

int main(int argc, char **argv) {
  int c;
  map<std::string,std::string> json_map;
  value root;
  int i;
  int robot = 0;
  int place;
  std_msgs::String command_msg;
  ros::init(argc, argv, "noun");
  ros::NodeHandle n;
  ros::Publisher command_pub = n.advertise<std_msgs::String>("command_noun", 1000);
  string aa;
  string bb = "pioneer";
  while(1){
    messanger::nounConstPtr noun_msg = ros::topic::waitForMessage<messanger::noun>("noun");
    //ros::Subscriber sub = n.subscribe("command_verb", 1000, chatterCallback);
    for(i = 1; i < noun_msg->value ; i++){
      //   cout << i << endl;
      aa = noun_msg->noun[i];
      //aa.replace(aa.begin(),aa.end(),'\n',' ');
      //  aa.ignore();
      //      cout << aa << endl;
      root = value();
      c = 0;
 
      ifstream stream("/home/yhirai/catkin_ws/src/noun/src/noun.json");
      //if ( !stream.is_open() ) return 1;
      stream >> root;
      //      assert( get_last_error().empty() );
    
      const picojson::value::object& obj = root.get<picojson::object>();
      for (picojson::value::object::const_iterator i = obj.begin(); i != obj.end(); ++i) {
	json_map.insert( make_pair(i->first,i->second.to_str() ) );
      }
      c = json_map.count(aa);
      //cout << c << endl;
      if(c > 0){
	object image = root.get<object>()[aa].get<object>();
	//	cout << "command=" << image["command"].get<string>() << endl;
	command_msg.data = image["command"].get<string>();
      }
      cout << aa << endl;
      if(aa == "pioneer")
	robot = 1;
      else if(aa == "roomba")
	robot = 2;
    }
    // cout << command_msg.data << endl;
    command_pub.publish(command_msg);
    //cout << command_msg.data << endl;
    std_msgs::StringConstPtr msg = ros::topic::waitForMessage<std_msgs::String>("command_verb");
    if(msg->data == "move" && robot == 1){
      cout << command_msg.data << endl;
      robot = 0;
    }
    
  }
}
