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
  map<std::string,std::string> json_map,json_map2;
  value root,root2;
  int i;
  std_msgs::String command_msg;
  ros::init(argc, argv, "noun");
  ros::NodeHandle n;
  ros::Publisher command_pub = n.advertise<std_msgs::String>("chatter", 1000);
  string aa;
  string bb = "なし";
  int robot = 0;
  int ok = 0;
  string com;
  string aa2;
  while(1){
    messanger::nounConstPtr noun_msg = ros::topic::waitForMessage<messanger::noun>("noun");
    com = "";
    aa2 = "";
    aa2 = noun_msg->verb;
    root2 = value();
    c = 0;
    ifstream stream2("/home/yhirai/catkin_ws/src/verb/src/verb.json");
    stream2 >> root2;

    const picojson::value::object& obj2 = root2.get<picojson::object>();
    for (picojson::value::object::const_iterator i2 = obj2.begin(); i2 != obj2.end(); ++i2) {
      json_map2.insert( make_pair(i2->first,i2->second.to_str() ) );
    } 
    c = json_map2.count(aa2);
    if(c > 0){
      object image2 = root2.get<object>()[aa2].get<object>();
      // cout << "command=" << image2["command"].get<string>() << endl;
      com = image2["command"].get<string>();
    }
    
    for(i = 1; i < noun_msg->value ; i++){
      aa = "";
      aa = noun_msg->noun[i];

      root = value();
      c = 0;
 
      ifstream stream("/home/yhirai/catkin_ws/src/noun/src/noun.json");
       stream >> root;
     
      const picojson::value::object& obj = root.get<picojson::object>();
      for (picojson::value::object::const_iterator i = obj.begin(); i != obj.end(); ++i) {
	json_map.insert( make_pair(i->first,i->second.to_str() ) );
      }
      c = json_map.count(aa);
      // cout << c << endl;
      if(c > 0){
	object image = root.get<object>()[aa].get<object>();
	//	cout << "command=" << image["command"].get<string>() << endl;
	command_msg.data = image["command"].get<string>();
      }
      if(aa == "pioneer")
	robot = 1;
      else if(aa == "roomba")
	robot = 2;
    }
    // cout << com << endl;
    if(com == "move"){
      while(1){
	/*if(command_msg.data != ""){
	if(robot == 1){
	  cout << command_msg.data << endl;
	  command_pub.publish(command_msg);
	}
	else if(robot == 0){
	  cout << "Which robot?" << endl;
	  messanger::nounConstPtr noun_msg = ros::topic::waitForMessage<messanger::noun>("noun");
	  for(i = 1; i < noun_msg->value ; i++ ){
	    if(noun_msg->noun[i] == "pioneer"){
	      robot = 1;
	      cout << command_msg.data << endl;
	      command_pub.publish(command_msg);
	    }
	  }
	  if(robot == 0){
	    cout << "Sorry... I don't know this robot..." << endl;
	  }
	}
      }  //ここまで場所が入ってる場合
      else if{
	
      } //場所が入っていない場合
	*/
	if(robot == 0){
	  cout << "Which robot?" << endl;
	  system("aplay /home/yhirai/dictation-kit-v4.3.1-linux/know_robot2.wav");
	  messanger::nounConstPtr noun_msg = ros::topic::waitForMessage<messanger::noun>("noun");
	  for(i = 1; i < noun_msg->value ; i++ ){
	    if(noun_msg->noun[i] == "pioneer"){
	      robot = 1;
	    }
	  }
	  if(robot == 0){
	    cout << "Sorry... I don't know this robot..." << endl;
	    system("aplay /home/yhirai/dictation-kit-v4.3.1-linux/know_robot.wav");
	    continue;  //登録されていないロボットの場合ループ
	  }
	}  //ここまで場所が入ってる場合
	if(command_msg.data == ""){
	  cout << "Where place?" << endl;
	  system("aplay /home/yhirai/dictation-kit-v4.3.1-linux/know_place2.wav");
	  messanger::nounConstPtr noun_msg = ros::topic::waitForMessage<messanger::noun>("noun");
	  for(i = 0;i < noun_msg->value ; i++){
	    c = json_map.count(noun_msg->noun[i]);
	    if(c > 0){
	      object image3 = root.get<object>()[noun_msg->noun[i]].get<object>();
	      //	cout << "command=" << image["command"].get<string>() << endl;
	      command_msg.data = image3["command"].get<string>();
	    }
	  }
	  if(command_msg.data == ""){
	    cout << "Sorry... Not register..." << endl;
	    system("aplay /home/yhirai/dictation-kit-v4.3.1-linux/know_place.wav");
	    continue; //登録されていない場所の場合ループ
	  }	  
	} //場所が入っていない場合
	if(command_msg.data != "" && robot == 1){
	  system("aplay /home/yhirai/dictation-kit-v4.3.1-linux/go.wav");
	  cout << command_msg.data << endl;
	  command_pub.publish(command_msg);
	  robot = 0;
	  break;
	}//全部入ったとき
      }//全部はいるまでループ
    }
    else if(com == "visual" && robot == 1 && command_msg.data != ""){
      cout << command_msg.data << endl;
    }
    robot = 0;
    command_msg.data = "";
    com = "";
  }
}
