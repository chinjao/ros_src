#include <fstream>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <iostream>
#include <string>

using namespace std;
//const std_msgs::String::ConstPtr msg;
/*
void chatterCallback(&msg)
{
  //ROS_INFO("I heard: [%s]", msg->data.c_str());
}
*/
int main(int argc, char **argv){
  
  while(1){
    ros::init(argc,argv,"command");
    ros::NodeHandle n;
    ros::Publisher final_pub = n.advertise<std_msgs::String>("chatter",1000);
    std_msgs::String final;
    
    std_msgs::StringConstPtr msg_verb = ros::topic::waitForMessage<std_msgs::String>("command_verb");
    //  ros::Subscriber sub = n.subscribe("command_verb", 1000, chatterCallback);
    if(msg_verb->data == "move"){
      std_msgs::StringConstPtr msg_noun = ros::topic::waitForMessage<std_msgs::String>("command_noun");
      final.data = msg_noun->data;
      final_pub.publish(final);
      cout << msg_noun->data << endl;
    }
  }
}
