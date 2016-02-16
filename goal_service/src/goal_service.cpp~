#include "ros/ros.h"
#include "std_msgs/String.h"
#include <iostream>
#include "humans_msgs/Int32.h"

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string>
#include <assert.h>

using namespace std;

map<string, int> stringToInt;




int main(int argc,char **argv){
  ros::init(argc,argv,"goal_service");
  ros::NodeHandle n;

  stringToInt["Uema"] = 1;
  stringToInt["Morioka"] = 2;
  stringToInt["Nogawa"] = 3;
  stringToInt["Ikegami"] = 4;
  stringToInt["Adachi"] = 5;
  stringToInt["Hirai"] = 6;
  stringToInt["Teranishi"] = 7;
  stringToInt["Kawakita"] = 8;
  stringToInt["Fukuhara"] = 9;
  stringToInt["Nishida"] = 10;
  stringToInt["Kawamoto"] = 11;
  stringToInt["yatsuzuka"] = 12;
  stringToInt["Shimada"] = 13;
  
  while(1){
     std_msgs::StringConstPtr command_msg = ros::topic::waitForMessage<std_msgs::String>("chatter");
     ros::ServiceClient client = n.serviceClient<humans_msgs::Int32>("goal_okao_id");
     humans_msgs::Int32 okaoID;
     okaoID.request.n =  stringToInt[command_msg->data];//atoi(command_msg->data.c_str());
     if(client.call(okaoID))
       cout << "ok" << endl;
     else
       cout << "no" << endl;
  }
}
