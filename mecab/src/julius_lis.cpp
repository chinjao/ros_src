#include "ros/ros.h"
#include "messanger/noun.h"
#include "messanger/mecab.h"
#include "std_msgs/String.h"
#include <sstream>
#include <string>
#include <iostream>
#include <fstream>
#include <unistd.h>

using namespace std;

/*
void chatterCallback(const messanger::mecab::ConstPtr& msg)
{

}
*/
int main(int argc, char **argv){
  int cons = 0;
  int search = 0;
  int mod = 0;
  int b = 0;
  int no = 0;
  string line,word;
  ros::init(argc, argv, "juli");
  ros::NodeHandle n;
  ros::Publisher jul_pub = n.advertise<std_msgs::String>("mess", 1000);
  std_msgs::String julius_mes;

  while(1){
    messanger::mecabConstPtr msg = ros::topic::waitForMessage<messanger::mecab>("mecab_res");
    cout << "全文:" << msg->all << endl;
    julius_mes.data = msg->all;
    jul_pub.publish(julius_mes);
  }
  //ros::Subscriber sub = n.subscribe("mecab_res", 1000, chatterCallback);
  //ros::spin();
  return 0;
}
