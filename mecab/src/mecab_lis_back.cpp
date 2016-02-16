#include "ros/ros.h"
#include "messanger/mecab.h"
#include "std_msgs/String.h"
#include <sstream>
#include <string>
#include <iostream>
#include <fstream>

using namespace std;


void chatterCallback(const messanger::mecab::ConstPtr& msg)
{
  int i = 1;
  ros::NodeHandle n;
  ros::Publisher verb_pub = n.advertise<std_msgs::String>("chatter", 1000);

  std_msgs::String verb_msg;
  cout << "全文:" << msg->all << endl;
  cout << "語句数:"  << msg->number << endl;
  while(msg->word[i] != ""){
    if(msg->part[i] == "名詞"){
      if(msg->part3[i] == "人名")
	cout << msg->word[i] << " " << msg->part[i] << " " << msg->part2[i] << " " << msg->part3[i] << endl;
      else
	cout << msg->word[i] << " " << msg->part[i] << " " << msg->part2[i] << " " << msg->part4[i] << endl;
    }
    else if(msg->part[i] == "動詞"){
      cout << msg->word[i] << " " << msg->part[i] << " " << msg->part2[i] << " " << msg->part3[i] << " " << msg->part4[i] << endl;
      verb_msg.data = msg->word[i];
      verb_pub.publisher(verb_msg);
    }
    else
      cout << msg->word[i] << " " << msg->part[i] << endl;
    i++;
  }
  cout << endl;
}

int main(int argc, char **argv){
  int cons = 0;
  int search = 0;
  int mod = 0;
  string line,word;
  ros::init(argc, argv, "mecab");
  ros::NodeHandle n;

  /*while(1){
    //  messanger::mecabConstPtr testdata = ros::topic::waitForMessage<messanger::mecab>("mecab_res");
    //   if(testdata->word != "BOS"){

    cout << testdata->word << "," << word << endl;
      //}

      //    usleep(200000);
      }*/
  ros::Subscriber sub = n.subscribe("mecab_res", 1000, chatterCallback);
  ros::spin();
  return 0;
}
