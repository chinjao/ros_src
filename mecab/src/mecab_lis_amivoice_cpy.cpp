#include "ros/ros.h"
#include "zmq_ros/amivoice_noun_verb2.h"
#include "zmq_ros/mecab_amivoice2.h"
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
  ros::init(argc, argv, "mecab2");
  ros::NodeHandle n;
  ros::Publisher noun_pub = n.advertise<zmq_ros::amivoice_noun_verb2>("noun2", 1000);
  zmq_ros::amivoice_noun_verb2 noun_msg;
  while(1){
    zmq_ros::mecab_amivoice2ConstPtr msg = ros::topic::waitForMessage<zmq_ros::mecab_amivoice2>("mecab_res2");
    int i = 1;
    int d = 1;
    noun_msg.name = msg->name;
    noun_msg.trackingID = msg->tracking_ID;
    cout << "全文:" << msg->all << endl;
    cout << "語句数:"  << msg->number << endl;
    while(msg->word[i] != ""){
      if(msg->part[i] == "名詞" || msg->part[i] == "感動詞"){
	if(msg->part3[i] == "人名"){
	  cout << msg->word[i] << " " << msg->part[i] << " " << msg->part2[i] << " " << msg->part3[i] << endl;
	}
	else{
	  cout << msg->word[i] << " " << msg->part[i] << " " << msg->part2[i] << endl;
	}
	if(msg->word[i] == "バイバイ" || msg->word[i] == "ばいばい" || msg->word[i] == "さようなら"){
	      noun_msg.verb = msg->word[i];
	}
	noun_msg.noun[d] = msg->word[i];
	d++;
      }
      else if(msg->part[i] == "動詞"){
	if(msg->word[i+1] == "な"){
	  no = 1;
	}
	else{
	  cout << msg->word[i] << " " << msg->part[i] << " " << msg->part2[i] << " " << msg->part3[i] << " " << msg->part4[i] << endl;
	  if(msg->part3[i] == "未然形"){
	    no = 1;
	  }
	  else{
	    if(msg->part4[i] == "する"){
	      for(b = 1; msg->word[i-b] != msg->part[0] ;b++){
		if(msg->part[i-b] == "名詞"){
		  noun_msg.verb = msg->word[i-b];
		  //verb_pub.publish(verb_msg);
		  break;
		}
	      }
	    }
	    else{
	      noun_msg.verb = msg->part4[i];
	      //verb_pub.publish(verb_msg);
	    }
	  }
	}
      }
      else
	cout << msg->word[i] << " " << msg->part[i] << endl;
      
      i++;
    }
    if(no != 1){
      noun_msg.value = d;
      noun_msg.all = msg->all;
      noun_pub.publish(noun_msg);
    }
    no = 0;
    cout << endl;
    noun_msg.verb = "";
    for(i = 0; i < d ; i++){
      noun_msg.noun[i] = "";
    }
  }
  //ros::Subscriber sub = n.subscribe("mecab_res", 1000, chatterCallback);
  //ros::spin();
  return 0;
}
