#include "ros/ros.h"
#include "zmq_ros/amivoice_noun_verb2.h"
#include "zmq_ros/mecab_amivoice2.h"
#include "speech_msgs/Speech.h"
#include "speech_msgs/Extract.h"
#include "std_msgs/String.h"
#include <sstream>
#include <string>
#include <iostream>
#include <fstream>
#include <unistd.h>
#include <sys/stat.h>

using namespace std;

int main(int argc, char **argv){
  int cons = 0;
  int search = 0;
  int mod = 0;
  int b = 0;
  int no = 0;
  string line,word;
  ros::init(argc, argv, "mecab");
  ros::NodeHandle n;
  ros::Publisher noun_pub = n.advertise<speech_msgs::Speech>("noun", 1000);
  while(1){
    speech_msgs::SpeechConstPtr noun_msg 
      = ros::topic::waitForMessage<speech_msgs::Speech>("mecab_res");
    speech_msgs::Extract msg;
    int i = 1;
    int d = 1;
    //noun_msg->extract.clean();
    msg.com_jud = 0;
    while(noun_msg->words[i].word != ""){
      if(noun_msg->words[i].part[0] == "名詞" || noun_msg->words[i].part[0] == "感動詞"){
	if(noun_msg->words[i].part[0] == "名詞"){
	  struct stat st;
	  string textfile;
	  int ret;
	  const char* text;
	  if(noun_msg->words[i].word == "バイバイ" || noun_msg->words[i].word == "ばいばい" || noun_msg->words[i].word == "さようなら"){
	    msg.com = "bye";
	    msg.com_jud = 1;
	  }
	  msg.noun.push_back(noun_msg->words[i].word);
	  d++;
	}
	else if(noun_msg->words[i].part[0] == "動詞"){
	  if(noun_msg->words[i].word == "な"){
	    no = 1;
	  }
	  else{
	    if(noun_msg->words[i].part[2] == "未然形"){
	      no = 1;
	    }
	    else{
	      if(noun_msg->words[i].part[3] == "する"){
		for(b = 1; i-b != 0 ;b++){
		  if(noun_msg->words[i-b].part[0] == "名詞"){
		    msg.com = noun_msg->words[i-b].word;
		    msg.com_jud = 1;
		    //verb_pub.publish(verb_msg);
		    break;
		  }
		}
	      }
	      else{
		msg.verb.push_back(noun_msg->words[i].part[3]);
		//verb_pub.publish(verb_msg);
	      }
	    }
	  }
	}
	else	
	i++;
      }
      if(no != 1){
	msg.noun_num = d;
	speech_msgs::Speech sp_msg;
	sp_msg = noun_msg;
       	sp_msg.extracts = msg;
	noun_pub.publish(sp_msg);
      }
    }
    //ros::Subscriber sub = n.subscribe("mecab_res", 1000, chatterCallback);
    //ros::spin();
    return 0;
  }
}
