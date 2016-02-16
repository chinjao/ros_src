#include "ros/ros.h"
#include "std_msgs/String.h"
#include <iostream>
#include "zmq.hpp"
#include <unistd.h>
#include <string>
#include <assert.h>
#include "msgpack.hpp"
#include <fstream>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/time.h>
#include <map>
#include <pthread.h>
#include "speech_msgs/Speech.h"
#include "speech_msgs/Emotion.h"
#include "speech_msgs/Bayesian.h"
#include <time.h>
#include <stdio.h>


using namespace std;


float pos,neg,nor;
string name;
string current_obserb;

void emo_and_nameCallback(const speech_msgs::Emotion::ConstPtr& emo_msg){
  name = emo_msg->name;
  current_obserb = emo_msg->emotion;
  pos = emo_msg->positive;
  neg = emo_msg->negative;
  nor = emo_msg->normal;
  
}

class bayesian
{
private:
  map< string,map< string,float > > emotion_trans;
  map< string,map< string,float > > current_emotion_value,emo_to_obserb,obserb_to_emo;

  int multiply_now;
 
  
  string current_emotion;


  int chrono;
  map< int,float > chrono_emotion;
  map< string ,time_t> name_timeline;
  string before_emotion;
  string before_obserb;
  
public:
  void initialize(){
    emotion_trans["happy"]["happy"] = 0.421;
    emotion_trans["happy"]["quiet"] = 0.362;
    emotion_trans["happy"]["sad"] = 0.061;
    emotion_trans["happy"]["surprise"] = 0.06;
    emotion_trans["happy"]["angry"] = 0.027;
    emotion_trans["happy"]["fear"] = 0.034;
    emotion_trans["happy"]["disgust"] = 0.032;

    emotion_trans["quiet"]["happy"] = 0.213;
    emotion_trans["quiet"]["quiet"] = 0.509;
    emotion_trans["quiet"]["sad"] = 0.09;
    emotion_trans["quiet"]["surprise"] = 0.055;
    emotion_trans["quiet"]["angry"] = 0.039;
    emotion_trans["quiet"]["fear"] = 0.051;
    emotion_trans["quiet"]["disgust"] = 0.042;

    emotion_trans["sad"]["happy"] = 0.084;
    emotion_trans["sad"]["quiet"] = 0.296;
    emotion_trans["sad"]["sad"] = 0.32;
    emotion_trans["sad"]["surprise"] = 0.058;
    emotion_trans["sad"]["angry"] = 0.108;
    emotion_trans["sad"]["fear"] = 0.064;
    emotion_trans["sad"]["disgust"] = 0.068;

    emotion_trans["surprise"]["happy"] = 0.19;
    emotion_trans["surprise"]["quiet"] = 0.264;
    emotion_trans["surprise"]["sad"] = 0.091;
    emotion_trans["surprise"]["surprise"] = 0.243;
    emotion_trans["surprise"]["angry"] = 0.086;
    emotion_trans["surprise"]["fear"] = 0.076;
    emotion_trans["surprise"]["disgust"] = 0.048;

    emotion_trans["angry"]["happy"] = 0.056;
    emotion_trans["angry"]["quiet"] = 0.262;
    emotion_trans["angry"]["sad"] = 0.123;
    emotion_trans["angry"]["surprise"] = 0.075;
    emotion_trans["angry"]["angry"] = 0.293;
    emotion_trans["angry"]["fear"] = 0.069;
    emotion_trans["angry"]["disgust"] = 0.121;

    emotion_trans["fear"]["happy"] = 0.05;
    emotion_trans["fear"]["quiet"] = 0.244;
    emotion_trans["fear"]["sad"] = 0.137;
    emotion_trans["fear"]["surprise"] = 0.101;
    emotion_trans["fear"]["angry"] = 0.096;
    emotion_trans["fear"]["fear"] = 0.279;
    emotion_trans["fear"]["disgust"] = 0.092;

    emotion_trans["disgust"]["happy"] = 0.047;
    emotion_trans["disgust"]["quiet"] = 0.252;
    emotion_trans["disgust"]["sad"] = 0.092;
    emotion_trans["disgust"]["surprise"] = 0.056;
    emotion_trans["disgust"]["angry"] = 0.164;
    emotion_trans["disgust"]["fear"] = 0.075;
    emotion_trans["disgust"]["disgust"] = 0.313;


    emo_to_obserb["happy"]["positive"] = 0.905;
    emo_to_obserb["quiet"]["positive"] = 0.143;
    emo_to_obserb["sad"]["positive"] = 0.262;
    emo_to_obserb["surprise"]["positive"] = 0.613;
    emo_to_obserb["fear"]["positive"] = 0.425;
    emo_to_obserb["disgust"]["positive"] = 0.388;
    emo_to_obserb["angry"]["positive"] = 0.600;

    emo_to_obserb["happy"]["negative"] = 0.071;
    emo_to_obserb["quiet"]["negative"] = 0.179;
    emo_to_obserb["sad"]["negative"] = 0.714;
    emo_to_obserb["surprise"]["negative"] = 0.213;
    emo_to_obserb["fear"]["negative"] = 0.475;
    emo_to_obserb["disgust"]["negative"] = 0.513;
    emo_to_obserb["angry"]["negative"] = 0.288;

    emo_to_obserb["happy"]["normal"] = 0.024;
    emo_to_obserb["quiet"]["normal"] = 0.679;
    emo_to_obserb["sad"]["normal"] = 0.024;
    emo_to_obserb["surprise"]["normal"] = 0.175;
    emo_to_obserb["fear"]["normal"] = 0.100;
    emo_to_obserb["disgust"]["normal"] = 0.100;
    emo_to_obserb["angry"]["normal"] = 0.113;

    current_emotion = "quiet";    
  }

 
  void run(){
    ros::NodeHandle nh;
    ros::Publisher bayes_pub = nh.advertise<speech_msgs::Bayesian>("bayes_result",1000);
    speech_msgs::Bayesian bayes_msg;
    ofstream ofs("/home/yhirai/data/result.txt",ios::trunc);

    multiply_now = 0;

    time_t start_time;
    int timer = 0;


    while(1){
      //      cout << "bayesian now2" << endl;
   
      speech_msgs::EmotionConstPtr emo_msg = ros::topic::waitForMessage<speech_msgs::Emotion>("emo_and_name");



    
      // ros::Subscriber sub = nh.subscribe("emo_and_name", 1000, emo_and_nameCallback);

      //ros::spin();
      cout << "bayesian now" << endl;
            pos = emo_msg->positive;
      neg = emo_msg->negative;
      nor = emo_msg->normal;

      name = emo_msg->name;
      current_obserb = emo_msg->emotion;
      if(current_emotion_value.count(name) == 0)
	initialize_emo(name);
      
      if(pos+neg+nor != 1){
	if(pos != 0)
	  pos = pos/(pos+neg+nor);
	if(neg != 0)
	  neg = neg/(pos+neg+nor);
	if(nor != 0)
	  nor = nor/(pos+neg+nor);
      }
      timer = difftime(time(NULL),name_timeline[name]);

      /* cout << timer << endl;
      cout << name_timeline[name] << endl;
      cout << time(NULL) << endl;
      */
      timeline(timer);

      multiply();
      string now = now_emo();
      cout << name << "'s Current emotion:" << now << "  name:" << name <<  endl;     name_timeline[name] = time(NULL);
      cout << "happy:" << current_emotion_value[name]["happy"] << endl;
      cout << "quiet:" << current_emotion_value[name]["quiet"] << endl;
      cout << "sad:" << current_emotion_value[name]["sad"] << endl;
      cout << "surprise:" << current_emotion_value[name]["surprise"] << endl;
      cout << "angry:" << current_emotion_value[name]["angry"] << endl;
      cout << "fear:" << current_emotion_value[name]["fear"] << endl;
      cout << "disgust:" << current_emotion_value[name]["disgust"] << endl;
      cout << endl;

      bayes_msg.happy = current_emotion_value[name]["happy"];
      bayes_msg.quiet = current_emotion_value[name]["quiet"];
      bayes_msg.sad = current_emotion_value[name]["sad"];
      bayes_msg.surprise = current_emotion_value[name]["surprise"];
      bayes_msg.angry = current_emotion_value[name]["angry"];
      bayes_msg.fear = current_emotion_value[name]["fear"];
      bayes_msg.disgust = current_emotion_value[name]["disgust"];

      bayes_msg.emotion = now;
      bayes_msg.name = name;
      usleep(400000);
      cout << "sleep escape" << endl;
      bayes_pub.publish(bayes_msg);
      ofs << "Name:" << emo_msg->name << endl;
      ofs << "happy:" << current_emotion_value[name]["happy"] << " " << "quiet:" << current_emotion_value[name]["quiet"] << " " << "sad:" << current_emotion_value[name]["sad"] << " " << "angry:" << current_emotion_value[name]["angry"] << " " << "surprise:" << current_emotion_value[name]["surprise"] << " " << "fear:" << current_emotion_value[name]["fear"] << " " << "disgust:" << current_emotion_value[name]["disgust"] << endl;
      ofs << endl;
 
    }
  }

private:
  void multiply(){
    string emo = now_emo();
    float happy,quiet,sad,surprise,angry,fear,disgust;

    happy = emo_to_obserb["happy"][current_obserb] * emotion_trans["happy"]["happy"] * current_emotion_value[name]["happy"] + emo_to_obserb["happy"][current_obserb] * emotion_trans["quiet"]["happy"] * current_emotion_value[name]["quiet"] + emo_to_obserb["happy"][current_obserb] * emotion_trans["sad"]["happy"] * current_emotion_value[name]["sad"] + emo_to_obserb["happy"][current_obserb] * emotion_trans["surprise"]["happy"] * current_emotion_value[name]["surprise"] + emo_to_obserb["happy"][current_obserb] * emotion_trans["angry"]["happy"] * current_emotion_value[name]["angry"] + emo_to_obserb["happy"][current_obserb] * emotion_trans["fear"]["happy"] * current_emotion_value[name]["fear"] + emo_to_obserb["happy"][current_obserb] * emotion_trans["disgust"]["happy"] * current_emotion_value[name]["disgust"];

    quiet = emo_to_obserb["quiet"][current_obserb] * emotion_trans["happy"]["quiet"] * current_emotion_value[name]["happy"] + emo_to_obserb["quiet"][current_obserb] * emotion_trans["quiet"]["quiet"] * current_emotion_value[name]["quiet"] + emo_to_obserb["quiet"][current_obserb] * emotion_trans["sad"]["quiet"] * current_emotion_value[name]["sad"] + emo_to_obserb["quiet"][current_obserb] * emotion_trans["surprise"]["quiet"] * current_emotion_value[name]["surprise"] + emo_to_obserb["quiet"][current_obserb] * emotion_trans["angry"]["quiet"] * current_emotion_value[name]["angry"] + emo_to_obserb["quiet"][current_obserb] * emotion_trans["fear"]["quiet"] * current_emotion_value[name]["fear"] + emo_to_obserb["quiet"][current_obserb] * emotion_trans["quiet"]["happy"] * current_emotion_value[name]["disgust"];

    sad = emo_to_obserb["sad"][current_obserb] * emotion_trans["happy"]["sad"] * current_emotion_value[name]["happy"] + emo_to_obserb["sad"][current_obserb] * emotion_trans["quiet"]["sad"] * current_emotion_value[name]["quiet"] + emo_to_obserb["sad"][current_obserb] * emotion_trans["sad"]["sad"] * current_emotion_value[name]["sad"] + emo_to_obserb["sad"][current_obserb] * emotion_trans["surprise"]["sad"] * current_emotion_value[name]["surprise"] + emo_to_obserb["sad"][current_obserb] * emotion_trans["angry"]["sad"] * current_emotion_value[name]["angry"] + emo_to_obserb["sad"][current_obserb] * emotion_trans["fear"]["sad"] * current_emotion_value[name]["fear"] + emo_to_obserb["sad"][current_obserb] * emotion_trans["disgust"]["sad"] * current_emotion_value[name]["disgust"];


    surprise = emo_to_obserb["surprise"][current_obserb] * emotion_trans["happy"]["surprise"] * current_emotion_value[name]["happy"] + emo_to_obserb["surprise"][current_obserb] * emotion_trans["quiet"]["surprise"] * current_emotion_value[name]["quiet"] + emo_to_obserb["surprise"][current_obserb] * emotion_trans["sad"]["surprise"] * current_emotion_value[name]["sad"] + emo_to_obserb["surprise"][current_obserb] * emotion_trans["surprise"]["surprise"] * current_emotion_value[name]["surprise"] + emo_to_obserb["surprise"][current_obserb] * emotion_trans["angry"]["surprise"] * current_emotion_value[name]["angry"] + emo_to_obserb["surprise"][current_obserb] * emotion_trans["fear"]["surprise"] * current_emotion_value[name]["fear"] + emo_to_obserb["surprise"][current_obserb] * emotion_trans["disgust"]["surprise"] * current_emotion_value[name]["disgust"];


    angry = emo_to_obserb["angry"][current_obserb] * emotion_trans["happy"]["angry"] * current_emotion_value[name]["happy"] + emo_to_obserb["angry"][current_obserb] * emotion_trans["quiet"]["angry"] * current_emotion_value[name]["quiet"] + emo_to_obserb["angry"][current_obserb] * emotion_trans["sad"]["angry"] * current_emotion_value[name]["sad"] + emo_to_obserb["angry"][current_obserb] * emotion_trans["surprise"]["angry"] * current_emotion_value[name]["surprise"] + emo_to_obserb["angry"][current_obserb] * emotion_trans["angry"]["angry"] * current_emotion_value[name]["angry"] + emo_to_obserb["angry"][current_obserb] * emotion_trans["fear"]["angry"] * current_emotion_value[name]["fear"] + emo_to_obserb["angry"][current_obserb] * emotion_trans["disgust"]["angry"] * current_emotion_value[name]["disgust"];


    fear = emo_to_obserb["fear"][current_obserb] * emotion_trans["happy"]["fear"] * current_emotion_value[name]["happy"] + emo_to_obserb["fear"][current_obserb] * emotion_trans["quiet"]["fear"] * current_emotion_value[name]["quiet"] + emo_to_obserb["fear"][current_obserb] * emotion_trans["sad"]["fear"] * current_emotion_value[name]["sad"] + emo_to_obserb["fear"][current_obserb] * emotion_trans["surprise"]["fear"] * current_emotion_value[name]["surprise"] + emo_to_obserb["fear"][current_obserb] * emotion_trans["angry"]["fear"] * current_emotion_value[name]["angry"] + emo_to_obserb["fear"][current_obserb] * emotion_trans["fear"]["fear"] * current_emotion_value[name]["fear"] + emo_to_obserb["fear"][current_obserb] * emotion_trans["disgust"]["fear"] * current_emotion_value[name]["disgust"];


    disgust = emo_to_obserb["disgust"][current_obserb] * emotion_trans["happy"]["disgust"] * current_emotion_value[name]["happy"] + emo_to_obserb["disgust"][current_obserb] * emotion_trans["quiet"]["disgust"] * current_emotion_value[name]["quiet"] + emo_to_obserb["disgust"][current_obserb] * emotion_trans["sad"]["disgust"] * current_emotion_value[name]["sad"] + emo_to_obserb["disgust"][current_obserb] * emotion_trans["surprise"]["disgust"] * current_emotion_value[name]["surprise"] + emo_to_obserb["disgust"][current_obserb] * emotion_trans["angry"]["disgust"] * current_emotion_value[name]["angry"] + emo_to_obserb["disgust"][current_obserb] * emotion_trans["fear"]["disgust"] * current_emotion_value[name]["fear"] + emo_to_obserb["disgust"][current_obserb] * emotion_trans["disgust"]["disgust"] * current_emotion_value[name]["disgust"];


    current_emotion_value[name]["happy"] = happy/(happy+quiet+sad+surprise+angry+fear+disgust);
    current_emotion_value[name]["quiet"] = quiet/(happy+quiet+sad+surprise+angry+fear+disgust);
    current_emotion_value[name]["sad"] = sad/(happy+quiet+sad+surprise+angry+fear+disgust);
    current_emotion_value[name]["surprise"] = surprise/(happy+quiet+sad+surprise+angry+fear+disgust);
    current_emotion_value[name]["angry"] = angry/(happy+quiet+sad+surprise+angry+fear+disgust);
    current_emotion_value[name]["fear"] = fear/(happy+quiet+sad+surprise+angry+fear+disgust);
    current_emotion_value[name]["disgust"] = disgust/(happy+quiet+sad+surprise+angry+fear+disgust);
    

    
  }
  
  string now_emo(){
    string emo;
    float max = 0;
    for(map<string,float>::iterator it = current_emotion_value[name].begin();
	it != current_emotion_value[name].end(); it++)
      if (max < it->second)
	{
	  max = it->second;
	  emo = it->first;
	}
    return emo;
  }
  
  void initialize_emo(string name){
    current_emotion_value[name]["happy"] = 0.201;
    current_emotion_value[name]["quiet"] = 0.378;
    current_emotion_value[name]["sad"] = 0.118;
    current_emotion_value[name]["surprise"] = 0.076;
    current_emotion_value[name]["angry"] = 0.083;
    current_emotion_value[name]["fear"] = 0.070;
    current_emotion_value[name]["disgust"] = 0.073;
    name_timeline[name] = time(NULL);
  }

  void timeline(int timer){

    float happy,quiet,sad,surprise,angry,fear,disgust;

    for(int i = 0; i < timer; i++){
       happy = current_emotion_value[name]["happy"] * emotion_trans["happy"]["happy"] + current_emotion_value[name]["quiet"] * emotion_trans["quiet"]["happy"] + current_emotion_value[name]["sad"] * emotion_trans["sad"]["happy"] + current_emotion_value[name]["surprise"] * emotion_trans["surprise"]["happy"] + current_emotion_value[name]["angry"] * emotion_trans["angry"]["happy"] + current_emotion_value[name]["fear"] * emotion_trans["fear"]["happy"] + current_emotion_value[name]["disgust"] * emotion_trans["disgust"]["happy"];
  
      quiet =  current_emotion_value[name]["happy"] * emotion_trans["happy"]["quiet"] + current_emotion_value[name]["quiet"] * emotion_trans["quiet"]["quiet"] + current_emotion_value[name]["sad"] * emotion_trans["sad"]["quiet"] + current_emotion_value[name]["surprise"] * emotion_trans["surprise"]["quiet"] + current_emotion_value[name]["angry"] * emotion_trans["angry"]["quiet"] + current_emotion_value[name]["fear"] * emotion_trans["fear"]["quiet"] + current_emotion_value[name]["disgust"] * emotion_trans["disgust"]["quiet"];
    
      sad =  current_emotion_value[name]["happy"] * emotion_trans["happy"]["sad"] + current_emotion_value[name]["quiet"] * emotion_trans["quiet"]["sad"] + current_emotion_value[name]["sad"] * emotion_trans["sad"]["sad"] + current_emotion_value[name]["surprise"] * emotion_trans["surprise"]["sad"] + current_emotion_value[name]["angry"] * emotion_trans["angry"]["sad"] + current_emotion_value[name]["fear"] * emotion_trans["fear"]["sad"] + current_emotion_value[name]["disgust"] * emotion_trans["disgust"]["sad"];
  
      surprise = current_emotion_value[name]["happy"] * emotion_trans["happy"]["surprise"] + current_emotion_value[name]["quiet"] * emotion_trans["quiet"]["surprise"] + current_emotion_value[name]["sad"] * emotion_trans["sad"]["surprise"] + current_emotion_value[name]["surprise"] * emotion_trans["surprise"]["surprise"] + current_emotion_value[name]["angry"] * emotion_trans["angry"]["surprise"] + current_emotion_value[name]["fear"] * emotion_trans["fear"]["surprise"] + current_emotion_value[name]["disgust"] * emotion_trans["disgust"]["surprise"];
  
      angry =  current_emotion_value[name]["happy"] * emotion_trans["happy"]["angry"] + current_emotion_value[name]["quiet"] * emotion_trans["quiet"]["angry"] + current_emotion_value[name]["sad"] * emotion_trans["sad"]["angry"] + current_emotion_value[name]["surprise"] * emotion_trans["surprise"]["angry"] + current_emotion_value[name]["angry"] * emotion_trans["angry"]["angry"] + current_emotion_value[name]["fear"] * emotion_trans["fear"]["angry"] + current_emotion_value[name]["disgust"] * emotion_trans["disgust"]["angry"];
    
      fear =  current_emotion_value[name]["happy"] * emotion_trans["happy"]["fear"] + current_emotion_value[name]["quiet"] * emotion_trans["quiet"]["fear"] + current_emotion_value[name]["sad"] * emotion_trans["sad"]["fear"] + current_emotion_value[name]["surprise"] * emotion_trans["surprise"]["fear"] + current_emotion_value[name]["angry"] * emotion_trans["angry"]["fear"]  +  current_emotion_value[name]["fear"] * emotion_trans["fear"]["fear"] + current_emotion_value[name]["disgust"] * emotion_trans["disgust"]["fear"];
     
      disgust =  current_emotion_value[name]["happy"] * emotion_trans["happy"]["disgust"] + current_emotion_value[name]["quiet"] * emotion_trans["quiet"]["disgust"] + current_emotion_value[name]["sad"] * emotion_trans["sad"]["disgust"] + current_emotion_value[name]["surprise"] * emotion_trans["surprise"]["disgust"] + current_emotion_value[name]["angry"] * emotion_trans["angry"]["disgust"] + current_emotion_value[name]["fear"] * emotion_trans["fear"]["disgust"] + current_emotion_value[name]["disgust"] * emotion_trans["disgust"]["disgust"];

    current_emotion_value[name]["happy"] = happy/(happy+quiet+sad+surprise+angry+fear+disgust);
    current_emotion_value[name]["quiet"] = quiet/(happy+quiet+sad+surprise+angry+fear+disgust);
    current_emotion_value[name]["sad"] = sad/(happy+quiet+sad+surprise+angry+fear+disgust);
    current_emotion_value[name]["surprise"] = surprise/(happy+quiet+sad+surprise+angry+fear+disgust);
    current_emotion_value[name]["angry"] = angry/(happy+quiet+sad+surprise+angry+fear+disgust);
    current_emotion_value[name]["fear"] = fear/(happy+quiet+sad+surprise+angry+fear+disgust);
    current_emotion_value[name]["disgust"] = disgust/(happy+quiet+sad+surprise+angry+fear+disgust);

    }
  }


  
};

int main(int argc, char **argv){
  bayesian app;
  ros::init(argc, argv, "emotion_bayesian");
  app.initialize();
  app.run();
  return 0;
}
