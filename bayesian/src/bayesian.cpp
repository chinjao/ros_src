#include <iostream>
#include <unistd.h>
#include <string>
#include <assert.h>
#include <fstream>
#include <sys/stat.h>
#include <sys/types.h>
#include <map>
#include <cmath>
#include <time.h>
#include <stdio.h>


using namespace std;

class bayesian
{
private:
  map< string,map< string,float > > emotion_trans,emo_to_obserb;
  map< string,float> current_emotion_value;
  float pos,neg,nor;
  string current_obserb;
  string current_emotion;
  int timeline;
  map< int,map< string , float > > timeline_emotion_value;
  string before_obserb;
  map< int, string > timeline_emotion;
  map< string, float > out_emo;
  
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

    current_emotion_value["happy"] = 0.201;
    current_emotion_value["quiet"] = 0.378;
    current_emotion_value["sad"] = 0.118;
    current_emotion_value["surprise"] = 0.076;
    current_emotion_value["angry"] = 0.083;
    current_emotion_value["fear"] = 0.070;
    current_emotion_value["disgust"] = 0.073;

    timeline_emotion_value[0]["happy"] = 0.201;
    timeline_emotion_value[0]["quiet"] = 0.378;
    timeline_emotion_value[0]["sad"] = 0.118;
    timeline_emotion_value[0]["surprise"] = 0.076;
    timeline_emotion_value[0]["angry"] = 0.083;
    timeline_emotion_value[0]["fear"] = 0.070;
    timeline_emotion_value[0]["disgust"] = 0.073;

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
    timeline = 1;
    int iii = 0;
    time_t start_time;
    while(1){
      start_time = time(NULL);
      cout << "please current obserb 1.positive 2.negative 3.normal:";
      cin >> iii;

      if(iii == 1)
	current_obserb = "positive";
      else if(iii == 2)
	current_obserb = "negative";
      else if(iii == 3)
	current_obserb = "normal";
      else
	current_obserb = "normal";
      
      
      
      cout << endl;
      
      if(pos+neg+nor != 1){
	if(pos != 0)
	  pos = pos/(pos+neg+nor);
	if(neg != 0)
	  neg = neg/(pos+neg+nor);
	if(nor != 0)
	  nor = nor/(pos+neg+nor);
      }
      int time_aaa = difftime(time(NULL),start_time);
      
      timeline_emo(time_aaa);


      multiply_before_sum();

      string now = now_emo();

     
      printf("happy:%.3f\n",current_emotion_value["happy"]);
      printf("quiet:%.3f\n",current_emotion_value["quiet"]);
      printf("sad:%.3f\n",current_emotion_value["sad"]);
      printf("surprise:%.3f\n",current_emotion_value["surprise"]);
      printf("angry:%.3f\n",current_emotion_value["angry"]);
      printf("fear:%.3f\n",current_emotion_value["fear"]);
      printf("disgust:%.3f\n",current_emotion_value["disgust"]);

      cout << "Current emotion:" << now << endl;
 

 
      cout << endl;
      
      timeline_emotion[timeline] = now;
      timeline++;
      before_obserb = current_obserb;
    }
  }

private:

  void multiply_before_sum(){
    string emo = now_emo();
    float happy,quiet,sad,surprise,angry,fear,disgust;
    float sum_emo;
 
    happy = emo_to_obserb["happy"][current_obserb] * emotion_trans["happy"]["happy"] * current_emotion_value["happy"] + emo_to_obserb["happy"][current_obserb] * emotion_trans["quiet"]["happy"] * current_emotion_value["quiet"] + emo_to_obserb["happy"][current_obserb] * emotion_trans["sad"]["happy"] * current_emotion_value["sad"] + emo_to_obserb["happy"][current_obserb] * emotion_trans["surprise"]["happy"] * current_emotion_value["surprise"] + emo_to_obserb["happy"][current_obserb] * emotion_trans["angry"]["happy"] * current_emotion_value["angry"] + emo_to_obserb["happy"][current_obserb] * emotion_trans["fear"]["happy"] * current_emotion_value["fear"] + emo_to_obserb["happy"][current_obserb] * emotion_trans["disgust"]["happy"] * current_emotion_value["disgust"];

    quiet = emo_to_obserb["quiet"][current_obserb] * emotion_trans["happy"]["quiet"] * current_emotion_value["happy"] + emo_to_obserb["quiet"][current_obserb] * emotion_trans["quiet"]["quiet"] * current_emotion_value["quiet"] + emo_to_obserb["quiet"][current_obserb] * emotion_trans["sad"]["quiet"] * current_emotion_value["sad"] + emo_to_obserb["quiet"][current_obserb] * emotion_trans["surprise"]["quiet"] * current_emotion_value["surprise"] + emo_to_obserb["quiet"][current_obserb] * emotion_trans["angry"]["quiet"] * current_emotion_value["angry"] + emo_to_obserb["quiet"][current_obserb] * emotion_trans["fear"]["quiet"] * current_emotion_value["fear"] + emo_to_obserb["quiet"][current_obserb] * emotion_trans["quiet"]["happy"] * current_emotion_value["disgust"];

    sad = emo_to_obserb["sad"][current_obserb] * emotion_trans["happy"]["sad"] * current_emotion_value["happy"] + emo_to_obserb["sad"][current_obserb] * emotion_trans["quiet"]["sad"] * current_emotion_value["quiet"] + emo_to_obserb["sad"][current_obserb] * emotion_trans["sad"]["sad"] * current_emotion_value["sad"] + emo_to_obserb["sad"][current_obserb] * emotion_trans["surprise"]["sad"] * current_emotion_value["surprise"] + emo_to_obserb["sad"][current_obserb] * emotion_trans["angry"]["sad"] * current_emotion_value["angry"] + emo_to_obserb["sad"][current_obserb] * emotion_trans["fear"]["sad"] * current_emotion_value["fear"] + emo_to_obserb["sad"][current_obserb] * emotion_trans["disgust"]["sad"] * current_emotion_value["disgust"];


    surprise = emo_to_obserb["surprise"][current_obserb] * emotion_trans["happy"]["surprise"] * current_emotion_value["happy"] + emo_to_obserb["surprise"][current_obserb] * emotion_trans["quiet"]["surprise"] * current_emotion_value["quiet"] + emo_to_obserb["surprise"][current_obserb] * emotion_trans["sad"]["surprise"] * current_emotion_value["sad"] + emo_to_obserb["surprise"][current_obserb] * emotion_trans["surprise"]["surprise"] * current_emotion_value["surprise"] + emo_to_obserb["surprise"][current_obserb] * emotion_trans["angry"]["surprise"] * current_emotion_value["angry"] + emo_to_obserb["surprise"][current_obserb] * emotion_trans["fear"]["surprise"] * current_emotion_value["fear"] + emo_to_obserb["surprise"][current_obserb] * emotion_trans["disgust"]["surprise"] * current_emotion_value["disgust"];


    angry = emo_to_obserb["angry"][current_obserb] * emotion_trans["happy"]["angry"] * current_emotion_value["happy"] + emo_to_obserb["angry"][current_obserb] * emotion_trans["quiet"]["angry"] * current_emotion_value["quiet"] + emo_to_obserb["angry"][current_obserb] * emotion_trans["sad"]["angry"] * current_emotion_value["sad"] + emo_to_obserb["angry"][current_obserb] * emotion_trans["surprise"]["angry"] * current_emotion_value["surprise"] + emo_to_obserb["angry"][current_obserb] * emotion_trans["angry"]["angry"] * current_emotion_value["angry"] + emo_to_obserb["angry"][current_obserb] * emotion_trans["fear"]["angry"] * current_emotion_value["fear"] + emo_to_obserb["angry"][current_obserb] * emotion_trans["disgust"]["angry"] * current_emotion_value["disgust"];


    fear = emo_to_obserb["fear"][current_obserb] * emotion_trans["happy"]["fear"] * current_emotion_value["happy"] + emo_to_obserb["fear"][current_obserb] * emotion_trans["quiet"]["fear"] * current_emotion_value["quiet"] + emo_to_obserb["fear"][current_obserb] * emotion_trans["sad"]["fear"] * current_emotion_value["sad"] + emo_to_obserb["fear"][current_obserb] * emotion_trans["surprise"]["fear"] * current_emotion_value["surprise"] + emo_to_obserb["fear"][current_obserb] * emotion_trans["angry"]["fear"] * current_emotion_value["angry"] + emo_to_obserb["fear"][current_obserb] * emotion_trans["fear"]["fear"] * current_emotion_value["fear"] + emo_to_obserb["fear"][current_obserb] * emotion_trans["disgust"]["fear"] * current_emotion_value["disgust"];


    disgust = emo_to_obserb["disgust"][current_obserb] * emotion_trans["happy"]["disgust"] * current_emotion_value["happy"] + emo_to_obserb["disgust"][current_obserb] * emotion_trans["quiet"]["disgust"] * current_emotion_value["quiet"] + emo_to_obserb["disgust"][current_obserb] * emotion_trans["sad"]["disgust"] * current_emotion_value["sad"] + emo_to_obserb["disgust"][current_obserb] * emotion_trans["surprise"]["disgust"] * current_emotion_value["surprise"] + emo_to_obserb["disgust"][current_obserb] * emotion_trans["angry"]["disgust"] * current_emotion_value["angry"] + emo_to_obserb["disgust"][current_obserb] * emotion_trans["fear"]["disgust"] * current_emotion_value["fear"] + emo_to_obserb["disgust"][current_obserb] * emotion_trans["disgust"]["disgust"] * current_emotion_value["disgust"];


    sum_emo = happy+quiet+sad+surprise+fear+disgust+angry;

    happy = happy/sum_emo;
    quiet = quiet/sum_emo;
    sad = sad/sum_emo;
    surprise = surprise/sum_emo;
    angry = angry/sum_emo;
    fear = fear/sum_emo;
    disgust = disgust/sum_emo;

    current_emotion_value["happy"] = happy;
    current_emotion_value["quiet"] = quiet;
    current_emotion_value["sad"] = sad;
    current_emotion_value["surprise"] = surprise;
    current_emotion_value["angry"] = angry;
    current_emotion_value["fear"] = fear;
    current_emotion_value["disgust"] = disgust;

    timeline_emotion_value[timeline]["happy"] = happy;
    timeline_emotion_value[timeline]["quiet"] = quiet;
    timeline_emotion_value[timeline]["sad"] = sad;
    timeline_emotion_value[timeline]["surprise"] = surprise;
    timeline_emotion_value[timeline]["angry"] = angry;
    timeline_emotion_value[timeline]["fear"] = fear;
    timeline_emotion_value[timeline]["disgust"] = disgust;
       
  }


  string now_emo(){
    string emo;
    float max = 0;
    for(map<string,float>::iterator it = current_emotion_value.begin();
	it != current_emotion_value.end(); it++)
      if (max < it->second)
	{
	  max = it->second;
	  emo = it->first;
	}
    return emo;
  }

  void timeline_emo(int timer){
    float happy,quiet,sad,surprise,angry,fear,disgust;

    for(int i = 0; i < timer; i++){
      happy = current_emotion_value["happy"] * emotion_trans["happy"]["happy"] + current_emotion_value["quiet"] * emotion_trans["quiet"]["happy"] + current_emotion_value["sad"] * emotion_trans["sad"]["happy"] + current_emotion_value["surprise"] * emotion_trans["surprise"]["happy"] + current_emotion_value["angry"] * emotion_trans["angry"]["happy"] + current_emotion_value["fear"] * emotion_trans["fear"]["happy"] + current_emotion_value["disgust"] * emotion_trans["disgust"]["happy"];
  
      quiet =  current_emotion_value["happy"] * emotion_trans["happy"]["quiet"] + current_emotion_value["quiet"] * emotion_trans["quiet"]["quiet"] + current_emotion_value["sad"] * emotion_trans["sad"]["quiet"] + current_emotion_value["surprise"] * emotion_trans["surprise"]["quiet"] + current_emotion_value["angry"] * emotion_trans["angry"]["quiet"] + current_emotion_value["fear"] * emotion_trans["fear"]["quiet"] + current_emotion_value["disgust"] * emotion_trans["disgust"]["quiet"];
    
      sad =  current_emotion_value["happy"] * emotion_trans["happy"]["sad"] + current_emotion_value["quiet"] * emotion_trans["quiet"]["sad"] + current_emotion_value["sad"] * emotion_trans["sad"]["sad"] + current_emotion_value["surprise"] * emotion_trans["surprise"]["sad"] + current_emotion_value["angry"] * emotion_trans["angry"]["sad"] + current_emotion_value["fear"] * emotion_trans["fear"]["sad"] + current_emotion_value["disgust"] * emotion_trans["disgust"]["sad"];
  
      surprise = current_emotion_value["happy"] * emotion_trans["happy"]["surprise"] + current_emotion_value["quiet"] * emotion_trans["quiet"]["surprise"] + current_emotion_value["sad"] * emotion_trans["sad"]["surprise"] + current_emotion_value["surprise"] * emotion_trans["surprise"]["surprise"] + current_emotion_value["angry"] * emotion_trans["angry"]["surprise"] + current_emotion_value["fear"] * emotion_trans["fear"]["surprise"] + current_emotion_value["disgust"] * emotion_trans["disgust"]["surprise"];
  
      angry =  current_emotion_value["happy"] * emotion_trans["happy"]["angry"] + current_emotion_value["quiet"] * emotion_trans["quiet"]["angry"] + current_emotion_value["sad"] * emotion_trans["sad"]["angry"] + current_emotion_value["surprise"] * emotion_trans["surprise"]["angry"] + current_emotion_value["angry"] * emotion_trans["angry"]["angry"] + current_emotion_value["fear"] * emotion_trans["fear"]["angry"] + current_emotion_value["disgust"] * emotion_trans["disgust"]["angry"];
    
      fear =  current_emotion_value["happy"] * emotion_trans["happy"]["fear"] + current_emotion_value["quiet"] * emotion_trans["quiet"]["fear"] + current_emotion_value["sad"] * emotion_trans["sad"]["fear"] + current_emotion_value["surprise"] * emotion_trans["surprise"]["fear"] + current_emotion_value["angry"] * emotion_trans["angry"]["fear"]  +  current_emotion_value["fear"] * emotion_trans["fear"]["fear"] + current_emotion_value["disgust"] * emotion_trans["disgust"]["fear"];
     
      disgust =  current_emotion_value["happy"] * emotion_trans["happy"]["disgust"] + current_emotion_value["quiet"] * emotion_trans["quiet"]["disgust"] + current_emotion_value["sad"] * emotion_trans["sad"]["disgust"] + current_emotion_value["surprise"] * emotion_trans["surprise"]["disgust"] + current_emotion_value["angry"] * emotion_trans["angry"]["disgust"] + current_emotion_value["fear"] * emotion_trans["fear"]["disgust"] + current_emotion_value["disgust"] * emotion_trans["disgust"]["disgust"];

    current_emotion_value["happy"] = happy/(happy+quiet+sad+surprise+angry+fear+disgust);
    current_emotion_value["quiet"] = quiet/(happy+quiet+sad+surprise+angry+fear+disgust);
    current_emotion_value["sad"] = sad/(happy+quiet+sad+surprise+angry+fear+disgust);
    current_emotion_value["surprise"] = surprise/(happy+quiet+sad+surprise+angry+fear+disgust);
    current_emotion_value["angry"] = angry/(happy+quiet+sad+surprise+angry+fear+disgust);
    current_emotion_value["fear"] = fear/(happy+quiet+sad+surprise+angry+fear+disgust);
    current_emotion_value["disgust"] = disgust/(happy+quiet+sad+surprise+angry+fear+disgust);

    }
  }

};

int main(){
  bayesian app;
  app.initialize();
  app.run();
  return 0;
}
