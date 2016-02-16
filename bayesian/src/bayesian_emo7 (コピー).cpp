#include <iostream>
#include <unistd.h>
#include <string>
#include <assert.h>
#include <fstream>
#include <sys/stat.h>
#include <sys/types.h>
#include <map>
#include <cmath>
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
  int sevenvalue[7];
  map< int,map< string , float > > timeline_emotion_value;
  string before_obserb;
  map< int, string > timeline_emotion;
  map< string, float > out_emo;
  
public:
  void initialize(){

    emotion_trans["happy"]["happy"] = 0.443;
    emotion_trans["happy"]["quiet"] = 0.274;
    emotion_trans["happy"]["sad"] = 0.042;
    emotion_trans["happy"]["surprise"] = 0.121;
    emotion_trans["happy"]["angry"] = 0.029;
    emotion_trans["happy"]["fear"] = 0.017;
    emotion_trans["happy"]["disgust"] = 0.073;

    emotion_trans["quiet"]["happy"] = 0.471;
    emotion_trans["quiet"]["quiet"] = 0.259;
    emotion_trans["quiet"]["sad"] = 0.047;
    emotion_trans["quiet"]["surprise"] = 0.150;
    emotion_trans["quiet"]["angry"] = 0.035;
    emotion_trans["quiet"]["fear"] = 0.021;
    emotion_trans["quiet"]["disgust"] = 0.018;

    emotion_trans["sad"]["happy"] = 0.369;
    emotion_trans["sad"]["quiet"] = 0.296;
    emotion_trans["sad"]["sad"] = 0.099;
    emotion_trans["sad"]["surprise"] = 0.147;
    emotion_trans["sad"]["angry"] = 0.045;
    emotion_trans["sad"]["fear"] = 0.027;
    emotion_trans["sad"]["disgust"] = 0.015;

    emotion_trans["surprise"]["happy"] = 0.426;
    emotion_trans["surprise"]["quiet"] = 0.238;
    emotion_trans["surprise"]["sad"] = 0.058;
    emotion_trans["surprise"]["surprise"] = 0.186;
    emotion_trans["surprise"]["angry"] = 0.048;
    emotion_trans["surprise"]["fear"] = 0.021;
    emotion_trans["surprise"]["disgust"] = 0.023;

    emotion_trans["angry"]["happy"] = 0.355;
    emotion_trans["angry"]["quiet"] = 0.276;
    emotion_trans["angry"]["sad"] = 0.058;
    emotion_trans["angry"]["surprise"] = 0.158;
    emotion_trans["angry"]["angry"] = 0.093;
    emotion_trans["angry"]["fear"] = 0.038;
    emotion_trans["angry"]["disgust"] = 0.022;

    emotion_trans["fear"]["happy"] = 0.324;
    emotion_trans["fear"]["quiet"] = 0.288;
    emotion_trans["fear"]["sad"] = 0.058;
    emotion_trans["fear"]["surprise"] = 0.145;
    emotion_trans["fear"]["angry"] = 0.047;
    emotion_trans["fear"]["fear"] = 0.094;
    emotion_trans["fear"]["disgust"] = 0.044;

    emotion_trans["disgust"]["happy"] = 0.383;
    emotion_trans["disgust"]["quiet"] = 0.290;
    emotion_trans["disgust"]["sad"] = 0.054;
    emotion_trans["disgust"]["surprise"] = 0.135;
    emotion_trans["disgust"]["angry"] = 0.049;
    emotion_trans["disgust"]["fear"] = 0.035;
    emotion_trans["disgust"]["disgust"] = 0.055;

    current_emotion_value["happy"] = 0.201;
    current_emotion_value["quiet"] = 0.378;
    current_emotion_value["sad"] = 0.118;
    current_emotion_value["surprise"] = 0.076;
    current_emotion_value["angry"] = 0.083;
    current_emotion_value["fear"] = 0.070;
    current_emotion_value["disgust"] = 0.073;

    timeline_emotion_value[0]["happy"] = 0;
    timeline_emotion_value[0]["quiet"] = 1;
    timeline_emotion_value[0]["sad"] = 0;
    timeline_emotion_value[0]["surprise"] = 0;
    timeline_emotion_value[0]["angry"] = 0;
    timeline_emotion_value[0]["fear"] = 0;

    emo_to_obserb["happy"]["joy"] = 0.527;
    emo_to_obserb["quiet"]["joy"] = 0.000;
    emo_to_obserb["sad"]["joy"] = 0.016;
    emo_to_obserb["surprise"]["joy"] = 0.032;
    emo_to_obserb["fear"]["joy"] = 0.080;
    emo_to_obserb["disgust"]["joy"] = 0.016;
    emo_to_obserb["angry"]["joy"] = 0.104;

    emo_to_obserb["happy"]["sadness"] = 0.073;
    emo_to_obserb["quiet"]["sadness"] = 0.026;
    emo_to_obserb["sad"]["sadness"] = 0.565;
    emo_to_obserb["surprise"]["sadness"] = 0.016;
    emo_to_obserb["fear"]["sadness"] = 0.24;
    emo_to_obserb["disgust"]["sadness"] = 0.097;
    emo_to_obserb["angry"]["sadness"] = 0.060;

    emo_to_obserb["happy"]["neutral"] = 0.000;
    emo_to_obserb["quiet"]["neutral"] = 0.821;
    emo_to_obserb["sad"]["neutral"] = 0.032;
    emo_to_obserb["surprise"]["neutral"] = 0.032;
    emo_to_obserb["fear"]["neutral"] = 0.040;
    emo_to_obserb["disgust"]["neutral"] = 0.048;
    emo_to_obserb["angry"]["neutral"] = 0.045;

    emo_to_obserb["happy"]["angry"] = 0.218;
    emo_to_obserb["quiet"]["angry"] = 0.064;
    emo_to_obserb["sad"]["angry"] = 0.065;
    emo_to_obserb["surprise"]["angry"] = 0.063;
    emo_to_obserb["fear"]["angry"] = 0.100;
    emo_to_obserb["disgust"]["angry"] = 0.210;
    emo_to_obserb["angry"]["angry"] = 0.522;

    emo_to_obserb["happy"]["disgust"] = 0.036;
    emo_to_obserb["quiet"]["disgust"] = 0.090;
    emo_to_obserb["sad"]["disgust"] = 0.129;
    emo_to_obserb["surprise"]["disgust"] = 0.000;
    emo_to_obserb["fear"]["disgust"] = 0.100;
    emo_to_obserb["disgust"]["disgust"] = 0.468;
    emo_to_obserb["angry"]["disgust"] = 0.149;

    emo_to_obserb["happy"]["fear"] = 0.055;
    emo_to_obserb["quiet"]["fear"] = 0.000;
    emo_to_obserb["sad"]["fear"] = 0.194;
    emo_to_obserb["surprise"]["fear"] = 0.095;
    emo_to_obserb["fear"]["fear"] = 0.34;
    emo_to_obserb["disgust"]["fear"] = 0.113;
    emo_to_obserb["angry"]["fear"] = 0.045;

    emo_to_obserb["happy"]["surprise"] = 0.090;
    emo_to_obserb["quiet"]["surprise"] = 0.000;
    emo_to_obserb["sad"]["surprise"] = 0.000;
    emo_to_obserb["surprise"]["surprise"] = 0.762;
    emo_to_obserb["fear"]["surprise"] = 0.100;
    emo_to_obserb["disgust"]["surprise"] = 0.048;
    emo_to_obserb["angry"]["surprise"] = 0.075;

    current_emotion = "quiet";
  }
  
  void run(){
    timeline = 1;
    int iii = 0;
    while(1){
      cout << "please current obserb 1.joy 2.neutral 3.sadness 4.angry 5.surprise 6.fear 7.disgust:";
      cin >> iii;
       
      
      cout << endl;
      
      if(pos+neg+nor != 1){
	if(pos != 0)
	  pos = pos/(pos+neg+nor);
	if(neg != 0)
	  neg = neg/(pos+neg+nor);
	if(nor != 0)
	  nor = nor/(pos+neg+nor);
      }
      multiply_before_sum();

      string now = now_emo();

      cout << "Current emotion:" << now << endl;
 
      printf("happy:%.3f\n",current_emotion_value["happy"]);
      printf("quiet:%.3f\n",current_emotion_value["quiet"]);
      printf("sad:%.3f\n",current_emotion_value["sad"]);
      printf("surprise:%.3f\n",current_emotion_value["surprise"]);
      printf("angry:%.3f\n",current_emotion_value["angry"]);
      printf("fear:%.3f\n",current_emotion_value["fear"]);
      printf("disgust:%.3f\n",current_emotion_value["disgust"]);
 
      cout << endl;
      /*
      current_emotion_value["happy"] += 1;
      current_emotion_value["quiet"] += 1;
      
      current_emotion_value["sad"] += 1;
      
      current_emotion_value["surprise"] += 1;
      
      current_emotion_value["angry"] += 1;
      
      current_emotion_value["fear"] += 1;
      current_emotion_value["disgust"] += 1;
      */
      
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

};

int main(){
  bayesian app;
  app.initialize();
  app.run();
  return 0;
}
