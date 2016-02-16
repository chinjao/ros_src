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
    emotion_trans["sad"]["quiet"] = 0.269;
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

    current_emotion_value["happy"] = 0;
    current_emotion_value["quiet"] = 1;
    current_emotion_value["sad"] = 0;
    current_emotion_value["surprise"] = 0;
    current_emotion_value["angry"] = 0;
    current_emotion_value["fear"] = 0;
    current_emotion_value["disgust"] = 0;

    timeline_emotion_value[0]["happy"] = 0;
    timeline_emotion_value[0]["quiet"] = 1;
    timeline_emotion_value[0]["sad"] = 0;
    timeline_emotion_value[0]["surprise"] = 0;
    timeline_emotion_value[0]["angry"] = 0;
    timeline_emotion_value[0]["fear"] = 0;

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
    float power[7][7];
	float sum_power = 0;
    for(int i = 0 ; i < 7 ; i++){
      for(int l = 0 ; l < 7 ; l++){
	power[i][l] = 1;
      }
    }
    for(int i = 0; i < 100 ; i++){
		power[0][0] *= emotion_trans["happy"]["happy"];
		power[0][1] *= emotion_trans["happy"]["quiet"];
		power[0][2] *= emotion_trans["happy"]["sad"];
		power[0][3] *= emotion_trans["happy"]["surprise"];
		power[0][4] *= emotion_trans["happy"]["angry"];
		power[0][5] *= emotion_trans["happy"]["fear"];
		power[0][6] *= emotion_trans["happy"]["disgust"];

		sum_power = power[0][0] + power[0][1] + power[0][2] + power[0][3] + power[0][4] + power[0][5] + power[0][6];

		power[0][0] = power[0][0] / sum_power;		
		power[0][1] = power[0][1] / sum_power;
		power[0][2] = power[0][2] / sum_power;
		power[0][3] = power[0][3] / sum_power;
		power[0][4] = power[0][4] / sum_power;
		power[0][5] = power[0][5] / sum_power;
		power[0][6] = power[0][6] / sum_power;

		power[1][0] *= emotion_trans["quiet"]["happy"];
		power[1][1] *= emotion_trans["quiet"]["quiet"];
		power[1][2] *= emotion_trans["quiet"]["sad"];
		power[1][3] *= emotion_trans["quiet"]["surprise"];
		power[1][4] *= emotion_trans["quiet"]["angry"];
		power[1][5] *= emotion_trans["quiet"]["fear"];
		power[1][6] *= emotion_trans["quiet"]["disgust"];

		sum_power = power[1][0] + power[1][1] + power[1][2] + power[1][3] + power[1][4] + power[1][5] + power[1][6];

		power[1][0] = power[1][0] / sum_power;
		power[1][1] = power[1][1] / sum_power;
		power[1][2] = power[1][2] / sum_power;
		power[1][3] = power[1][3] / sum_power;
		power[1][4] = power[1][4] / sum_power;
		power[1][5] = power[1][5] / sum_power;
		power[1][6] = power[1][6] / sum_power;

		power[2][0] *= emotion_trans["sad"]["happy"];
		power[2][1] *= emotion_trans["sad"]["quiet"];
		power[2][2] *= emotion_trans["sad"]["sad"];
		power[2][3] *= emotion_trans["sad"]["surprise"];
		power[2][4] *= emotion_trans["sad"]["angry"];
		power[2][5] *= emotion_trans["sad"]["fear"];
		power[2][6] *= emotion_trans["sad"]["disgust"];

		sum_power = power[2][0] + power[2][1] + power[2][2] + power[2][3] + power[2][4] + power[2][5] + power[2][6];

		power[2][0] = power[2][0] / sum_power;
		power[2][1] = power[2][1] / sum_power;
		power[2][2] = power[2][2] / sum_power;
		power[2][3] = power[2][3] / sum_power;
		power[2][4] = power[2][4] / sum_power;
		power[2][5] = power[2][5] / sum_power;
		power[2][6] = power[2][6] / sum_power;



		power[3][0] *= emotion_trans["surprise"]["happy"];
		power[3][1] *= emotion_trans["surprise"]["quiet"];
		power[3][2] *= emotion_trans["surprise"]["sad"];
		power[3][3] *= emotion_trans["surprise"]["surprise"];
		power[3][4] *= emotion_trans["surprise"]["angry"];
		power[3][5] *= emotion_trans["surprise"]["fear"];
		power[3][6] *= emotion_trans["surprise"]["disgust"];

		sum_power = power[3][0] + power[3][1] + power[3][2] + power[3][3] + power[3][4] + power[3][5] + power[3][6];

		power[3][0] = power[3][0] / sum_power;
		power[3][1] = power[3][1] / sum_power;
		power[3][2] = power[3][2] / sum_power;
		power[3][3] = power[3][3] / sum_power;
		power[3][4] = power[3][4] / sum_power;
		power[3][5] = power[3][5] / sum_power;
		power[3][6] = power[3][6] / sum_power;


		power[4][0] *= emotion_trans["angry"]["happy"];
		power[4][1] *= emotion_trans["angry"]["quiet"];
		power[4][2] *= emotion_trans["angry"]["sad"];
		power[4][3] *= emotion_trans["angry"]["surprise"];
		power[4][4] *= emotion_trans["angry"]["angry"];
		power[4][5] *= emotion_trans["angry"]["fear"];
		power[4][6] *= emotion_trans["angry"]["disgust"];

		sum_power = power[4][0] + power[4][1] + power[4][2] + power[4][3] + power[4][4] + power[4][5] + power[4][6];

		power[4][0] = power[4][0] / sum_power;
		power[4][1] = power[4][1] / sum_power;
		power[4][2] = power[4][2] / sum_power;
		power[4][3] = power[4][3] / sum_power;
		power[4][4] = power[4][4] / sum_power;
		power[4][5] = power[4][5] / sum_power;
		power[4][6] = power[4][6] / sum_power;

		power[5][0] *= emotion_trans["fear"]["happy"];
		power[5][1] *= emotion_trans["fear"]["quiet"];
		power[5][2] *= emotion_trans["fear"]["sad"];
		power[5][3] *= emotion_trans["fear"]["surprise"];
		power[5][4] *= emotion_trans["fear"]["angry"];
		power[5][5] *= emotion_trans["fear"]["fear"];
		power[5][6] *= emotion_trans["fear"]["disgust"];

		sum_power = power[5][0] + power[5][1] + power[5][2] + power[5][3] + power[5][4] + power[5][5] + power[5][6];

		power[5][0] = power[5][0] / sum_power;
		power[5][1] = power[5][1] / sum_power;
		power[5][2] = power[5][2] / sum_power;
		power[5][3] = power[5][3] / sum_power;
		power[5][4] = power[5][4] / sum_power;
		power[5][5] = power[5][5] / sum_power;
		power[5][6] = power[5][6] / sum_power;


		power[6][0] *= emotion_trans["disgust"]["happy"];
		power[6][1] *= emotion_trans["disgust"]["quiet"];
		power[6][2] *= emotion_trans["disgust"]["sad"];
		power[6][3] *= emotion_trans["disgust"]["surprise"];
		power[6][4] *= emotion_trans["disgust"]["angry"];
		power[6][5] *= emotion_trans["disgust"]["fear"];
		power[6][6] *= emotion_trans["disgust"]["disgust"];

		sum_power = power[6][0] + power[6][1] + power[6][2] + power[6][3] + power[6][4] + power[6][5] + power[6][6];

		power[6][0] = power[6][0] / sum_power;
		power[6][1] = power[6][1] / sum_power;
		power[6][2] = power[6][2] / sum_power;
		power[6][3] = power[6][3] / sum_power;
		power[6][4] = power[6][4] / sum_power;
		power[6][5] = power[6][5] / sum_power;
		power[6][6] = power[6][6] / sum_power;


    }

	for (int i = 0; i < 7; i++){
		for (int l = 0; l < 7; l++){
			printf("%.3f ", power[i][l]);
		}
		cout << endl;
	}
  }

};

int main(){
  bayesian app;
  app.initialize();
  app.run();
  return 0;
}
