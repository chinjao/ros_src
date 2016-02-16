#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <string>
#include <iostream>
#include <fstream>

#include<iconv.h>
#include <unistd.h>

using namespace std;


int main(int argc, char **argv)
{
   int count = 0;
   int search = 0;
   int mod = 0;
   char kk[20],before[20];
   ros::init(argc, argv, "listener");
   ros::NodeHandle n;
   while(1){

     std_msgs::StringConstPtr testdata = ros::topic::waitForMessage<std_msgs::String>("chatter");

     strcpy(kk,testdata->data.c_str());
     if(strcmp(kk,"roomba_connect") == 0 && strcmp(before,kk) != 0){
       system("sh ~/openjtalk/talk.sh \"ルンバと接続しています\"");
       system("roslaunch automove roomba.launch");
       system("sh ~/openjtalk/talk.sh \"ルンバとの接続を終了しました\"");
     }
     else if(strcmp(kk,"weather") == 0 && strcmp(before,kk) != 0){
       system("sh ~/bin/weather.sh");
     }
     else if(strcmp(kk,"dictation") == 0 && strcmp(before,kk) != 0 ){
       system("roslaunch automove dic.launch");
     }
/*     else if(strcmp(kk,"message") == 0 && strcmp(before,kk) != 0){
        system("roslaunch automove record.launch");
     }
     */
     else if(strcmp(kk,"message_talk") == 0 && strcmp(before,kk) != 0){
       system("aplay ~/record/out.wav");
     }
  
     /*
     else if(strcmp(kk,"=END=") == 0 && mod == 1){
       system("kill `ps -ef | grep julius| grep -v grep | awk '{print $2}'`");
       mod = 0;
     }
     */
     strcpy(before,kk);

     usleep(200000);

   }

  return 0;
}
