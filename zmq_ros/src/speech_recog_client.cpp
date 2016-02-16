/*
サービスの口を持っていたらいいんじゃね？

*/


#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <std_msgs/Int32.h>
#include <tf/transform_listener.h>

#include <cstdlib>
#include <ctime>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include "speech_msgs/SpeechSrv.h"

using namespace std;

class SpeechRecogClient
{
private:
  ros::NodeHandle n;

  ros::ServiceClient word_clt;
  //ros::ServiceServer goal_srv;
  

public:
  SpeechRecogClient() 
  { 
    word_clt
      = n.serviceClient<speech_msgs::SpeechSrv>("word_search");

    //goal_srv
    //  = n.advertiseService("word_search",
    //			   &SpeechRecogClient::, this);

  }

  ~SpeechRecogClient()
  {

  }

  void client()
  {
    cout << "input word: ";
    string word;
    cin >> word; 

    speech_msgs::SpeechSrv ssrv;
    ssrv.request.word = word;
    if( word_clt.call(ssrv) )
      {
	cout << "this word okao_id: " << ssrv.response.okao_id << endl;
      }
    else
      {
	cout << "no such word: " << word <<endl; 	
      }
  }


 




};

int main(int argc, char** argv)
{
  ros::init( argc, argv, "speech_recog_client" );
  SpeechRecogClient SRCObj;
  while(ros::ok())
    {
      SRCObj.client();
      //ros::spin();
    }
  return 0;
}
