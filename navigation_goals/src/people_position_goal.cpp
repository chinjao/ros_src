/*
1.名前を受け取ったら、OKAO_IDに変換
2.モジュールにすべてのDIS_IDをリクエスト、
3.レスポンスを受け取り、OKAO_IDが一致するか確かめる
4.もし一致すれば、その時の名前とポジションを受け取る
5.DIS_IDがなくなるまで３，4を繰り返す
6.もし名前がなければ、ゴール地点の指定はなされない
 */

#include "ros/ros.h"
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
//#include <std_msgs>
#include "std_msgs/String.h"
#include <map>
#include <string>
#include "okao_client/OkaoPosSrv.h"

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
using namespace std;




int main(int argc, char** argv)
{
  ros::init(argc, argv, "goal_client");
  ros::NodeHandle n;

  map<string, int> nameToId;
  nameToId["Unknown"] = 0;
  nameToId["Uema"] = 1;
  nameToId["Morioka"] = 2;
  nameToId["Nogawa"] = 3;
  nameToId["Ikegami"] = 4;
  nameToId["Adachi"] = 5;
  nameToId["Hirai"] = 6;
  nameToId["Teranishi"] = 7;
  nameToId["Kawakita"] = 8;
  nameToId["Hukuhara"] = 9;
  nameToId["Nishida"] = 10;
  nameToId["Kawamoto"] = 11;
  nameToId["Yatsuzuka"] = 12;
 
  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);
  
  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0)))
    {
      ROS_INFO("Waiting for the move_base action server to come up");
    }

  while(ros::ok())
    {
      //名前(name)のサブスクライブ
      std_msgs::StringConstPtr name = ros::topic::waitForMessage<std_msgs::String>("chatter");

      if(name->data != "")
	{
	  //クライアント

	  ros::ServiceClient okaoClient = n.serviceClient<okao_client::OkaoPosSrv>("OkaoClient_srv");
	  //okao_client::OkaoPosSrv okao_num;
	  //okao_num.request.rule = "allnumreq";
	  //okaoClient.call(okao_num);
	  //認識人数の取得
	  //int people_num = okao_num.response.n;
	  
	  //for(int i = 0; i < people_num; ++i)
	  //  {
	      //人物の取得
	  okao_client::OkaoPosSrv okao_data;
	  okao_data.request.okao_id = nameToId[name->data];
	  okao_data.request.rule = "request";
	  std::cout << "send okao_data.request.id:" << okao_data.request.okao_id << std::endl;
	  if(okaoClient.call(okao_data))
	    {
	      
	      //subscribeした名前と、requestしたIDと比較
	      // if(okao_data.response.okao_id == nameToId[name->data])
	      //  {   
	      std::cout << "replay:(" << okao_data.response.res_x << "," << okao_data.response.res_y << ")" << std::endl;
	      /*
	      goal_x = okao_data.response.res_x;
	      goal_y = okao_data.response.res_y;
	      goal_z = 0;
	      goal_w = 1;
	      */
	      std::cout << name  << " is found!" << std::endl;
	      std::cout << name  << " planning start! " << std::endl;

	      move_base_msgs::MoveBaseGoal goal;	      
	      goal.target_pose.header.stamp = ros::Time::now();
	      goal.target_pose.header.frame_id = "map";
	      goal.target_pose.pose.position.x = okao_data.response.res_x;
	      goal.target_pose.pose.position.y = okao_data.response.res_y;
	      goal.target_pose.pose.orientation.z = 0;
	      goal.target_pose.pose.orientation.w = 1;
	      
	      ROS_INFO("Sending goal");
	      ac.sendGoalAndWait(goal);
	      ac.waitForResult();
	      
	      if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
		ROS_INFO("Hooray");
	      else
		ROS_INFO("Failed");   
	    }
	}	
    }
  return 0;
}


