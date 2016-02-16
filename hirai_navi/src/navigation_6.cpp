#include <ros/ros.h>
#include "std_msgs/String.h"
#include <sstream>
#include <string>
#include <iostream>
#include <fstream>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include<iconv.h>
#include <unistd.h>

using namespace std;
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){
  ros::init(argc, argv, "simple_navigation_goals");
  char kk[30];
  ros::NodeHandle n;
  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  //wait for the action server to come up
/*  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }
*/
/*  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
    }*/
  move_base_msgs::MoveBaseGoal goal;
  double goal_x,goal_y,goal_z,goal_w;
  int i,mod;
  char before[30];
  mod = 0;
  //we'll send a goal to the robot to move 1 meter forward
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();
  mod = 0;
  while(1){

    std_msgs::StringConstPtr testdata = ros::topic::waitForMessage<std_msgs::String>("chatter");
    strcpy(kk,testdata->data.c_str());
    if(strcmp(kk,"movebase") == 0 && mod == 0){
      mod = 1;
    }
    else if(strcmp(kk,"here1") == 0 && strcmp(before,kk) != 0 && mod == 1){
      cout << "hirai" << endl;
      
      goal_x = 2.7879755497;
      goal_y = 9.07890319824;
      goal_z = 0.714342971653;
      goal_w = 0.699795769385;
    
      goal.target_pose.pose.position.x = goal_x;
      goal.target_pose.pose.position.y = goal_y;
      goal.target_pose.pose.orientation.z = goal_z;
      goal.target_pose.pose.orientation.w = goal_w;
    
      ac.sendGoalAndWait(goal);
    
      ac.waitForResult();
      mod = 0;
    }

    else if(strcmp(kk,"here2") == 0 && strcmp(before,kk) != 0 && mod == 1){
      cout << "yuma" << endl; 
      goal_x = 3.9155125618;
      goal_y = -16.3433704376;
      goal_z = -0.722735105458;
      goal_w = 0.691125145931;
    
      goal.target_pose.pose.position.x = goal_x;
      goal.target_pose.pose.position.y = goal_y;
      goal.target_pose.pose.orientation.z = goal_z;
      goal.target_pose.pose.orientation.w = goal_w;
    
      ac.sendGoalAndWait(goal);
    
      ac.waitForResult();
      mod = 0;
    }

    else if(strcmp(kk,"localspace") == 0 && strcmp(before,kk) != 0 && mod == 1){
      cout << "yuma2" << endl;
      
      goal_x = 3.64904785156;
      goal_y = 1.87395739555;
      goal_z = -6.09923154112e-06;
      goal_w = 0.999999999981;
    
      goal.target_pose.pose.position.x = goal_x;
      goal.target_pose.pose.position.y = goal_y;
      goal.target_pose.pose.orientation.z = goal_z;
      goal.target_pose.pose.orientation.w = goal_w;
    
      ac.sendGoalAndWait(goal);
    
      ac.waitForResult();
      mod = 0;
    }

    strcpy(before,kk);
  }
  return 0;
  
}