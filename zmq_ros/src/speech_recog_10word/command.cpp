#include "include/speech.h"

extern void baxterarm_up(speech_msgs::Extract ext_msg,string name,string admin);
extern void move_command(speech_msgs::Extract ext_msg,string name,string admin);
extern string TIDtoName(int okao_id);


void command(speech_msgs::Extract ext_msg,int okao_id,string name){
  ros::NodeHandle nh;
    int c;
    map<std::string,std::string> json_map,json_map2;
    value root,root2,root3;
    std::ifstream ifs("/home/yhirai/catkin_ws/src/noun/src/rank.txt");
    string rank[5];
    int i = 0;
    std_msgs::String command_msg;
    std_msgs::String chat_msg;
    ros::Publisher command_pub = nh.advertise<std_msgs::String>("chatter", 1000);
    string aa;
    string bb = "なし";
    int robot = 0;
    int sub_robot = 0;
    int ok = 0;
    int chat_ok = 0;
    string com;
    string admin = "admin";
    string aa2;
    if(ext_msg.verb.size() == 0)
      return;
    cout << "command now" << endl;
    if(ext_msg.com_jud == 1){
    }
    else{
      aa2 = ext_msg.verb[0];
      root2 = value();
      c = 0;
      ifstream stream2("/home/yhirai/catkin_ws/src/verb/src/verb_admin.json");
      stream2 >> root2;
      //   cout << com << endl;
      const picojson::value::object& obj2 = root2.get<picojson::object>();
      for (picojson::value::object::const_iterator i2 = obj2.begin(); i2 != obj2.end(); ++i2) {
	json_map2.insert( make_pair(i2->first,i2->second.to_str() ) );
      } 
      c = json_map2.count(aa2);
      if(c > 0){
	object image2 = root2.get<object>()[aa2].get<object>();
	// cout << "command=" << image2["command"].get<string>() << endl;
	com = image2["command"].get<string>();
	admin = image2["admin"].get<string>();
      }
      if(com == "move"){
	cout << name << endl;
	move_command(ext_msg,name,admin);
      }
      else if(com == "up"){
	baxterarm_up(ext_msg,name,admin);	
      }
    }
  }
    

