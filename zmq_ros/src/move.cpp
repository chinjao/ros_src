#include "speech.h"
 void move_command(speech_msgs::Extract ext_msg,string name,string admin){
    int c;
    map<std::string,std::string> json_map;
    value root;
    int i = 0;
    std_msgs::String command_msg;
    ros::Publisher command_pub = nh.advertise<std_msgs::String>("chatter", 1000);
    string aa;
    string com;
    string aa2;
    int chat_ok;
    string name_com;
    if(admin != name){
      cout << name << endl;
      robot_browser("no",0);
      cout << "あなたじゃだめですよ" << endl;
      return;
    }
    chat_ok = 1;
    for(i = 0; i < ext_msg.noun_num ; i++){
      aa = "";
      aa = ext_msg.noun[i];
	
      root = value();
      c = 0;
 
      ifstream stream("/home/yhirai/catkin_ws/src/noun/src/noun.json");
      stream >> root;
	
      const picojson::value::object& obj = root.get<picojson::object>();
      for (picojson::value::object::const_iterator i = obj.begin(); i != obj.end(); ++i) {
	json_map.insert( make_pair(i->first,i->second.to_str() ) );
      }
      c = json_map.count(aa);
      if(c > 0){
	object image = root.get<object>()[aa].get<object>();
	command_msg.data = image["command"].get<string>();
	name_com = image["word"].get<string>();
      }
    }
    if(command_msg.data != ""){
      command_pub.publish(command_msg);
      string output;
      cout << "Go to " << name_com << "'s position." << endl;
      output = name_com + "の場所に移動します";
      robot_browser(output,1);
    }
  }
