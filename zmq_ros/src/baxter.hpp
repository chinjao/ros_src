using namespace std;
using namespace picojson;
ros::NodeHandle nh;

void robot_browser(string action,int admin);

int robot_number(string noun){
  ifstream stream("/home/yhirai/catkin_ws/src/noun/src/robot.json");
  value root;
  int c;
  map<std::string,std::string> json_map;
  stream >> root;
  const picojson::value::object& obj = root.get<picojson::object>();
  for (picojson::value::object::const_iterator i = obj.begin(); i != obj.end(); ++i) {
    json_map.insert( make_pair(i->first,i->second.to_str() ) );
  }
  c = json_map.count(noun);

  if(c > 0){
    object image = root.get<object>()[noun].get<object>();
    double hh = image["robot_num"].get<double>();
    return hh;
  }
  else{
    return 0;
  }
}

void armdown(speech_msgs::Extract ext_msg,string name, string admin){
  int i;
  i = 0;
  ros::Subscriber ok_sub;
  cout << i << endl;
}

  void baxterarm_up(speech_msgs::Extract ext_msg,string name,string admin){
    int robot = 0;
    if(admin != name){
      robot_browser("no",0);
      cout << "あなたじゃだめですよ" << endl;
      return;
    }
    string hh;
    std_msgs::String command_msg;
    ros::Publisher baxter_pub = nh.advertise<std_msgs::String>("baxter_chatter", 1000);
    for(int i = 0 ; i < ext_msg.noun_num ; i++){
      robot = robot_number(ext_msg.noun[i]);
      if(robot != 0)
	break;
    }
    map<std::string,std::string> json_map;
    value root;
    for(int i = 1; i < ext_msg.noun_num ; i++){
      string aa = "";
      hh = "";
      aa = ext_msg.noun[i];
      root = value();
      robot = robot_number(aa);
      int c = 0;
      ifstream stream("/home/yhirai/catkin_ws/src/noun/src/noun.json");
      stream >> root;

      const picojson::value::object& obj = root.get<picojson::object>();
      for (picojson::value::object::const_iterator i = obj.begin(); i != obj.end(); ++i) {
	json_map.insert( make_pair(i->first,i->second.to_str() ) );
      }
      c = json_map.count(aa);

      if(c > 0){
	object image = root.get<object>()[aa].get<object>();
	//    cout << "command=" << image["command"].get<string>() << endl;                                                                                 
	hh = image["command"].get<string>();
      }
      if(robot == 3 && hh == "righthand"){
	command_msg.data = "armup_r";
	cout << command_msg.data << endl;
	baxter_pub.publish(command_msg);
      }
      else if(robot == 3 && hh == "lefthand"){
	command_msg.data = "armup_l";
	cout << command_msg.data << endl;
	baxter_pub.publish(command_msg);
      }
      else if(robot == 3 && hh == "doublehand"){
	command_msg.data = "armup_d";
	cout << command_msg.data << endl;
	baxter_pub.publish(command_msg);
      }
    }
    
  }


