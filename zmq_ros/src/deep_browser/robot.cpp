#include "include/speech.h"

void robot_browser(string action,int admin){
  zmq::context_t context(1);
  zmq::socket_t sender(context,ZMQ_PUSH);
  msgpack::sbuffer buffer;
  sender.connect("tcp://133.19.23.224:8081");
  map<string, string> dict;
  dict["senderName"] = "speechRecognition";
  dict["name"] = "Robot";
  dict["time"] = "";
  if(admin == 0)
    dict["speechRec"] = "あなたの命令は聞けません";
  else
    dict["speechRec"] = action;
  dict["place"] = "pioneer";
  msgpack::pack(buffer,dict);
  zmq::message_t message(buffer.size());
  memcpy(message.data(),buffer.data(),buffer.size());
  sender.send(message);
}

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
