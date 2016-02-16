#include "picojson-master/picojson.h"

#include <fstream>
#include <iostream>
#include <cassert>
#include <memory>
#include <string>

using namespace std;
using namespace picojson;

int main() {
  value root;
  int c;
  map<std::string,std::string> json_map;
  while(1){
    {
      c = 0;
      ifstream stream("/home/yhirai/catkin_ws/src/verb/src/verb.json");
      if ( !stream.is_open() ) return 1;
      stream >> root;
      assert( get_last_error().empty() );
    }
    const picojson::value::object& obj = root.get<picojson::object>();
    for (picojson::value::object::const_iterator i = obj.begin(); i != obj.end(); ++i) {
         json_map.insert( make_pair(i->first,i->second.to_str() ) ); //insertする
    }
    c = json_map.count("動く");
    cout << c << endl;
    if(c > 0){
      object image = root.get<object>()["動く"].get<object>();
      cout << "command=" << image["command"].get<string>() << endl;
    }
    sleep(1);
  }
}