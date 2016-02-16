#include <iostream>
#include <boost/regex.hpp>
#include <boost/asio.hpp>
#include <fstream>
#include <ros/ros.h>
#include <unistd.h>
#include <sstream>

#define BUFSIZE 256


int main(int argc,char **argv){

  using namespace boost::asio;
  using namespace std;
  ifstream fs1("/home/yhirai/docomo/message.txt");
  char buffer[50];
  string buf,buf2,buf3,buf4;
  ostringstream ostr;
  char command[100];
  buf2 = "sh ~/openjtalk/talk.sh \"";
  buf3  = "\"";

  while(fs1.getline(buffer,BUFSIZE) != NULL){
    // buf4 = "";
    sprintf(command,"sh ~/openjtalk/talk.sh \"%s\"",buffer);
    //ostr << "sh ~/openjtalk/talk.sh \"" << buf << "\"";
    //cout << ostr.str() << endl;
    //buf4 = buf2 + buf + buf3;
    //cout << buf4 << endl;
    //memcpy(command,buf4.c_str(),BUFSIZE);
    //cout << command << endl;
    system(command);
  }

}
