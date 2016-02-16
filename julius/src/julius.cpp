#include <iostream>
#include <boost/regex.hpp>
#include <boost/asio.hpp>
#include <fstream>
#include <ros/ros.h>
#include <unistd.h>

int main(int argc, char const* argv[]){
  
  system("/home/yhirai/dictation-kit-v4.3.1-linux/bin/julius -C /home/yhirai/dictation-kit-v4.3.1-linux/main.jconf -C /home/yhirai/dictation-kit-v4.3.1-linux/am-gmm.jconf -module");
 
  return 0;
}
