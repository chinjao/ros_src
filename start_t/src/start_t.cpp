#include "ros/ros.h"
#include <sstream>
#include <string>
#include <iostream>
#include <fstream>

#include <unistd.h>

int main(){

  char ip[50];
  char final[100];

  scanf("%s",ip);

  sprintf(final,"telnet %s 39390 > ~/MMD/telnet.txt",ip);
  system(final);

  return 0;
}
