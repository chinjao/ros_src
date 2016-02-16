#include "ros/ros.h"

#include <sstream>
#include <string>
#include <iostream>
#include <fstream>

#include<iconv.h>
#include <unistd.h>

using namespace std;

int main(int argc, char **argv)
{

  iconv_t ic;
  ic = iconv_open("UTF-8","Shift_JIS");
  int count = 0;
  char ss[100];
  char human[30][20];
  char hito[20];
  FILE *fp;
  int a = 0;
  int b = 0;
  int c = 0;
  int d = 0;
  int i = 0;
  char *ptr_in;
  char *ptr_out;
  size_t mybufsz = (size_t)19;
  while (1)
  {
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */

    fp = fopen("/home/yhirai/jjj/rere.txt","r");
      //     cout << "ファイルに名前が入っていません" << endl;
 

    while(fgets(ss,256,fp) != NULL){
      for(i = 0 ; ss[i] != '\0' ; i++){
	if(ss[i] == '='&&a == 0){
	  b = 1;
	  a = 1;
	}
	else if(ss[i] == '='&&a == 1){
	  hito[d] = '\0';
	  b = 0;
	  a = 0;
	  c = 1;
	}
	else if(b == 1){
	  hito[d]=ss[i];
	  d++;
	} 
      }
      if(c == 1){
	strcpy(human[count],hito);
	ptr_in = hito;
	ptr_out = human[count];
	//cout << hito << endl;
	iconv(ic,&ptr_in,&mybufsz,&ptr_out,&mybufsz);
	c = 0;
	d = 0;
	count++;
      }
    }


    if(strcmp(human[count-1],"END") == 0)
      return(0);

    /**
     * The publish() function is how you send messages. The parameter
     * is the message object. The type of this object must agree with the type
     * given as a template parameter to the advertise<>() call, as was done
     * in the constructor above.
     */

    count = 0;
    fclose(fp);
    usleep(200000);
  }


  return 0;
}
