#include <iostream>
#include "zmq.hpp"
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string>
#include <assert.h>
#include "msgpack.hpp"

using namespace std;

struct speech{
  string tracking_ID;
  string speech_dictation;
  MSGPACK_DEFINE(tracking_ID,speech_dictation);
}speech_dic;

int main(){
  zmq::context_t context(1);
  zmq::socket_t publisher(context,ZMQ_PUB);
  publisher.bind("tcp://*:4413");
  speech_dic.tracking_ID = "111234123";
  speech_dic.speech_dictation = "こんちは";
  while(1){
    msgpack::sbuffer sbuf;
    msgpack::pack(&sbuf,speech_dic);
    zmq::message_t message_send(sbuf.size());
    memcpy(message_send.data(),sbuf.data(),sbuf.size());
    publisher.send(message_send);

    sleep(1);
  }
  return 0;
}
