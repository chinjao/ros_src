/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2010, Kyoto University and Honda Motor Co.,Ltd. All rights reserved.
 *
 * HARK was developed by researchers in Okuno Laboratory at the Kyoto University and 
 * Honda Research Institute Japan Co.,Ltd.
 *
 * Redistribution and use in source and binary forms, with or without modification, are 
 * permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice, this list 
 *    of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright notice, this 
 *    list of conditions and the following disclaimer in the documentation and/or other 
 *    materials provided with the distribution.
 *  * Neither the name of Kyoto University and Honda Motor Co.,Ltd., Inc. nor the names 
 *    of its contributors may be used to endorse or promote products derived from this 
 *    software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY 
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES 
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT 
 * SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCI-
 * DENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, 
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSI-
 * NESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTR-
 * ACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY 
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#define NDEBUG

#include <iostream>
#include <BufferedNode.h>
#include <Buffer.h>
#include <../config.h>
#include "TimeStamp.h"
#include <csignal>

#ifdef ENABLE_ROS

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include "hark_msgs/HarkWave.h"
#include "hark_msgs/HarkWaveVal.h"
#include "HarkRosGlobals.h"
#include <boost/thread.hpp>

#include <Matrix.h>
#include <math.h>
#include <string>
#include <stdio.h>
#include <cstdlib>
#include <unistd.h>
#include <sys/time.h>
#include <queue>

using namespace std;
using namespace boost;
using namespace FD;

class callbackFuncAudioStreamFromRos
{
public:
  deque<hark_msgs::HarkWave> deque_wave;
  int data_buffer_num;  
  int init_flag;
  bool debug_print;
  callbackFuncAudioStreamFromRos(int buffer, bool debug){
    data_buffer_num = buffer;
    init_flag = 1;
    debug_print = debug;
    deque_wave.resize(0);
  }
  void cb(const hark_msgs::HarkWave::ConstPtr& msg)
  {

    if (debug_print == true) 
      ROS_INFO_STREAM("Received [" << msg->count << "] [thread=" << boost::this_thread::get_id() << "]");
        
    hark_msgs::HarkWave current_wave;
    current_wave.header.frame_id = msg->header.frame_id;
    current_wave.header.stamp = msg->header.stamp;
    current_wave.count      = (int)msg->count;
    current_wave.nch        = (int)msg->nch;
    current_wave.length     = (int)msg->length;
    current_wave.data_bytes = (int)msg->data_bytes;       
    for(int k = 0 ; k < (int)msg->nch ; k++){
      hark_msgs::HarkWaveVal current_wave_val;
      current_wave_val.wavedata.resize((int)msg->length);
      for (int i = 0; i < (int)msg->length; i++) {
	current_wave_val.wavedata[i] = (float)msg->src[k].wavedata[i];
      }
      current_wave.src.push_back(current_wave_val);
    }
    
    if(deque_wave.size() < data_buffer_num){
      deque_wave.push_back(current_wave);
    }else{
      deque_wave.pop_front();
      deque_wave.push_back(current_wave);      
    }      
    
    init_flag = 0;
    
  }
};


class AudioStreamFromRos;

DECLARE_NODE(AudioStreamFromRos);
/*Node
 *
 * @name AudioStreamFromRos
 * @category HARK:ROS:IO
 * @description This takes an audio stream of HarkWave from rosbag file or topics without any packet loss.
 *
 * @output_name OUTPUT
 * @output_type Matrix<float>
 * @output_description  This outputs audio stream of multiple microphones. This is a 2D matrix. The row denotes indices of microphones.  The column denotes wave samples. 
 *
 * @output_name TIMESTAMP
 * @output_type TimeStamp
 * @output_description Time stamp of the subscribed messages
 *
 * @output_name NOT_EOF
 * @output_type bool
 * @output_description True unless End-of-File
 * 
 * @parameter_name TOPIC_NAME
 * @parameter_type string
 * @parameter_value HarkWave
 * @parameter_description Subscribed topic name for ROS
 * 
 * @parameter_name ROS_LOOP_RATE
 * @parameter_type float
 * @parameter_value 1000000
 * @parameter_description This allows you to specify a frequency that you would like to loop at [Hz]. Keep this value large. (If ROS interval is shorter than HARK interval, ROS interval is overwritten.)
 * 
 * @parameter_name MSG_BUFFER_NUM
 * @parameter_type int
 * @parameter_value 100
 * @parameter_description Buffer size for a ROS subscribed message.
 * 
 * @parameter_name DATA_BUFFER_NUM
 * @parameter_type int
 * @parameter_value 100
 * @parameter_description Buffer save size for a HARK output. Too small value makes packet loss. Too large value takes large memory.
 * 
 * @parameter_name KILL_TIME_LEN
 * @parameter_type int
 * @parameter_value 900000
 * @parameter_description Kill time length for this network [usec] (must be less than 10[sec]). Set KILL_NETWORK enable.
 *
 * @parameter_name KILL_NETWORK
 * @parameter_type bool
 * @parameter_value true
 * @parameter_list true:false
 * @parameter_description Kill whole hark network when this node does not take any message from ROS.
 *
 * @parameter_name DEBUG_PRINT
 * @parameter_type bool
 * @parameter_value false
 * @parameter_list true:false
 * @parameter_description print debug message of this module in case of true.
 *
END*/

bool exit_flag_AudioStreamFromRos = false;

void signal_handler_AudioStreamFromRos(int s)
{
  exit_flag_AudioStreamFromRos = true;
}

class AudioStreamFromRos : public BufferedNode {

  int outputID;
  int timestampID;
  int eofID;

  int nb_channels;
  int length;
  
  string topic_name;
  float ros_loop_rate;
  int msg_buffer_num;  
  int data_buffer_num;
  bool debug_print;
  int kill_time_len;  
  bool kill_network;

  callbackFuncAudioStreamFromRos *cbf;

  struct timeval tv_base;
  struct timeval tv_tmp;

private:
  ros::Subscriber _sub;
  ros::AsyncSpinner *_snum;

public:
  AudioStreamFromRos(string nodeName, ParameterSet params)
    : BufferedNode(nodeName, params)
  {
    timestampID = addOutput("TIMESTAMP");
    outputID   = addOutput("OUTPUT");
    eofID = addOutput("NOT_EOF");

    topic_name  = object_cast<String>(parameters.get("TOPIC_NAME"));
    ros_loop_rate = dereference_cast<float>(parameters.get("ROS_LOOP_RATE"));
    msg_buffer_num = dereference_cast<int>(parameters.get("MSG_BUFFER_NUM"));
    data_buffer_num = dereference_cast<int>(parameters.get("DATA_BUFFER_NUM"));
    debug_print = dereference_cast<bool>(parameters.get("DEBUG_PRINT"));
    kill_time_len  = dereference_cast<int>(parameters.get("KILL_TIME_LEN"));
    kill_network = dereference_cast<bool>(parameters.get("KILL_NETWORK"));

    signal(SIGINT, signal_handler_AudioStreamFromRos);
    signal(SIGHUP, signal_handler_AudioStreamFromRos);

    inOrder = true;

    cbf = new callbackFuncAudioStreamFromRos(data_buffer_num, debug_print);

    nb_channels = 0;
    length = 0;

    cout << getName() << " constructor end..." << endl;
  }

  ~AudioStreamFromRos()
  {
    _snum->stop();
  }

  virtual void initialize()
  {
    cout << getName() << " initialized..." << endl;
    outputs[outputID].lookAhead = outputs[eofID].lookAhead = outputs[timestampID].lookAhead = 1+max(max(outputs[outputID].lookAhead, outputs[eofID].lookAhead), outputs[timestampID].lookAhead);
    outputs[outputID].lookBack  = outputs[eofID].lookBack  = outputs[timestampID].lookBack  = 1+max(max(outputs[outputID].lookBack,  outputs[eofID].lookBack),  outputs[timestampID].lookBack);
    //outputs[audioID].lookBack += 1;
    this->BufferedNode::initialize();
    _sub = __MasterRosNodeHandler__->subscribe<hark_msgs::HarkWave>(topic_name, msg_buffer_num, &callbackFuncAudioStreamFromRos::cb, cbf);
    _snum = new ros::AsyncSpinner(1);
    _snum->start();
  }

  void calculate(int output_id, int count, Buffer &out)
  {

    RCPtr<TimeStamp> cur_timestamp(new TimeStamp());
    (*outputs[timestampID].buffer)[count] = cur_timestamp;

    Buffer &eofBuffer = *(outputs[eofID].buffer);
    eofBuffer[count] = TrueObject;

    ros::Rate loop_rate(ros_loop_rate);

    while(cbf->init_flag){
      if(exit_flag_AudioStreamFromRos)
	throw new NodeException(NULL, getName()+string(" Force-quit..."), __FILE__, __LINE__);
      usleep(1);
    }
    
    gettimeofday(&tv_base, NULL);
    long int usec_base = tv_base.tv_usec;
    long int sec_base  = tv_base.tv_sec % 10;
    long int sec_cnt_base = sec_base * 1000000 + usec_base;
    
    while(cbf->deque_wave.size()==0){
      gettimeofday(&tv_tmp, NULL);
      long int usec_tmp = tv_tmp.tv_usec;
      long int sec_tmp  = tv_tmp.tv_sec % 10;
      if(sec_tmp < sec_base){sec_tmp += 10;}      
      long int sec_cnt_tmp = sec_tmp * 1000000 + usec_tmp;
      usleep(1);
      // If there has been no input from ROS for more than [kill_time_len]usec, kill this node
      if((sec_cnt_tmp - sec_cnt_base > kill_time_len)&&(kill_network)){
	eofBuffer[count] = FalseObject;
	break;
      }      
      if(exit_flag_AudioStreamFromRos)
	throw new NodeException(NULL, getName()+string(" Force-quit..."), __FILE__, __LINE__);
    }
      
    length = cbf->deque_wave[0].length;
    nb_channels = cbf->deque_wave[0].nch;
    
    RCPtr<Matrix<float> > output(new Matrix<float>(nb_channels, length));    
    (*(outputs[outputID].buffer))[count] = output;

    if(cbf->deque_wave.size()){

      cur_timestamp->setTime((long int)cbf->deque_wave[0].header.stamp.sec, (long int)cbf->deque_wave[0].header.stamp.nsec);
      
      for (int k = 0; k < nb_channels; k++) {
	for (int i = 0; i < length; i++) {
	  (*output)(k, i) = cbf->deque_wave[0].src[k].wavedata[i];
	}
      }
      
      loop_rate.sleep();
      
      if (debug_print == true) cout    << count << " : " << cbf->deque_wave.size() << " : " << cbf->deque_wave[0].count << " : " << cbf->deque_wave[0].nch << endl;
      
      cbf->deque_wave.pop_front();    
    }

  }

  //IN_ORDER_NODE_SPEEDUP(AudioStreamFromRos);
};

#endif
