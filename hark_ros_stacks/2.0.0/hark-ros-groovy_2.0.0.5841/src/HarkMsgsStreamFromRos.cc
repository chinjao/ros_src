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

#include <iostream>
#include <BufferedNode.h>
#include <Buffer.h>
#include "Vector.h"
#include "Map.h"
#include "Source.h"
#include "Matrix.h"
#include <../config.h>
#include "TimeStamp.h"
#include <csignal>

#ifdef ENABLE_ROS

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <boost/thread.hpp>
#include "hark_msgs/HarkFeature.h"
#include "hark_msgs/HarkFeatureVal.h"
#include "hark_msgs/HarkSource.h"
#include "hark_msgs/HarkSourceVal.h"
#include "hark_msgs/HarkSrcFeature.h"
#include "hark_msgs/HarkSrcFeatureVal.h"
#include "hark_msgs/HarkSrcFeatureMFM.h"
#include "hark_msgs/HarkSrcFeatureMFMVal.h"
#include "hark_msgs/HarkSrcWave.h"
#include "hark_msgs/HarkSrcWaveVal.h"
#include "hark_msgs/HarkSrcFFT.h"
#include "hark_msgs/HarkSrcFFTVal.h"
#include "hark_msgs/HarkWave.h"
#include "hark_msgs/HarkWaveVal.h"
#include "hark_msgs/HarkFFT.h"
#include "hark_msgs/HarkFFTVal.h"
#include "HarkRosGlobals.h"
#include <math.h>
#include <vector>
#include <sys/time.h>
#include <queue>

using namespace std;
using namespace FD;


class callbackFuncHarkMsgsStreamFromRos
{

public:
  deque<hark_msgs::HarkFeature>       deque_harkfeature;
  deque<hark_msgs::HarkSource>        deque_harksource;
  deque<hark_msgs::HarkSrcFeature>    deque_harksrcfeature;
  deque<hark_msgs::HarkSrcFeatureMFM> deque_harksrcfeaturemfm;
  deque<hark_msgs::HarkSrcWave>       deque_harksrcwave;
  deque<hark_msgs::HarkSrcFFT>        deque_harksrcfft;
  deque<hark_msgs::HarkWave>          deque_harkwave;
  deque<hark_msgs::HarkFFT>           deque_harkfft;
  bool enable_debug;
  int init_flag;
  int data_buffer_num;  

  callbackFuncHarkMsgsStreamFromRos(int buffer, bool debug){
    data_buffer_num = buffer;
    init_flag = 1;
    enable_debug = debug;
    deque_harkfeature.resize(0);
    deque_harksource.resize(0);
    deque_harksrcfeature.resize(0);
    deque_harksrcfeaturemfm.resize(0);
    deque_harksrcwave.resize(0);
    deque_harksrcfft.resize(0);
    deque_harkwave.resize(0);
    deque_harkfft.resize(0);
  }

  void cb_harkfeature(const hark_msgs::HarkFeature::ConstPtr& msg)
  {
    if(enable_debug)
      ROS_INFO_STREAM("HarkFeature received [" << msg->count << "] [thread=" << boost::this_thread::get_id() << "]");
    hark_msgs::HarkFeature value_harkfeature;
    value_harkfeature.header.frame_id = msg->header.frame_id;
    value_harkfeature.header.stamp = msg->header.stamp;
    value_harkfeature.count = msg->count;
    value_harkfeature.exist_src_num = msg->exist_src_num;
    value_harkfeature.src.resize(0);
    for(int i = 0; i < msg->exist_src_num; i++){
      hark_msgs::HarkFeatureVal HarkFeatureValMsg;
      HarkFeatureValMsg.id         = msg->src[i].id;
      HarkFeatureValMsg.length     = msg->src[i].length;
      HarkFeatureValMsg.data_bytes = msg->src[i].data_bytes;
      HarkFeatureValMsg.featuredata.resize(msg->src[i].featuredata.size());
      for(int j = 0; j < msg->src[i].featuredata.size(); j++){
	HarkFeatureValMsg.featuredata[j] = msg->src[i].featuredata[j];
      }
      value_harkfeature.src.push_back(HarkFeatureValMsg);
    }

    if(deque_harkfeature.size() < data_buffer_num){
      deque_harkfeature.push_back(value_harkfeature);
    }else{
      deque_harkfeature.pop_front();
      deque_harkfeature.push_back(value_harkfeature);      
    }

    init_flag = 0;

  }

  void cb_harksource(const hark_msgs::HarkSource::ConstPtr& msg)
  {
    if(enable_debug)
      ROS_INFO_STREAM("HarkSource received [" << msg->count << "] [thread=" << boost::this_thread::get_id() << "]");
    hark_msgs::HarkSource value_harksource;
    value_harksource.header.frame_id = msg->header.frame_id;
    value_harksource.header.stamp = msg->header.stamp;
    value_harksource.count = msg->count;
    value_harksource.exist_src_num = msg->exist_src_num;
    value_harksource.src.resize(0);
    for(int i = 0; i < msg->exist_src_num; i++){
      hark_msgs::HarkSourceVal HarkSourceValMsg;
      HarkSourceValMsg.id    = msg->src[i].id;
      HarkSourceValMsg.power = msg->src[i].power;
      HarkSourceValMsg.x     = msg->src[i].x;
      HarkSourceValMsg.y     = msg->src[i].y;
      HarkSourceValMsg.z     = msg->src[i].z;
      HarkSourceValMsg.theta = msg->src[i].theta; 
      value_harksource.src.push_back(HarkSourceValMsg);
    }

    if(deque_harksource.size() < data_buffer_num){
      deque_harksource.push_back(value_harksource);
    }else{
      deque_harksource.pop_front();
      deque_harksource.push_back(value_harksource);      
    }

    init_flag = 0;

  }

  void cb_harksrcfeature(const hark_msgs::HarkSrcFeature::ConstPtr& msg)
  {
    if(enable_debug)
      ROS_INFO_STREAM("HarkSrcFeature received [" << msg->count << "] [thread=" << boost::this_thread::get_id() << "]");
    hark_msgs::HarkSrcFeature value_harksrcfeature;
    value_harksrcfeature.header.frame_id = msg->header.frame_id;
    value_harksrcfeature.header.stamp = msg->header.stamp;
    value_harksrcfeature.count = msg->count;
    value_harksrcfeature.exist_src_num = msg->exist_src_num;
    value_harksrcfeature.src.resize(0);
    for(int i = 0; i < msg->exist_src_num; i++){
      hark_msgs::HarkSrcFeatureVal HarkSrcFeatureValMsg;      
      HarkSrcFeatureValMsg.id    = msg->src[i].id;
      HarkSrcFeatureValMsg.power = msg->src[i].power;
      HarkSrcFeatureValMsg.x     = msg->src[i].x;
      HarkSrcFeatureValMsg.y     = msg->src[i].y;
      HarkSrcFeatureValMsg.z     = msg->src[i].z;
      HarkSrcFeatureValMsg.theta = msg->src[i].theta;
      HarkSrcFeatureValMsg.length     = msg->src[i].length;
      HarkSrcFeatureValMsg.data_bytes = msg->src[i].data_bytes;
      HarkSrcFeatureValMsg.featuredata.resize(msg->src[i].featuredata.size());
      for(int j = 0; j < msg->src[i].featuredata.size(); j++){
	HarkSrcFeatureValMsg.featuredata[j] = msg->src[i].featuredata[j];
      }
      value_harksrcfeature.src.push_back(HarkSrcFeatureValMsg);
    }

    if(deque_harksrcfeature.size() < data_buffer_num){
      deque_harksrcfeature.push_back(value_harksrcfeature);
    }else{
      deque_harksrcfeature.pop_front();
      deque_harksrcfeature.push_back(value_harksrcfeature);      
    }

    init_flag = 0;

  }

  void cb_harksrcfeaturemfm(const hark_msgs::HarkSrcFeatureMFM::ConstPtr& msg)
  {
    if(enable_debug)
      ROS_INFO_STREAM("HarkSrcFeatureMFM received [" << msg->count << "] [thread=" << boost::this_thread::get_id() << "]");
    hark_msgs::HarkSrcFeatureMFM value_harksrcfeaturemfm;
    value_harksrcfeaturemfm.header.frame_id = msg->header.frame_id;
    value_harksrcfeaturemfm.header.stamp = msg->header.stamp;
    value_harksrcfeaturemfm.count = msg->count;
    value_harksrcfeaturemfm.exist_src_num = msg->exist_src_num;
    value_harksrcfeaturemfm.src.resize(0);
    for(int i = 0; i < msg->exist_src_num; i++){
      hark_msgs::HarkSrcFeatureMFMVal HarkSrcFeatureMFMValMsg;
      HarkSrcFeatureMFMValMsg.id    = msg->src[i].id;
      HarkSrcFeatureMFMValMsg.power = msg->src[i].power;
      HarkSrcFeatureMFMValMsg.x     = msg->src[i].x;
      HarkSrcFeatureMFMValMsg.y     = msg->src[i].y;
      HarkSrcFeatureMFMValMsg.z     = msg->src[i].z;
      HarkSrcFeatureMFMValMsg.theta = msg->src[i].theta;
      HarkSrcFeatureMFMValMsg.length_feature = msg->src[i].length_feature;
      HarkSrcFeatureMFMValMsg.data_bytes_feature = msg->src[i].data_bytes_feature;
      HarkSrcFeatureMFMValMsg.length_mfm = msg->src[i].length_mfm;
      HarkSrcFeatureMFMValMsg.data_bytes_mfm = msg->src[i].data_bytes_mfm;
      HarkSrcFeatureMFMValMsg.featuredata_feature.resize(msg->src[i].featuredata_feature.size());
      for(int j = 0; j < msg->src[i].featuredata_feature.size(); j++){
	HarkSrcFeatureMFMValMsg.featuredata_feature[j] = msg->src[i].featuredata_feature[j];
      }
      HarkSrcFeatureMFMValMsg.featuredata_mfm.resize(msg->src[i].featuredata_mfm.size());
      for(int j = 0; j < msg->src[i].featuredata_mfm.size(); j++){
	HarkSrcFeatureMFMValMsg.featuredata_mfm[j] = msg->src[i].featuredata_mfm[j];
      }
      value_harksrcfeaturemfm.src.push_back(HarkSrcFeatureMFMValMsg);
    }

    if(deque_harksrcfeaturemfm.size() < data_buffer_num){
      deque_harksrcfeaturemfm.push_back(value_harksrcfeaturemfm);
    }else{
      deque_harksrcfeaturemfm.pop_front();
      deque_harksrcfeaturemfm.push_back(value_harksrcfeaturemfm);      
    }

    init_flag = 0;

  }

  void cb_harksrcwave(const hark_msgs::HarkSrcWave::ConstPtr& msg)
  {
    if(enable_debug)
      ROS_INFO_STREAM("HarkSrcWave received [" << msg->count << "] [thread=" << boost::this_thread::get_id() << "]");
    hark_msgs::HarkSrcWave value_harksrcwave;
    value_harksrcwave.header.frame_id = msg->header.frame_id;
    value_harksrcwave.header.stamp = msg->header.stamp;
    value_harksrcwave.count = msg->count;
    value_harksrcwave.exist_src_num = msg->exist_src_num;
    value_harksrcwave.src.resize(0);
    for(int i = 0; i < msg->exist_src_num; i++){
      hark_msgs::HarkSrcWaveVal HarkSrcWaveValMsg;
      HarkSrcWaveValMsg.id     = msg->src[i].id;
      HarkSrcWaveValMsg.power  = msg->src[i].power;
      HarkSrcWaveValMsg.x      = msg->src[i].x;
      HarkSrcWaveValMsg.y      = msg->src[i].y;
      HarkSrcWaveValMsg.z      = msg->src[i].z;
      HarkSrcWaveValMsg.theta  = msg->src[i].theta;
      HarkSrcWaveValMsg.length = msg->src[i].length;
      HarkSrcWaveValMsg.data_bytes = msg->src[i].data_bytes;
      HarkSrcWaveValMsg.wavedata.resize(msg->src[i].wavedata.size());
      for(int j = 0; j < msg->src[i].wavedata.size(); j++){
        HarkSrcWaveValMsg.wavedata[j] = msg->src[i].wavedata[j];
      }
      value_harksrcwave.src.push_back(HarkSrcWaveValMsg);
    }

    if(deque_harksrcwave.size() < data_buffer_num){
      deque_harksrcwave.push_back(value_harksrcwave);
    }else{
      deque_harksrcwave.pop_front();
      deque_harksrcwave.push_back(value_harksrcwave);      
    }

    init_flag = 0;

  }

  void cb_harksrcfft(const hark_msgs::HarkSrcFFT::ConstPtr& msg)
  {
    if(enable_debug)
      ROS_INFO_STREAM("HarkSrcFFT received [" << msg->count << "] [thread=" << boost::this_thread::get_id() << "]");
    hark_msgs::HarkSrcFFT value_harksrcfft;
    value_harksrcfft.header.frame_id = msg->header.frame_id;
    value_harksrcfft.header.stamp = msg->header.stamp;
    value_harksrcfft.count = msg->count;
    value_harksrcfft.exist_src_num = msg->exist_src_num;
    value_harksrcfft.src.resize(0);
    for(int i = 0; i < msg->exist_src_num; i++){
      hark_msgs::HarkSrcFFTVal HarkSrcFFTValMsg;
      HarkSrcFFTValMsg.id    = msg->src[i].id;
      HarkSrcFFTValMsg.power = msg->src[i].power;
      HarkSrcFFTValMsg.x     = msg->src[i].x;
      HarkSrcFFTValMsg.y     = msg->src[i].y;
      HarkSrcFFTValMsg.z     = msg->src[i].z;
      HarkSrcFFTValMsg.theta = msg->src[i].theta;
      HarkSrcFFTValMsg.length = msg->src[i].length;
      HarkSrcFFTValMsg.fftdata_real.resize(msg->src[i].fftdata_real.size());
      for(int j = 0; j < msg->src[i].fftdata_real.size(); j++){
	HarkSrcFFTValMsg.fftdata_real[j] = msg->src[i].fftdata_real[j];
      }
      HarkSrcFFTValMsg.fftdata_imag.resize(msg->src[i].fftdata_imag.size());
      for(int j = 0; j < msg->src[i].fftdata_imag.size(); j++){
	HarkSrcFFTValMsg.fftdata_imag[j] = msg->src[i].fftdata_imag[j];
      }
      value_harksrcfft.src.push_back(HarkSrcFFTValMsg);
    }

    if(deque_harksrcfft.size() < data_buffer_num){
      deque_harksrcfft.push_back(value_harksrcfft);
    }else{
      deque_harksrcfft.pop_front();
      deque_harksrcfft.push_back(value_harksrcfft);      
    }

    init_flag = 0;

  }

  void cb_harkwave(const hark_msgs::HarkWave::ConstPtr& msg)
  {
    if(enable_debug)
      ROS_INFO_STREAM("HarkWave received [" << msg->count << "] [thread=" << boost::this_thread::get_id() << "]");
    hark_msgs::HarkWave value_harkwave;
    value_harkwave.header.frame_id = msg->header.frame_id;
    value_harkwave.header.stamp = msg->header.stamp;
    value_harkwave.count = msg->count;
    value_harkwave.nch = msg->nch;
    value_harkwave.length = msg->length;
    value_harkwave.data_bytes = msg->data_bytes;
    value_harkwave.src.resize(0);
    for(int i = 0; i < msg->src.size(); i++){
      hark_msgs::HarkWaveVal HarkWaveValMsg;
      HarkWaveValMsg.wavedata.resize(msg->src[i].wavedata.size());
      for(int j = 0; j < msg->src[i].wavedata.size(); j++){
	HarkWaveValMsg.wavedata[j] = msg->src[i].wavedata[j];
      }
      value_harkwave.src.push_back(HarkWaveValMsg);
    }

    if(deque_harkwave.size() < data_buffer_num){
      deque_harkwave.push_back(value_harkwave);
    }else{
      deque_harkwave.pop_front();
      deque_harkwave.push_back(value_harkwave);
    }

    init_flag = 0;

  }

  void cb_harkfft(const hark_msgs::HarkFFT::ConstPtr& msg)
  {
    if(enable_debug)
      ROS_INFO_STREAM("HarkFFT received [" << msg->count << "] [thread=" << boost::this_thread::get_id() << "]");
    hark_msgs::HarkFFT value_harkfft;
    value_harkfft.header.frame_id = msg->header.frame_id;
    value_harkfft.header.stamp = msg->header.stamp;
    value_harkfft.count = msg->count;
    value_harkfft.nch = msg->nch;
    value_harkfft.length = msg->length;
    value_harkfft.src.resize(0);
    for(int i = 0; i < msg->src.size(); i++){
      hark_msgs::HarkFFTVal HarkFFTValMsg;
      HarkFFTValMsg.fftdata_real.resize(msg->src[i].fftdata_real.size());
      for(int j = 0; j < msg->src[i].fftdata_real.size(); j++){
	HarkFFTValMsg.fftdata_real[j] = msg->src[i].fftdata_real[j];
      }
      HarkFFTValMsg.fftdata_imag.resize(msg->src[i].fftdata_imag.size());
      for(int j = 0; j < msg->src[i].fftdata_imag.size(); j++){
	HarkFFTValMsg.fftdata_imag[j] = msg->src[i].fftdata_imag[j];
      }
      value_harkfft.src.push_back(HarkFFTValMsg);
    }

    if(deque_harkfft.size() < data_buffer_num){
      deque_harkfft.push_back(value_harkfft);
    }else{
      deque_harkfft.pop_front();
      deque_harkfft.push_back(value_harkfft);
    }

    init_flag = 0;

  }

};


class HarkMsgsStreamFromRos;

DECLARE_NODE(HarkMsgsStreamFromRos);
/*Node
 *
 * @name HarkMsgsStreamFromRos
 * @category HARK:ROS:IO
 * @description ROS msg subscriber node for hark common messages (hark_msgs) without any packet loss. Use this module like AudioStreamFromWave. 
 * 
 * @parameter_name STREAM_SELECTION
 * @parameter_type string
 * @parameter_value HarkWave
 * @parameter_list HarkWave:HarkFFT:HarkFeature:HarkSource:HarkSrcWave:HarkSrcFFT:HarkSrcFeature:HarkSrcFeatureMFM
 * @parameter_description Selector of stream you want to realize. All other subscription will be disabled.
 * 
 * @parameter_name TOPIC_NAME_HARKWAVE
 * @parameter_type string
 * @parameter_value HarkWave
 * @parameter_valid STREAM_SELECTION=HarkWave
 * @parameter_description Subscribed topic name for ROS (HarkWave type message)
 * 
 * @parameter_name TOPIC_NAME_HARKFFT
 * @parameter_type string
 * @parameter_value HarkFFT
 * @parameter_valid STREAM_SELECTION=HarkFFT
 * @parameter_description Subscribed topic name for ROS (HarkFFT type message)
 * 
 * @parameter_name TOPIC_NAME_HARKFEATURE
 * @parameter_type string
 * @parameter_value HarkFeature
 * @parameter_valid STREAM_SELECTION=HarkFeature
 * @parameter_description Subscribed topic name for ROS (HarkFeature type message)
 * 
 * @parameter_name TOPIC_NAME_HARKSOURCE
 * @parameter_type string
 * @parameter_value HarkSource
 * @parameter_valid STREAM_SELECTION=HarkSource
 * @parameter_description Subscribed topic name for ROS (HarkSource type message)
 * 
 * @parameter_name TOPIC_NAME_HARKSRCWAVE
 * @parameter_type string
 * @parameter_value HarkSrcWave
 * @parameter_valid STREAM_SELECTION=HarkSrcWave
 * @parameter_description Subscribed topic name for ROS (HarkSrcWave type message)
 * 
 * @parameter_name TOPIC_NAME_HARKSRCFFT
 * @parameter_type string
 * @parameter_value HarkSrcFFT
 * @parameter_valid STREAM_SELECTION=HarkSrcFFT
 * @parameter_description Subscribed topic name for ROS (HarkSrcFFT type message)
 * 
 * @parameter_name TOPIC_NAME_HARKSRCFEATURE
 * @parameter_type string
 * @parameter_value HarkSrcFeature
 * @parameter_valid STREAM_SELECTION=HarkSrcFeature
 * @parameter_description Subscribed topic name for ROS (HarkSrcFeature type message)
 * 
 * @parameter_name TOPIC_NAME_HARKSRCFEATUREMFM
 * @parameter_type string
 * @parameter_value HarkSrcFeatureMFM
 * @parameter_valid STREAM_SELECTION=HarkSrcFeatureMFM
 * @parameter_description Subscribed topic name for ROS (HarkSrcFeatureMFM type message)
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
 * @parameter_name ENABLE_DEBUG
 * @parameter_type bool
 * @parameter_value false
 * @parameter_list true:false
 * @parameter_description print debug message of this module in case of true.
 *
 * @output_name MATOUT
 * @output_type any
 * @output_description Matrix output port when STREAM_SELECTION=HarkWave or HarkFFT 
 *
 * @output_name SRCOUT
 * @output_type Vector<ObjectRef>
 * @output_description Source output port when STREAM_SELECTION=HarkSource or HarkSrcWave or HarkSrcFFT or HarkSrcFeature or HarkSrcFeatureMFM 
 *
 * @output_name MAPOUT
 * @output_type Map<int,ObjectRef>
 * @output_description Map output port when STREAM_SELECTION=HarkFeature or HarkSrcWave or HarkSrcFFT or HarkSrcFeature or HarkSrcFeatureMFM 
 *
 * @output_name TIMESTAMP
 * @output_type TimeStamp
 * @output_description Time stamp of the subscribed messages
 *
 * @output_name NOT_EOF
 * @output_type bool
 * @output_description True unless End-of-File
 *
END*/

bool exit_flag_HarkMsgsStreamFromRos = false;

void signal_handler_HarkMsgsStreamFromRos(int s)
{
  exit_flag_HarkMsgsStreamFromRos = true;
}

class HarkMsgsStreamFromRos : public BufferedNode {

  int matoutID;
  int srcoutID;
  int mapoutID;
  int timestampID;
  int eofID;

  bool bmatoutID; 
  bool bsrcoutID; 
  bool bmapoutID; 

  string topic_name_harkwave;
  string topic_name_harkfft;
  string topic_name_harkfeature;
  string topic_name_harksource;
  string topic_name_harksrcwave;
  string topic_name_harksrcfft;
  string topic_name_harksrcfeature;
  string topic_name_harksrcfeaturemfm;

  string stream_selection;
  bool enable_debug;
  int msg_buffer_num;  
  int data_buffer_num;
  float ros_loop_rate;
  int kill_time_len;  
  bool kill_network;
  int output_exception_flag;
  
  callbackFuncHarkMsgsStreamFromRos *cbf;

  struct timeval tv_base;
  struct timeval tv_tmp;

private:
  ros::Subscriber _sub_harkwave;
  ros::Subscriber _sub_harkfft;
  ros::Subscriber _sub_harkfeature;
  ros::Subscriber _sub_harksource;
  ros::Subscriber _sub_harksrcwave;
  ros::Subscriber _sub_harksrcfft;
  ros::Subscriber _sub_harksrcfeature;
  ros::Subscriber _sub_harksrcfeaturemfm;
  ros::AsyncSpinner *_snum;
  
public:
  HarkMsgsStreamFromRos(string nodeName, ParameterSet params)
    : BufferedNode(nodeName, params), 
      matoutID(-1), srcoutID(-1), mapoutID(-1),
      bmatoutID(false), bsrcoutID(false), bmapoutID(false)
  {
    timestampID = addOutput("TIMESTAMP");
    eofID = addOutput("NOT_EOF");

    topic_name_harkwave           = object_cast<String>(parameters.get("TOPIC_NAME_HARKWAVE"));
    topic_name_harkfft            = object_cast<String>(parameters.get("TOPIC_NAME_HARKFFT"));
    topic_name_harkfeature        = object_cast<String>(parameters.get("TOPIC_NAME_HARKFEATURE"));
    topic_name_harksource         = object_cast<String>(parameters.get("TOPIC_NAME_HARKSOURCE"));
    topic_name_harksrcwave        = object_cast<String>(parameters.get("TOPIC_NAME_HARKSRCWAVE"));
    topic_name_harksrcfft         = object_cast<String>(parameters.get("TOPIC_NAME_HARKSRCFFT"));
    topic_name_harksrcfeature     = object_cast<String>(parameters.get("TOPIC_NAME_HARKSRCFEATURE"));
    topic_name_harksrcfeaturemfm  = object_cast<String>(parameters.get("TOPIC_NAME_HARKSRCFEATUREMFM"));

    stream_selection = object_cast<String>(parameters.get("STREAM_SELECTION"));
    enable_debug = dereference_cast<bool>(parameters.get("ENABLE_DEBUG"));
    ros_loop_rate = dereference_cast<float>(parameters.get("ROS_LOOP_RATE"));
    msg_buffer_num = dereference_cast<int>(parameters.get("MSG_BUFFER_NUM"));
    data_buffer_num = dereference_cast<int>(parameters.get("DATA_BUFFER_NUM"));
    kill_time_len  = dereference_cast<int>(parameters.get("KILL_TIME_LEN"));
    kill_network = dereference_cast<bool>(parameters.get("KILL_NETWORK"));
    output_exception_flag = 0;

    signal(SIGINT, signal_handler_HarkMsgsStreamFromRos);
    signal(SIGHUP, signal_handler_HarkMsgsStreamFromRos);

    cbf = new callbackFuncHarkMsgsStreamFromRos(data_buffer_num, enable_debug);
    inOrder = true;
    cout << getName() << " constructor end..." << endl;
  }

  ~HarkMsgsStreamFromRos()
  {
    _snum->stop();
  }

  virtual void initialize()
  {
    cout << getName() << " initialized..." << endl;

    if(stream_selection == "HarkWave"){
      _sub_harkwave          = __MasterRosNodeHandler__->subscribe<hark_msgs::HarkWave>(topic_name_harkwave, msg_buffer_num, &callbackFuncHarkMsgsStreamFromRos::cb_harkwave, cbf);
      outputs[matoutID].lookAhead = outputs[eofID].lookAhead = outputs[timestampID].lookAhead = 1+max(max(outputs[matoutID].lookAhead, outputs[eofID].lookAhead), outputs[timestampID].lookAhead);
      outputs[matoutID].lookBack  = outputs[eofID].lookBack  = outputs[timestampID].lookBack  = 1+max(max(outputs[matoutID].lookBack,  outputs[eofID].lookBack),  outputs[timestampID].lookBack);
      if((srcoutID != -1)||(mapoutID != -1)) output_exception_flag = 1;
    }
    if(stream_selection == "HarkFFT"){    
      _sub_harkfft           = __MasterRosNodeHandler__->subscribe<hark_msgs::HarkFFT>(topic_name_harkfft, msg_buffer_num, &callbackFuncHarkMsgsStreamFromRos::cb_harkfft, cbf);
      outputs[matoutID].lookAhead = outputs[eofID].lookAhead = outputs[timestampID].lookAhead = 1+max(max(outputs[matoutID].lookAhead, outputs[eofID].lookAhead), outputs[timestampID].lookAhead);
      outputs[matoutID].lookBack  = outputs[eofID].lookBack  = outputs[timestampID].lookBack  = 1+max(max(outputs[matoutID].lookBack,  outputs[eofID].lookBack),  outputs[timestampID].lookBack);
      if((srcoutID != -1)||(mapoutID != -1)) output_exception_flag = 1;
    }
    if(stream_selection == "HarkFeature"){
      _sub_harkfeature       = __MasterRosNodeHandler__->subscribe<hark_msgs::HarkFeature>(topic_name_harkfeature, msg_buffer_num, &callbackFuncHarkMsgsStreamFromRos::cb_harkfeature, cbf);
      outputs[mapoutID].lookAhead = outputs[eofID].lookAhead = outputs[timestampID].lookAhead = 1+max(max(outputs[mapoutID].lookAhead, outputs[eofID].lookAhead), outputs[timestampID].lookAhead);
      outputs[mapoutID].lookBack  = outputs[eofID].lookBack  = outputs[timestampID].lookBack  = 1+max(max(outputs[mapoutID].lookBack,  outputs[eofID].lookBack),  outputs[timestampID].lookBack);
      if((srcoutID != -1)||(matoutID != -1)) output_exception_flag = 1;
    }
    if(stream_selection == "HarkSource"){
      _sub_harksource        = __MasterRosNodeHandler__->subscribe<hark_msgs::HarkSource>(topic_name_harksource, msg_buffer_num, &callbackFuncHarkMsgsStreamFromRos::cb_harksource, cbf);
      outputs[srcoutID].lookAhead = outputs[eofID].lookAhead = outputs[timestampID].lookAhead = 1+max(max(outputs[srcoutID].lookAhead, outputs[eofID].lookAhead), outputs[timestampID].lookAhead);
      outputs[srcoutID].lookBack  = outputs[eofID].lookBack  = outputs[timestampID].lookBack  = 1+max(max(outputs[srcoutID].lookBack,  outputs[eofID].lookBack),  outputs[timestampID].lookBack);
      if((matoutID != -1)||(mapoutID != -1)) output_exception_flag = 1;
    }
    if(stream_selection == "HarkSrcWave"){
      _sub_harksrcwave       = __MasterRosNodeHandler__->subscribe<hark_msgs::HarkSrcWave>(topic_name_harksrcwave, msg_buffer_num, &callbackFuncHarkMsgsStreamFromRos::cb_harksrcwave, cbf);
      outputs[mapoutID].lookAhead = outputs[srcoutID].lookAhead = outputs[eofID].lookAhead = outputs[timestampID].lookAhead = 1+max(max(max(outputs[mapoutID].lookAhead, outputs[srcoutID].lookAhead), outputs[eofID].lookAhead), outputs[timestampID].lookAhead);
      outputs[mapoutID].lookBack  = outputs[srcoutID].lookBack  = outputs[eofID].lookBack  = outputs[timestampID].lookBack  = 1+max(max(max(outputs[mapoutID].lookBack,  outputs[srcoutID].lookBack),  outputs[eofID].lookBack),  outputs[timestampID].lookBack);
      if(matoutID != -1) output_exception_flag = 1;
    }
    if(stream_selection == "HarkSrcFFT"){
      _sub_harksrcfft        = __MasterRosNodeHandler__->subscribe<hark_msgs::HarkSrcFFT>(topic_name_harksrcfft, msg_buffer_num, &callbackFuncHarkMsgsStreamFromRos::cb_harksrcfft, cbf);
      outputs[mapoutID].lookAhead = outputs[srcoutID].lookAhead = outputs[eofID].lookAhead = outputs[timestampID].lookAhead = 1+max(max(max(outputs[mapoutID].lookAhead, outputs[srcoutID].lookAhead), outputs[eofID].lookAhead), outputs[timestampID].lookAhead);
      outputs[mapoutID].lookBack  = outputs[srcoutID].lookBack  = outputs[eofID].lookBack  = outputs[timestampID].lookBack  = 1+max(max(max(outputs[mapoutID].lookBack,  outputs[srcoutID].lookBack),  outputs[eofID].lookBack),  outputs[timestampID].lookBack);
      if(matoutID != -1) output_exception_flag = 1;
    }
    if(stream_selection == "HarkSrcFeature"){
      _sub_harksrcfeature    = __MasterRosNodeHandler__->subscribe<hark_msgs::HarkSrcFeature>(topic_name_harksrcfeature, msg_buffer_num, &callbackFuncHarkMsgsStreamFromRos::cb_harksrcfeature, cbf);
      outputs[mapoutID].lookAhead = outputs[srcoutID].lookAhead = outputs[eofID].lookAhead = outputs[timestampID].lookAhead = 1+max(max(max(outputs[mapoutID].lookAhead, outputs[srcoutID].lookAhead), outputs[eofID].lookAhead), outputs[timestampID].lookAhead);
      outputs[mapoutID].lookBack  = outputs[srcoutID].lookBack  = outputs[eofID].lookBack  = outputs[timestampID].lookBack  = 1+max(max(max(outputs[mapoutID].lookBack,  outputs[srcoutID].lookBack),  outputs[eofID].lookBack),  outputs[timestampID].lookBack);
      if(matoutID != -1) output_exception_flag = 1;
    }
    if(stream_selection == "HarkSrcFeatureMFM"){
      _sub_harksrcfeaturemfm = __MasterRosNodeHandler__->subscribe<hark_msgs::HarkSrcFeatureMFM>(topic_name_harksrcfeaturemfm, msg_buffer_num, &callbackFuncHarkMsgsStreamFromRos::cb_harksrcfeaturemfm, cbf);
      outputs[mapoutID].lookAhead = outputs[srcoutID].lookAhead = outputs[eofID].lookAhead = outputs[timestampID].lookAhead = 1+max(max(max(outputs[mapoutID].lookAhead, outputs[srcoutID].lookAhead), outputs[eofID].lookAhead), outputs[timestampID].lookAhead);
      outputs[mapoutID].lookBack  = outputs[srcoutID].lookBack  = outputs[eofID].lookBack  = outputs[timestampID].lookBack  = 1+max(max(max(outputs[mapoutID].lookBack,  outputs[srcoutID].lookBack),  outputs[eofID].lookBack),  outputs[timestampID].lookBack);
      if(matoutID != -1) output_exception_flag = 1;
    }
    this->BufferedNode::initialize();
    _snum = new ros::AsyncSpinner(1);
    _snum->start();
  }

  // dynamic output-port translation
  virtual int translateOutput (string outputName)
  {

    if (outputName == "MATOUT") {
      if(bmatoutID){ 
	return matoutID; 
      }else{ 
	bmatoutID = true; 
	return matoutID = addOutput(outputName); 
      } 
    }

    if (outputName == "SRCOUT") {
      if(bsrcoutID){ 
	return srcoutID; 
      }else{ 
	bsrcoutID = true; 
	return srcoutID = addOutput(outputName); 
      } 
    }

    if (outputName == "MAPOUT") {
      if(bmapoutID){ 
	return mapoutID; 
      }else{ 
	bmapoutID = true; 
	return mapoutID = addOutput(outputName); 
      } 
    }

    for (unsigned int i=0; i< outputNames.size(); i++) {	
      if (outputNames[i] == outputName) {
	return i;
      }
    }  
    
    return addOutput(outputName);
  }
  
  void calculate(int output_id, int count, Buffer &out)
  {
    
    ///////////////////////////////////////////////////////////////
    //
    //      bodies
    //
    ///////////////////////////////////////////////////////////////

    RCPtr<TimeStamp> cur_timestamp(new TimeStamp());
    (*outputs[timestampID].buffer)[count] = cur_timestamp;

    Buffer &eofBuffer = *(outputs[eofID].buffer);
    eofBuffer[count] = TrueObject;

    ros::Rate loop_rate(ros_loop_rate);

    if(output_exception_flag){
      cout << "Error : "<< getName() << " output exception -> Connected output ports are not appropriate..." << endl;
      eofBuffer[count] = FalseObject;
      return;
    }

    while(cbf->init_flag){
      if(exit_flag_HarkMsgsStreamFromRos)
	throw new NodeException(NULL, getName()+string(" Force-quit..."), __FILE__, __LINE__);
      usleep(1);
    }

    // Set the base time

    gettimeofday(&tv_base, NULL);
    long int usec_base = tv_base.tv_usec;
    long int sec_base  = tv_base.tv_sec % 10;
    long int sec_cnt_base = sec_base * 1000000 + usec_base;

    //----- HarkWave message
    //////////////////////////////////////////////////////////////

    if(stream_selection == "HarkWave"){    

      while(cbf->deque_harkwave.size()==0){
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
	if(exit_flag_HarkMsgsStreamFromRos)
	  throw new NodeException(NULL, getName()+string(" Force-quit..."), __FILE__, __LINE__);
      }
      
      RCPtr<Matrix<float> > matout(new Matrix<float>(cbf->deque_harkwave[0].nch, cbf->deque_harkwave[0].length));    
      (*(outputs[matoutID].buffer))[count] = matout;
      
      if(cbf->deque_harkwave.size()){

        cur_timestamp->setTime((long int)cbf->deque_harkwave[0].header.stamp.sec, (long int)cbf->deque_harkwave[0].header.stamp.nsec);
      
	for (int k = 0; k < cbf->deque_harkwave[0].nch; k++) {
	  for (int i = 0; i < cbf->deque_harkwave[0].length; i++) {
	    (*matout)(k, i) = cbf->deque_harkwave[0].src[k].wavedata[i];
	  }
	}
	
	if (enable_debug == true) cout    << count << " : " << cbf->deque_harkwave.size() << " : " << cbf->deque_harkwave[0].count << " : " << cbf->deque_harkwave[0].nch << endl;
	
	cbf->deque_harkwave.pop_front();    
      }

    }

    //----- HarkFFT message
    //////////////////////////////////////////////////////////////

    if(stream_selection == "HarkFFT"){    

      while(cbf->deque_harkfft.size()==0){
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
	if(exit_flag_HarkMsgsStreamFromRos)
	  throw new NodeException(NULL, getName()+string(" Force-quit..."), __FILE__, __LINE__);
      }
      
      RCPtr<Matrix<complex<float> > > matout(new Matrix<complex<float> >(cbf->deque_harkfft[0].nch, cbf->deque_harkfft[0].length));    
      (*(outputs[matoutID].buffer))[count] = matout;
      
      if(cbf->deque_harkfft.size()){
      
        cur_timestamp->setTime((long int)cbf->deque_harkfft[0].header.stamp.sec, (long int)cbf->deque_harkfft[0].header.stamp.nsec);

	for (int k = 0; k < cbf->deque_harkfft[0].nch; k++) {
	  for (int i = 0; i < cbf->deque_harkfft[0].length; i++) {
	    (*matout)(k, i).real() = cbf->deque_harkfft[0].src[k].fftdata_real[i];
	    (*matout)(k, i).imag() = cbf->deque_harkfft[0].src[k].fftdata_imag[i];
	  }
	}
	
	if (enable_debug == true) cout    << count << " : " << cbf->deque_harkfft.size() << " : " << cbf->deque_harkfft[0].count << " : " << cbf->deque_harkfft[0].nch << endl;
	
	cbf->deque_harkfft.pop_front();    
      }

    }

    //----- HarkFeature message
    //////////////////////////////////////////////////////////////

    if(stream_selection == "HarkFeature"){    

      while(cbf->deque_harkfeature.size()==0){
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
	if(exit_flag_HarkMsgsStreamFromRos)
	  throw new NodeException(NULL, getName()+string(" Force-quit..."), __FILE__, __LINE__);
      }

      RCPtr<Map<int, ObjectRef> > mapout(new Map<int, ObjectRef>);
      (*(outputs[mapoutID].buffer))[count] = mapout;
      
      if(cbf->deque_harkfeature.size()){

        cur_timestamp->setTime((long int)cbf->deque_harkfeature[0].header.stamp.sec, (long int)cbf->deque_harkfeature[0].header.stamp.nsec);

	for(int i = 0; i < cbf->deque_harkfeature[0].exist_src_num; i++){
	  RCPtr<Vector<float> > feature(new Vector<float>(cbf->deque_harkfeature[0].src[i].length));
	  for(int j = 0; j < cbf->deque_harkfeature[0].src[i].length; j++){
	    (*feature)[j] = cbf->deque_harkfeature[0].src[i].featuredata[j];
	  }
	  (*mapout)[cbf->deque_harkfeature[0].src[i].id] = feature;
	}
	
	if (enable_debug == true) cout    << count << " : " << cbf->deque_harkfeature.size() << " : " << cbf->deque_harkfeature[0].count << " : " << cbf->deque_harkfeature[0].exist_src_num << endl;
	
	cbf->deque_harkfeature.pop_front();    
	
      }      

    }
      
    //----- HarkSource message
    //////////////////////////////////////////////////////////////

    if(stream_selection == "HarkSource"){    

      while(cbf->deque_harksource.size()==0){
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
	if(exit_flag_HarkMsgsStreamFromRos)
	  throw new NodeException(NULL, getName()+string(" Force-quit..."), __FILE__, __LINE__);
      }

      Vector<ObjectRef>& srcout = *new Vector<ObjectRef>;
      (*(outputs[srcoutID].buffer))[count] = &srcout;
      
      if(cbf->deque_harksource.size()){

        cur_timestamp->setTime((long int)cbf->deque_harksource[0].header.stamp.sec, (long int)cbf->deque_harksource[0].header.stamp.nsec);
	
	for(int i = 0; i < cbf->deque_harksource[0].exist_src_num; i++){
	  RCPtr<Source> src(new Source);
	  src->id    = cbf->deque_harksource[0].src[i].id;
	  src->power = cbf->deque_harksource[0].src[i].power;
	  src->x[0]  = cbf->deque_harksource[0].src[i].x;
	  src->x[1]  = cbf->deque_harksource[0].src[i].y;
	  src->x[2]  = cbf->deque_harksource[0].src[i].z;
	  srcout.push_back(src);
	}
	
	if (enable_debug == true) cout    << count << " : " << cbf->deque_harksource.size() << " : " << cbf->deque_harksource[0].count << " : " << cbf->deque_harksource[0].exist_src_num << endl;
	
	cbf->deque_harksource.pop_front();    
	
      }

    }
    
    //----- HarkSrcWave message
    //////////////////////////////////////////////////////////////

    if(stream_selection == "HarkSrcWave"){    

      while(cbf->deque_harksrcwave.size()==0){
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
	if(exit_flag_HarkMsgsStreamFromRos)
	  throw new NodeException(NULL, getName()+string(" Force-quit..."), __FILE__, __LINE__);
      }

      Vector<ObjectRef>& srcout = *new Vector<ObjectRef>;
      (*(outputs[srcoutID].buffer))[count] = &srcout;

      RCPtr<Map<int, ObjectRef> > mapout(new Map<int, ObjectRef>);
      (*(outputs[mapoutID].buffer))[count] = mapout;

      if(cbf->deque_harksrcwave.size()){

        cur_timestamp->setTime((long int)cbf->deque_harksrcwave[0].header.stamp.sec, (long int)cbf->deque_harksrcwave[0].header.stamp.nsec);
	
	for(int i = 0; i < cbf->deque_harksrcwave[0].exist_src_num; i++){
	  RCPtr<Source> src(new Source);
	  src->id    = cbf->deque_harksrcwave[0].src[i].id;
	  src->power = cbf->deque_harksrcwave[0].src[i].power;
	  src->x[0]  = cbf->deque_harksrcwave[0].src[i].x;
	  src->x[1]  = cbf->deque_harksrcwave[0].src[i].y;
	  src->x[2]  = cbf->deque_harksrcwave[0].src[i].z;
	  srcout.push_back(src);
	}    
		
	for(int i = 0; i < cbf->deque_harksrcwave[0].exist_src_num; i++){
	  RCPtr<Vector<float> > wave(new Vector<float>(cbf->deque_harksrcwave[0].src[i].length));
	  for(int j = 0; j < cbf->deque_harksrcwave[0].src[i].length; j++){
	    (*wave)[j] = cbf->deque_harksrcwave[0].src[i].wavedata[j];
	  }
	  (*mapout)[cbf->deque_harksrcwave[0].src[i].id] = wave;
	}

	if (enable_debug == true) cout    << count << " : " << cbf->deque_harksrcwave.size() << " : " << cbf->deque_harksrcwave[0].count << " : " << cbf->deque_harksrcwave[0].exist_src_num << endl;
	
	cbf->deque_harksrcwave.pop_front();    
	
      }

    }

    //----- HarkSrcFFT message
    //////////////////////////////////////////////////////////////

    if(stream_selection == "HarkSrcFFT"){    

      while(cbf->deque_harksrcfft.size()==0){
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
	if(exit_flag_HarkMsgsStreamFromRos)
	  throw new NodeException(NULL, getName()+string(" Force-quit..."), __FILE__, __LINE__);
      }

      Vector<ObjectRef>& srcout = *new Vector<ObjectRef>;
      (*(outputs[srcoutID].buffer))[count] = &srcout;
      
      RCPtr<Map<int, ObjectRef> > mapout(new Map<int, ObjectRef>);
      (*(outputs[mapoutID].buffer))[count] = mapout;
      
      if(cbf->deque_harksrcfft.size()){
	
        cur_timestamp->setTime((long int)cbf->deque_harksrcfft[0].header.stamp.sec, (long int)cbf->deque_harksrcfft[0].header.stamp.nsec);

	for(int i = 0; i < cbf->deque_harksrcfft[0].exist_src_num; i++){
	  RCPtr<Source> src(new Source);
	  src->id    = cbf->deque_harksrcfft[0].src[i].id;
	  src->power = cbf->deque_harksrcfft[0].src[i].power;
	  src->x[0]  = cbf->deque_harksrcfft[0].src[i].x;
	  src->x[1]  = cbf->deque_harksrcfft[0].src[i].y;
	  src->x[2]  = cbf->deque_harksrcfft[0].src[i].z;
	  srcout.push_back(src);
	}
	
	for(int i = 0; i < cbf->deque_harksrcfft[0].exist_src_num; i++){
	  RCPtr<Vector<complex<float> > > fft(new Vector<complex<float> >(cbf->deque_harksrcfft[0].src[i].length));
	  for(int j = 0; j < cbf->deque_harksrcfft[0].src[i].length; j++){
	    (*fft)[j].real() = cbf->deque_harksrcfft[0].src[i].fftdata_real[j];
	    (*fft)[j].imag() = cbf->deque_harksrcfft[0].src[i].fftdata_imag[j];
	  }
	(*mapout)[cbf->deque_harksrcfft[0].src[i].id] = fft;
	}

	if (enable_debug == true) cout    << count << " : " << cbf->deque_harksrcfft.size() << " : " << cbf->deque_harksrcfft[0].count << " : " << cbf->deque_harksrcfft[0].exist_src_num << endl;
	
	cbf->deque_harksrcfft.pop_front();    
		
      }

    }

    //----- HarkSrcFeature message
    //////////////////////////////////////////////////////////////

    if(stream_selection == "HarkSrcFeature"){    

      while(cbf->deque_harksrcfeature.size()==0){
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
	if(exit_flag_HarkMsgsStreamFromRos)
	  throw new NodeException(NULL, getName()+string(" Force-quit..."), __FILE__, __LINE__);
      }

      Vector<ObjectRef>& srcout = *new Vector<ObjectRef>;
      (*(outputs[srcoutID].buffer))[count] = &srcout;

      RCPtr<Map<int, ObjectRef> > mapout(new Map<int, ObjectRef>);
      (*(outputs[mapoutID].buffer))[count] = mapout;

      if(cbf->deque_harksrcfeature.size()){
	
        cur_timestamp->setTime((long int)cbf->deque_harksrcfeature[0].header.stamp.sec, (long int)cbf->deque_harksrcfeature[0].header.stamp.nsec);

	for(int i = 0; i < cbf->deque_harksrcfeature[0].exist_src_num; i++){
	  RCPtr<Source> src(new Source);
	  src->id    = cbf->deque_harksrcfeature[0].src[i].id;
	  src->power = cbf->deque_harksrcfeature[0].src[i].power;
	  src->x[0]  = cbf->deque_harksrcfeature[0].src[i].x;
	  src->x[1]  = cbf->deque_harksrcfeature[0].src[i].y;
	  src->x[2]  = cbf->deque_harksrcfeature[0].src[i].z;
	  srcout.push_back(src);
	}
            
	for(int i = 0; i < cbf->deque_harksrcfeature[0].exist_src_num; i++){
	  RCPtr<Vector<float> > feature(new Vector<float>(cbf->deque_harksrcfeature[0].src[i].length));
	  for(int j = 0; j < cbf->deque_harksrcfeature[0].src[i].length; j++){
	    (*feature)[j] = cbf->deque_harksrcfeature[0].src[i].featuredata[j];
	  }
	  (*mapout)[cbf->deque_harksrcfeature[0].src[i].id] = feature;
	}

	if (enable_debug == true) cout    << count << " : " << cbf->deque_harksrcfeature.size() << " : " << cbf->deque_harksrcfeature[0].count << " : " << cbf->deque_harksrcfeature[0].exist_src_num << endl;
	
	cbf->deque_harksrcfeature.pop_front();    
	      
      }

    }
    
    //----- HarkSrcFeatureMFM message
    //////////////////////////////////////////////////////////////

    if(stream_selection == "HarkSrcFeatureMFM"){    

      while(cbf->deque_harksrcfeaturemfm.size()==0){
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
	if(exit_flag_HarkMsgsStreamFromRos)
	  throw new NodeException(NULL, getName()+string(" Force-quit..."), __FILE__, __LINE__);
      }

      Vector<ObjectRef>& srcout = *new Vector<ObjectRef>;
      (*(outputs[srcoutID].buffer))[count] = &srcout;

      RCPtr<Map<int, ObjectRef> > mapout(new Map<int, ObjectRef>);
      (*(outputs[mapoutID].buffer))[count] = mapout;

      if(cbf->deque_harksrcfeaturemfm.size()){

        cur_timestamp->setTime((long int)cbf->deque_harksrcfeaturemfm[0].header.stamp.sec, (long int)cbf->deque_harksrcfeaturemfm[0].header.stamp.nsec);
	
	for(int i = 0; i < cbf->deque_harksrcfeaturemfm[0].exist_src_num; i++){
	  RCPtr<Source> src(new Source);
	  src->id    = cbf->deque_harksrcfeaturemfm[0].src[i].id;
	  src->power = cbf->deque_harksrcfeaturemfm[0].src[i].power;
	  src->x[0]  = cbf->deque_harksrcfeaturemfm[0].src[i].x;
	  src->x[1]  = cbf->deque_harksrcfeaturemfm[0].src[i].y;
	  src->x[2]  = cbf->deque_harksrcfeaturemfm[0].src[i].z;
	  srcout.push_back(src);
	}
      
	for(int i = 0; i < cbf->deque_harksrcfeaturemfm[0].exist_src_num; i++){
	  RCPtr<Vector<float> > mfm(new Vector<float>(cbf->deque_harksrcfeaturemfm[0].src[i].length_mfm));
	  for(int j = 0; j < cbf->deque_harksrcfeaturemfm[0].src[i].length_mfm; j++){
	    (*mfm)[j] = cbf->deque_harksrcfeaturemfm[0].src[i].featuredata_mfm[j];
	  }
	  (*mapout)[cbf->deque_harksrcfeaturemfm[0].src[i].id] = mfm;
	}
	
	if (enable_debug == true) cout    << count << " : " << cbf->deque_harksrcfeaturemfm.size() << " : " << cbf->deque_harksrcfeaturemfm[0].count << " : " << cbf->deque_harksrcfeaturemfm[0].exist_src_num << endl;
	
	cbf->deque_harksrcfeaturemfm.pop_front();    
	
      }

    }

    loop_rate.sleep();
    
    // Main loop routine ends here.

  }
};

#endif
