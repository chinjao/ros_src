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
#include "Matrix.h"
#include <../config.h>

#ifdef ENABLE_ROS

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <boost/thread.hpp>
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"
#include "std_msgs/Int8.h"
#include "std_msgs/Int8MultiArray.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Int16MultiArray.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Int32MultiArray.h"
#include "std_msgs/Int64.h"
#include "std_msgs/Int64MultiArray.h"
#include "std_msgs/UInt8.h"
#include "std_msgs/UInt8MultiArray.h"
#include "std_msgs/UInt16.h"
#include "std_msgs/UInt16MultiArray.h"
#include "std_msgs/UInt32.h"
#include "std_msgs/UInt32MultiArray.h"
#include "std_msgs/UInt64.h"
#include "std_msgs/UInt64MultiArray.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float64MultiArray.h"
#include "HarkRosGlobals.h"
#include <math.h>
#include <vector>
#include <complex>
#include <queue>

using namespace std;
using namespace FD;

class callbackFuncStdMsgsSubscriber
{

public:
  deque<std_msgs::Bool>              deque_stdbool;
  deque<std_msgs::String>            deque_stdstring;
  deque<std_msgs::Int32>             deque_stdint32;
  deque<std_msgs::Float32>           deque_stdfloat32;
  deque<std_msgs::Int32MultiArray>   deque_stdint32multiarray;
  deque<std_msgs::Float32MultiArray> deque_stdfloat32multiarray;
  bool enable_debug;
  int data_buffer_num;  

  callbackFuncStdMsgsSubscriber(int buffer, bool debug){
    data_buffer_num = buffer;
    enable_debug = debug;
    deque_stdbool.resize(0);
    deque_stdstring.resize(0);
    deque_stdint32.resize(0);
    deque_stdfloat32.resize(0);
    deque_stdint32multiarray.resize(0);
    deque_stdfloat32multiarray.resize(0);
  }

  void cb_stdbool(const std_msgs::Bool::ConstPtr& msg)
  {
    if(enable_debug)
      ROS_INFO_STREAM("Received [thread=" << boost::this_thread::get_id() << "]");
    std_msgs::Bool oMessage = *msg.get();
    if(deque_stdbool.size() < data_buffer_num){
      deque_stdbool.push_back(oMessage);
    }else{      
      deque_stdbool.pop_front();
      deque_stdbool.push_back(oMessage);
    }    
  }

  void cb_stdstring(const std_msgs::String::ConstPtr& msg)
  {
    if(enable_debug)
      ROS_INFO_STREAM("Received [thread=" << boost::this_thread::get_id() << "]");
    std_msgs::String oMessage = *msg.get();
    if(deque_stdstring.size() < data_buffer_num){
      deque_stdstring.push_back(oMessage);
    }else{      
      deque_stdstring.pop_front();
      deque_stdstring.push_back(oMessage);
    }    
  }

  void cb_stdint8(const std_msgs::Int8::ConstPtr& msg)
  {
    if(enable_debug)
      ROS_INFO_STREAM("Received [thread=" << boost::this_thread::get_id() << "]");
    std_msgs::Int32 oMessage;
    oMessage.data = (int)msg->data;
    if(deque_stdint32.size() < data_buffer_num){
      deque_stdint32.push_back(oMessage);
    }else{      
      deque_stdint32.pop_front();
      deque_stdint32.push_back(oMessage);
    }    
  }

  void cb_stdint16(const std_msgs::Int16::ConstPtr& msg)
  {
    if(enable_debug)
      ROS_INFO_STREAM("Received [thread=" << boost::this_thread::get_id() << "]");
    std_msgs::Int32 oMessage;
    oMessage.data = (int)msg->data;
    if(deque_stdint32.size() < data_buffer_num){
      deque_stdint32.push_back(oMessage);
    }else{      
      deque_stdint32.pop_front();
      deque_stdint32.push_back(oMessage);
    }    
  }

  void cb_stdint32(const std_msgs::Int32::ConstPtr& msg)
  {
    if(enable_debug)
      ROS_INFO_STREAM("Received [thread=" << boost::this_thread::get_id() << "]");
    std_msgs::Int32 oMessage;
    oMessage.data = (int)msg->data;
    if(deque_stdint32.size() < data_buffer_num){
      deque_stdint32.push_back(oMessage);
    }else{      
      deque_stdint32.pop_front();
      deque_stdint32.push_back(oMessage);
    }    
  }

  void cb_stdint64(const std_msgs::Int64::ConstPtr& msg)
  {
    if(enable_debug)
      ROS_INFO_STREAM("Received [thread=" << boost::this_thread::get_id() << "]");
    std_msgs::Int32 oMessage;
    oMessage.data = (int)msg->data;
    if(deque_stdint32.size() < data_buffer_num){
      deque_stdint32.push_back(oMessage);
    }else{      
      deque_stdint32.pop_front();
      deque_stdint32.push_back(oMessage);
    }    
  }

  void cb_stduint8(const std_msgs::UInt8::ConstPtr& msg)
  {
    if(enable_debug)
      ROS_INFO_STREAM("Received [thread=" << boost::this_thread::get_id() << "]");
    std_msgs::Int32 oMessage;
    oMessage.data = (int)msg->data;
    if(deque_stdint32.size() < data_buffer_num){
      deque_stdint32.push_back(oMessage);
    }else{      
      deque_stdint32.pop_front();
      deque_stdint32.push_back(oMessage);
    }    
  }

  void cb_stduint16(const std_msgs::UInt16::ConstPtr& msg)
  {
    if(enable_debug)
      ROS_INFO_STREAM("Received [thread=" << boost::this_thread::get_id() << "]");
    std_msgs::Int32 oMessage;
    oMessage.data = (int)msg->data;
    if(deque_stdint32.size() < data_buffer_num){
      deque_stdint32.push_back(oMessage);
    }else{      
      deque_stdint32.pop_front();
      deque_stdint32.push_back(oMessage);
    }    
  }

  void cb_stduint32(const std_msgs::UInt32::ConstPtr& msg)
  {
    if(enable_debug)
      ROS_INFO_STREAM("Received [thread=" << boost::this_thread::get_id() << "]");
    std_msgs::Int32 oMessage;
    oMessage.data = (int)msg->data;
    if(deque_stdint32.size() < data_buffer_num){
      deque_stdint32.push_back(oMessage);
    }else{      
      deque_stdint32.pop_front();
      deque_stdint32.push_back(oMessage);
    }    
  }

  void cb_stduint64(const std_msgs::UInt64::ConstPtr& msg)
  {
    if(enable_debug)
      ROS_INFO_STREAM("Received [thread=" << boost::this_thread::get_id() << "]");
    std_msgs::Int32 oMessage;
    oMessage.data = (int)msg->data;
    if(deque_stdint32.size() < data_buffer_num){
      deque_stdint32.push_back(oMessage);
    }else{      
      deque_stdint32.pop_front();
      deque_stdint32.push_back(oMessage);
    }
  }

  void cb_stdfloat32(const std_msgs::Float32::ConstPtr& msg)
  {
    if(enable_debug)
      ROS_INFO_STREAM("Received [thread=" << boost::this_thread::get_id() << "]");
    std_msgs::Float32 oMessage;
    oMessage.data = (float)msg->data;
    if(deque_stdfloat32.size() < data_buffer_num){
      deque_stdfloat32.push_back(oMessage);
    }else{      
      deque_stdfloat32.pop_front();
      deque_stdfloat32.push_back(oMessage);
    }
  }

  void cb_stdfloat64(const std_msgs::Float64::ConstPtr& msg)
  {
    if(enable_debug)
      ROS_INFO_STREAM("Received [thread=" << boost::this_thread::get_id() << "]");
    std_msgs::Float32 oMessage;
    oMessage.data = (float)msg->data;
    if(deque_stdfloat32.size() < data_buffer_num){
      deque_stdfloat32.push_back(oMessage);
    }else{      
      deque_stdfloat32.pop_front();
      deque_stdfloat32.push_back(oMessage);
    }    
  }

  void cb_stdint8multiarray(const std_msgs::Int8MultiArray::ConstPtr& msg)
  {
    if(enable_debug) ROS_INFO_STREAM("Received [thread=" << boost::this_thread::get_id() << "]");
    std_msgs::Int32MultiArray oMessage;
    oMessage.data.resize(msg->data.size());
    for(int cnt = 0; cnt < msg->data.size(); cnt++) oMessage.data[cnt] = (int)msg->data[cnt];
    oMessage.layout.data_offset = msg->layout.data_offset;
    oMessage.layout.dim.resize(msg->layout.dim.size());
    for(int cnt = 0; cnt < msg->layout.dim.size(); cnt++){
      oMessage.layout.dim[cnt].label  = msg->layout.dim[cnt].label;
      oMessage.layout.dim[cnt].size   = msg->layout.dim[cnt].size;
      oMessage.layout.dim[cnt].stride = msg->layout.dim[cnt].stride;
    }
    if(deque_stdint32multiarray.size() < data_buffer_num){
      deque_stdint32multiarray.push_back(oMessage);
    }else{      
      deque_stdint32multiarray.pop_front();
      deque_stdint32multiarray.push_back(oMessage);
    }    
  }

  void cb_stdint16multiarray(const std_msgs::Int16MultiArray::ConstPtr& msg)
  {
    if(enable_debug)
      ROS_INFO_STREAM("Received [thread=" << boost::this_thread::get_id() << "]");
    std_msgs::Int32MultiArray oMessage;
    oMessage.data.resize(msg->data.size());
    for(int cnt = 0; cnt < msg->data.size(); cnt++) oMessage.data[cnt] = (int)msg->data[cnt];
    oMessage.layout.data_offset = msg->layout.data_offset;
    oMessage.layout.dim.resize(msg->layout.dim.size());
    for(int cnt = 0; cnt < msg->layout.dim.size(); cnt++){
      oMessage.layout.dim[cnt].label  = msg->layout.dim[cnt].label;
      oMessage.layout.dim[cnt].size   = msg->layout.dim[cnt].size;
      oMessage.layout.dim[cnt].stride = msg->layout.dim[cnt].stride;
    }  
   if(deque_stdint32multiarray.size() < data_buffer_num){
      deque_stdint32multiarray.push_back(oMessage);
    }else{      
      deque_stdint32multiarray.pop_front();
      deque_stdint32multiarray.push_back(oMessage);
    }    
  }

  void cb_stdint32multiarray(const std_msgs::Int32MultiArray::ConstPtr& msg)
  {
    if(enable_debug)
      ROS_INFO_STREAM("Received [thread=" << boost::this_thread::get_id() << "]");
    std_msgs::Int32MultiArray oMessage;
    oMessage.data.resize(msg->data.size());
    for(int cnt = 0; cnt < msg->data.size(); cnt++) oMessage.data[cnt] = (int)msg->data[cnt];
    oMessage.layout.data_offset = msg->layout.data_offset;
    oMessage.layout.dim.resize(msg->layout.dim.size());
    for(int cnt = 0; cnt < msg->layout.dim.size(); cnt++){
      oMessage.layout.dim[cnt].label  = msg->layout.dim[cnt].label;
      oMessage.layout.dim[cnt].size   = msg->layout.dim[cnt].size;
      oMessage.layout.dim[cnt].stride = msg->layout.dim[cnt].stride;
    }  
    if(deque_stdint32multiarray.size() < data_buffer_num){
      deque_stdint32multiarray.push_back(oMessage);
    }else{      
      deque_stdint32multiarray.pop_front();
      deque_stdint32multiarray.push_back(oMessage);
    }    
  }

  void cb_stdint64multiarray(const std_msgs::Int64MultiArray::ConstPtr& msg)
  {
    if(enable_debug)
      ROS_INFO_STREAM("Received [thread=" << boost::this_thread::get_id() << "]");
    std_msgs::Int32MultiArray oMessage;
    oMessage.data.resize(msg->data.size());
    for(int cnt = 0; cnt < msg->data.size(); cnt++) oMessage.data[cnt] = (int)msg->data[cnt];
    oMessage.layout.data_offset = msg->layout.data_offset;
    oMessage.layout.dim.resize(msg->layout.dim.size());
    for(int cnt = 0; cnt < msg->layout.dim.size(); cnt++){
      oMessage.layout.dim[cnt].label  = msg->layout.dim[cnt].label;
      oMessage.layout.dim[cnt].size   = msg->layout.dim[cnt].size;
      oMessage.layout.dim[cnt].stride = msg->layout.dim[cnt].stride;
    }  
    if(deque_stdint32multiarray.size() < data_buffer_num){
      deque_stdint32multiarray.push_back(oMessage);
    }else{      
      deque_stdint32multiarray.pop_front();
      deque_stdint32multiarray.push_back(oMessage);
    }    
  }

  void cb_stduint8multiarray(const std_msgs::UInt8MultiArray::ConstPtr& msg)
  {
    if(enable_debug)
      ROS_INFO_STREAM("Received [thread=" << boost::this_thread::get_id() << "]");
    std_msgs::Int32MultiArray oMessage;
    oMessage.data.resize(msg->data.size());
    for(int cnt = 0; cnt < msg->data.size(); cnt++) oMessage.data[cnt] = (int)msg->data[cnt];
    oMessage.layout.data_offset = msg->layout.data_offset;
    oMessage.layout.dim.resize(msg->layout.dim.size());
    for(int cnt = 0; cnt < msg->layout.dim.size(); cnt++){
      oMessage.layout.dim[cnt].label  = msg->layout.dim[cnt].label;
      oMessage.layout.dim[cnt].size   = msg->layout.dim[cnt].size;
      oMessage.layout.dim[cnt].stride = msg->layout.dim[cnt].stride;
    }  
    if(deque_stdint32multiarray.size() < data_buffer_num){
      deque_stdint32multiarray.push_back(oMessage);
    }else{      
      deque_stdint32multiarray.pop_front();
      deque_stdint32multiarray.push_back(oMessage);
    }    
  }

  void cb_stduint16multiarray(const std_msgs::UInt16MultiArray::ConstPtr& msg)
  {
    if(enable_debug)
      ROS_INFO_STREAM("Received [thread=" << boost::this_thread::get_id() << "]");
    std_msgs::Int32MultiArray oMessage;
    oMessage.data.resize(msg->data.size());
    for(int cnt = 0; cnt < msg->data.size(); cnt++) oMessage.data[cnt] = (int)msg->data[cnt];
    oMessage.layout.data_offset = msg->layout.data_offset;
    oMessage.layout.dim.resize(msg->layout.dim.size());
    for(int cnt = 0; cnt < msg->layout.dim.size(); cnt++){
      oMessage.layout.dim[cnt].label  = msg->layout.dim[cnt].label;
      oMessage.layout.dim[cnt].size   = msg->layout.dim[cnt].size;
      oMessage.layout.dim[cnt].stride = msg->layout.dim[cnt].stride;
    }  
    if(deque_stdint32multiarray.size() < data_buffer_num){
      deque_stdint32multiarray.push_back(oMessage);
    }else{      
      deque_stdint32multiarray.pop_front();
      deque_stdint32multiarray.push_back(oMessage);
    }    
  }

  void cb_stduint32multiarray(const std_msgs::UInt32MultiArray::ConstPtr& msg)
  {
    if(enable_debug)
      ROS_INFO_STREAM("Received [thread=" << boost::this_thread::get_id() << "]");
    std_msgs::Int32MultiArray oMessage;
    oMessage.data.resize(msg->data.size());
    for(int cnt = 0; cnt < msg->data.size(); cnt++) oMessage.data[cnt] = (int)msg->data[cnt];
    oMessage.layout.data_offset = msg->layout.data_offset;
    oMessage.layout.dim.resize(msg->layout.dim.size());
    for(int cnt = 0; cnt < msg->layout.dim.size(); cnt++){
      oMessage.layout.dim[cnt].label  = msg->layout.dim[cnt].label;
      oMessage.layout.dim[cnt].size   = msg->layout.dim[cnt].size;
      oMessage.layout.dim[cnt].stride = msg->layout.dim[cnt].stride;
    }  
    if(deque_stdint32multiarray.size() < data_buffer_num){
      deque_stdint32multiarray.push_back(oMessage);
    }else{      
      deque_stdint32multiarray.pop_front();
      deque_stdint32multiarray.push_back(oMessage);
    }    
  }

  void cb_stduint64multiarray(const std_msgs::UInt64MultiArray::ConstPtr& msg)
  {
    if(enable_debug)
      ROS_INFO_STREAM("Received [thread=" << boost::this_thread::get_id() << "]");
    std_msgs::Int32MultiArray oMessage;
    oMessage.data.resize(msg->data.size());
    for(int cnt = 0; cnt < msg->data.size(); cnt++) oMessage.data[cnt] = (int)msg->data[cnt];
    oMessage.layout.data_offset = msg->layout.data_offset;
    oMessage.layout.dim.resize(msg->layout.dim.size());
    for(int cnt = 0; cnt < msg->layout.dim.size(); cnt++){
      oMessage.layout.dim[cnt].label  = msg->layout.dim[cnt].label;
      oMessage.layout.dim[cnt].size   = msg->layout.dim[cnt].size;
      oMessage.layout.dim[cnt].stride = msg->layout.dim[cnt].stride;
    }  
    if(deque_stdint32multiarray.size() < data_buffer_num){
      deque_stdint32multiarray.push_back(oMessage);
    }else{      
      deque_stdint32multiarray.pop_front();
      deque_stdint32multiarray.push_back(oMessage);
    }    
  }

  void cb_stdfloat32multiarray(const std_msgs::Float32MultiArray::ConstPtr& msg)
  {
    if(enable_debug)
      ROS_INFO_STREAM("Received [thread=" << boost::this_thread::get_id() << "]");
    std_msgs::Float32MultiArray oMessage;
    oMessage.data.resize(msg->data.size());
    for(int cnt = 0; cnt < msg->data.size(); cnt++) oMessage.data[cnt] = (float)msg->data[cnt];
    oMessage.layout.data_offset = msg->layout.data_offset;
    oMessage.layout.dim.resize(msg->layout.dim.size());
    for(int cnt = 0; cnt < msg->layout.dim.size(); cnt++){
      oMessage.layout.dim[cnt].label  = msg->layout.dim[cnt].label;
      oMessage.layout.dim[cnt].size   = msg->layout.dim[cnt].size;
      oMessage.layout.dim[cnt].stride = msg->layout.dim[cnt].stride;
    }  
    if(deque_stdfloat32multiarray.size() < data_buffer_num){
      deque_stdfloat32multiarray.push_back(oMessage);
    }else{      
      deque_stdfloat32multiarray.pop_front();
      deque_stdfloat32multiarray.push_back(oMessage);
    }
  }

  void cb_stdfloat64multiarray(const std_msgs::Float64MultiArray::ConstPtr& msg)
  {
    if(enable_debug)
      ROS_INFO_STREAM("Received [thread=" << boost::this_thread::get_id() << "]");
    std_msgs::Float32MultiArray oMessage;
    oMessage.data.resize(msg->data.size());
    for(int cnt = 0; cnt < msg->data.size(); cnt++) oMessage.data[cnt] = (float)msg->data[cnt];
    oMessage.layout.data_offset = msg->layout.data_offset;
    oMessage.layout.dim.resize(msg->layout.dim.size());
    for(int cnt = 0; cnt < msg->layout.dim.size(); cnt++){
      oMessage.layout.dim[cnt].label  = msg->layout.dim[cnt].label;
      oMessage.layout.dim[cnt].size   = msg->layout.dim[cnt].size;
      oMessage.layout.dim[cnt].stride = msg->layout.dim[cnt].stride;
    }  
    if(deque_stdfloat32multiarray.size() < data_buffer_num){
      deque_stdfloat32multiarray.push_back(oMessage);
    }else{      
      deque_stdfloat32multiarray.pop_front();
      deque_stdfloat32multiarray.push_back(oMessage);
    }    
  }

};


class RosStdMsgsSubscriber;

DECLARE_NODE(RosStdMsgsSubscriber);
/*Node
 *
 * @name RosStdMsgsSubscriber
 * @category HARK:ROS:IO
 * @description ROS strandard msg subscriber node (std_msgs).
 * 
 * @parameter_name ROS_MESSAGE_TYPE
 * @parameter_type string
 * @parameter_list Bool:String:Int8:Int8MultiArray:Int16:Int16MultiArray:Int32:Int32MultiArray:Int64:Int64MultiArray:UInt8:UInt8MultiArray:UInt16:UInt16MultiArray:UInt32:UInt32MultiArray:UInt64:UInt64MultiArray:Float32:Float32MultiArray:Float64:Float64MultiArray
 * @parameter_value Int32
 * @parameter_description Subscribed ROS message type. Match this setting and subscribed ROS message type.
 * 
 * @parameter_name OUT_ARRAY_FORMAT
 * @parameter_type string
 * @parameter_list Vector<>:Vector<complex<>>:Matrix<>:Matrix<complex<>>
 * @parameter_value Vector<>
 * @parameter_description Hark output format. This parameter is valid when the ROS_MESSAGE_TYPE is related to MultiArray. The subscribed MultiArray message is converted to this format.
 * 
 * @parameter_name BOOL_DEFAULT
 * @parameter_type bool
 * @parameter_value false
 * @parameter_description Default output bool during no data period [default: false]
 * 
 * @parameter_name INT_DEFAULT
 * @parameter_type int
 * @parameter_value 0
 * @parameter_description Default output integer during no data period [default: 0]
 *
 * @parameter_name FLOAT_DEFAULT
 * @parameter_type float
 * @parameter_value 0.0
 * @parameter_description Default output float during no data period [default: 0.0]
 *
 * @parameter_name ENABLE_DEBUG
 * @parameter_type bool
 * @parameter_value false
 * @parameter_list true:false
 * @parameter_description print debug message of this module in case of true.
 * 
 * @parameter_name TOPIC_NAME
 * @parameter_type string
 * @parameter_value HarkStdMsgs
 * @parameter_description Subscribed topic name for ROS (HarkWave type message)
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
 * @parameter_description Buffer save size for a HARK output. Too small value makes packet loss. Too large value takes large memory. Minimum size is 5.
 *
 * @parameter_name REMAIN_LATEST
 * @parameter_type bool
 * @parameter_value false
 * @parameter_list true:false
 * @parameter_description Enable to keep the latest data as output even when this module doesn't receive any messages [default: false]
 *
 * @output_name OUTPUT
 * @output_type any
 * @output_description Output of subscribed data
 *
END*/

class RosStdMsgsSubscriber : public BufferedNode {

  int outputID;
  string ros_message_type;
  string out_message_type;
  string out_array_format;
  bool bool_default;
  int int_default;
  float float_default;
  bool enable_debug;
  string topic_name;
  float ros_loop_rate;
  int msg_buffer_num;  
  int data_buffer_num;
  bool remain_latest;

  callbackFuncStdMsgsSubscriber *cbf;

private:
  ros::Subscriber _sub;
  ros::AsyncSpinner *_snum;
  
public:
  RosStdMsgsSubscriber(string nodeName, ParameterSet params)
    : BufferedNode(nodeName, params)
  {
    outputID = addOutput("OUTPUT");
    ros_message_type = object_cast<String>(parameters.get("ROS_MESSAGE_TYPE"));
    out_array_format = object_cast<String>(parameters.get("OUT_ARRAY_FORMAT"));
    topic_name = object_cast<String>(parameters.get("TOPIC_NAME"));
    bool_default = dereference_cast<bool>(parameters.get("BOOL_DEFAULT"));
    int_default = dereference_cast<int>(parameters.get("INT_DEFAULT"));
    float_default = dereference_cast<float>(parameters.get("FLOAT_DEFAULT"));
    enable_debug = dereference_cast<bool>(parameters.get("ENABLE_DEBUG"));
    ros_loop_rate = dereference_cast<float>(parameters.get("ROS_LOOP_RATE"));
    msg_buffer_num = dereference_cast<int>(parameters.get("MSG_BUFFER_NUM"));
    data_buffer_num = dereference_cast<int>(parameters.get("DATA_BUFFER_NUM"));
    remain_latest = dereference_cast<bool>(parameters.get("REMAIN_LATEST"));
    out_message_type = "";
    if(data_buffer_num <= 5) data_buffer_num = 5;
    cbf = new callbackFuncStdMsgsSubscriber(data_buffer_num, enable_debug);
    inOrder = true;
    cout << getName() << " constructor end..." << endl;
  }

  ~RosStdMsgsSubscriber()
  {
    _snum->stop();
  }

  virtual void initialize()
  {
    cout << getName() << " initialized..." << endl;

    if(ros_message_type == "Bool"){ 
      _sub  = __MasterRosNodeHandler__->subscribe<std_msgs::Bool>(topic_name, msg_buffer_num, &callbackFuncStdMsgsSubscriber::cb_stdbool, cbf);
      out_message_type = "Bool";
    }
    
    if(ros_message_type == "String"){
      _sub  = __MasterRosNodeHandler__->subscribe<std_msgs::String>(topic_name, msg_buffer_num, &callbackFuncStdMsgsSubscriber::cb_stdstring, cbf);
      out_message_type = "String";
    }
    
    if(ros_message_type == "Int8"){
      _sub  = __MasterRosNodeHandler__->subscribe<std_msgs::Int8>(topic_name, msg_buffer_num, &callbackFuncStdMsgsSubscriber::cb_stdint8, cbf);
      out_message_type = "Int32";
    }    
    if(ros_message_type == "Int16"){
      _sub  = __MasterRosNodeHandler__->subscribe<std_msgs::Int16>(topic_name, msg_buffer_num, &callbackFuncStdMsgsSubscriber::cb_stdint16, cbf);
      out_message_type = "Int32";
    }    
    if(ros_message_type == "Int32"){
      _sub  = __MasterRosNodeHandler__->subscribe<std_msgs::Int32>(topic_name, msg_buffer_num, &callbackFuncStdMsgsSubscriber::cb_stdint32, cbf);
      out_message_type = "Int32";
    }    
    if(ros_message_type == "Int64"){ 
      _sub  = __MasterRosNodeHandler__->subscribe<std_msgs::Int64>(topic_name, msg_buffer_num, &callbackFuncStdMsgsSubscriber::cb_stdint64, cbf);
      out_message_type = "Int32";
    }    
    if(ros_message_type == "UInt8"){
      _sub  = __MasterRosNodeHandler__->subscribe<std_msgs::UInt8>(topic_name, msg_buffer_num, &callbackFuncStdMsgsSubscriber::cb_stduint8, cbf);
      out_message_type = "Int32";
    }    
    if(ros_message_type == "UInt16"){
      _sub  = __MasterRosNodeHandler__->subscribe<std_msgs::UInt16>(topic_name, msg_buffer_num, &callbackFuncStdMsgsSubscriber::cb_stduint16, cbf);
      out_message_type = "Int32";
    }    
    if(ros_message_type == "UInt32"){
      _sub  = __MasterRosNodeHandler__->subscribe<std_msgs::UInt32>(topic_name, msg_buffer_num, &callbackFuncStdMsgsSubscriber::cb_stduint32, cbf);
      out_message_type = "Int32";
    }    
    if(ros_message_type == "UInt64"){
      _sub  = __MasterRosNodeHandler__->subscribe<std_msgs::UInt64>(topic_name, msg_buffer_num, &callbackFuncStdMsgsSubscriber::cb_stduint64, cbf);
      out_message_type = "Int32";
    }    

    if(ros_message_type == "Float32"){
      _sub  = __MasterRosNodeHandler__->subscribe<std_msgs::Float32>(topic_name, msg_buffer_num, &callbackFuncStdMsgsSubscriber::cb_stdfloat32, cbf);
      out_message_type = "Float32";
    }    
    if(ros_message_type == "Float64"){
      _sub  = __MasterRosNodeHandler__->subscribe<std_msgs::Float64>(topic_name, msg_buffer_num, &callbackFuncStdMsgsSubscriber::cb_stdfloat64, cbf);
      out_message_type = "Float32";
    }    

    if(ros_message_type == "Int8MultiArray"){
      _sub  = __MasterRosNodeHandler__->subscribe<std_msgs::Int8MultiArray>(topic_name, msg_buffer_num, &callbackFuncStdMsgsSubscriber::cb_stdint8multiarray, cbf);
      out_message_type = "Int32MultiArray";
    }    
    if(ros_message_type == "Int16MultiArray"){
      _sub  = __MasterRosNodeHandler__->subscribe<std_msgs::Int16MultiArray>(topic_name, msg_buffer_num, &callbackFuncStdMsgsSubscriber::cb_stdint16multiarray, cbf);
      out_message_type = "Int32MultiArray";
    }    
    if(ros_message_type == "Int32MultiArray"){
      _sub  = __MasterRosNodeHandler__->subscribe<std_msgs::Int32MultiArray>(topic_name, msg_buffer_num, &callbackFuncStdMsgsSubscriber::cb_stdint32multiarray, cbf);
      out_message_type = "Int32MultiArray";
    }    
    if(ros_message_type == "Int64MultiArray"){
      _sub  = __MasterRosNodeHandler__->subscribe<std_msgs::Int64MultiArray>(topic_name, msg_buffer_num, &callbackFuncStdMsgsSubscriber::cb_stdint64multiarray, cbf);
      out_message_type = "Int32MultiArray";
    }    
    if(ros_message_type == "UInt8MultiArray"){
      _sub  = __MasterRosNodeHandler__->subscribe<std_msgs::UInt8MultiArray>(topic_name, msg_buffer_num, &callbackFuncStdMsgsSubscriber::cb_stduint8multiarray, cbf);
      out_message_type = "Int32MultiArray";
    }    
    if(ros_message_type == "UInt16MultiArray"){
      _sub  = __MasterRosNodeHandler__->subscribe<std_msgs::UInt16MultiArray>(topic_name, msg_buffer_num, &callbackFuncStdMsgsSubscriber::cb_stduint16multiarray, cbf);
      out_message_type = "Int32MultiArray";
    }    
    if(ros_message_type == "UInt32MultiArray"){
      _sub  = __MasterRosNodeHandler__->subscribe<std_msgs::UInt32MultiArray>(topic_name, msg_buffer_num, &callbackFuncStdMsgsSubscriber::cb_stduint32multiarray, cbf);
      out_message_type = "Int32MultiArray";
    }    
    if(ros_message_type == "UInt64MultiArray"){
      _sub  = __MasterRosNodeHandler__->subscribe<std_msgs::UInt64MultiArray>(topic_name, msg_buffer_num, &callbackFuncStdMsgsSubscriber::cb_stduint64multiarray, cbf);
      out_message_type = "Int32MultiArray";
    }    

    if(ros_message_type == "Float32MultiArray"){
      _sub  = __MasterRosNodeHandler__->subscribe<std_msgs::Float32MultiArray>(topic_name, msg_buffer_num, &callbackFuncStdMsgsSubscriber::cb_stdfloat32multiarray, cbf);
      out_message_type = "Float32MultiArray";
    }    
    if(ros_message_type == "Float64MultiArray"){
      _sub  = __MasterRosNodeHandler__->subscribe<std_msgs::Float64MultiArray>(topic_name, msg_buffer_num, &callbackFuncStdMsgsSubscriber::cb_stdfloat64multiarray, cbf);
      out_message_type = "Float32MultiArray";
    }    

    this->BufferedNode::initialize();
    _snum = new ros::AsyncSpinner(1);
    _snum->start();
  }

  void calculate(int output_id, int count, Buffer &out)
  {
    
    ///////////////////////////////////////////////////////////////
    //
    //      bodies
    //
    ///////////////////////////////////////////////////////////////

    ros::Rate loop_rate(ros_loop_rate);

    //----- Bool message
    //////////////////////////////////////////////////////////////

    if(out_message_type == "Bool"){	
      if(cbf->deque_stdbool.size() == 0){
	bool oBool = bool_default;      
	(*(outputs[outputID].buffer))[count] = (oBool ? TrueObject : FalseObject);
      }else{
	bool oBool = cbf->deque_stdbool[0].data;
	(*(outputs[outputID].buffer))[count] = (oBool ? TrueObject : FalseObject);
	if(((cbf->deque_stdbool.size() >= 1)&&(!remain_latest))||((cbf->deque_stdbool.size() >= 2)&&(remain_latest))) cbf->deque_stdbool.pop_front();    
      }
    }    

    //----- String message
    //////////////////////////////////////////////////////////////

    if(out_message_type == "String"){	
      if(cbf->deque_stdstring.size() == 0){
	string oString = "";      
	(*(outputs[outputID].buffer))[count] = ObjectRef(new String(oString));
      }else{
	string oString = cbf->deque_stdstring[0].data;
	(*(outputs[outputID].buffer))[count] = ObjectRef(new String(oString));
	if(((cbf->deque_stdstring.size() >= 1)&&(!remain_latest))||((cbf->deque_stdstring.size() >= 2)&&(remain_latest))) cbf->deque_stdstring.pop_front();    
      }
    }
    
    //----- Int message
    //////////////////////////////////////////////////////////////

    if(out_message_type == "Int32"){	
      if(cbf->deque_stdint32.size() == 0){
	int oInt = int_default;      
	(*(outputs[outputID].buffer))[count] = ObjectRef(Int::alloc(oInt));
      }else{
	int oInt = cbf->deque_stdint32[0].data;
	(*(outputs[outputID].buffer))[count] = ObjectRef(Int::alloc(oInt));
	if(((cbf->deque_stdint32.size() >= 1)&&(!remain_latest))||((cbf->deque_stdint32.size() >= 2)&&(remain_latest))) cbf->deque_stdint32.pop_front();    
      }
    }
    
    //----- Float message
    //////////////////////////////////////////////////////////////

    if(out_message_type == "Float32"){	
      if(cbf->deque_stdfloat32.size() == 0){
	float oFloat = float_default;      
	(*(outputs[outputID].buffer))[count] = ObjectRef(Float::alloc(oFloat));
      }else{
	float oFloat = cbf->deque_stdfloat32[0].data;
	(*(outputs[outputID].buffer))[count] = ObjectRef(Float::alloc(oFloat));
	if(((cbf->deque_stdfloat32.size() >= 1)&&(!remain_latest))||((cbf->deque_stdfloat32.size() >= 2)&&(remain_latest))) cbf->deque_stdfloat32.pop_front();    
      }
    }
    
    //----- Int32MultiArray message
    //////////////////////////////////////////////////////////////

    if(out_message_type == "Int32MultiArray"){	
      
      //----- Vector<int> message
      //////////////////////////////////////////////////////////////
      
      
      if(out_array_format == "Vector<>"){	
	if(cbf->deque_stdint32multiarray.size() == 0){
	  RCPtr<Vector<int> > vec_out(new Vector<int>(0));
	  (*(outputs[outputID].buffer))[count] = vec_out;
	}else{      
	  RCPtr<Vector<int> > vec_out(new Vector<int>(cbf->deque_stdint32multiarray[0].layout.dim[0].size));
	  (*(outputs[outputID].buffer))[count] = vec_out;
	  for (int i = 0; i < cbf->deque_stdint32multiarray[0].layout.dim[0].size; i++) {
	    (*vec_out)[i] = cbf->deque_stdint32multiarray[0].data[i];
	  }
	  if(((cbf->deque_stdint32multiarray.size() >= 1)&&(!remain_latest))||((cbf->deque_stdint32multiarray.size() >= 2)&&(remain_latest))){
	    cbf->deque_stdint32multiarray.pop_front();    
	  }
	}
      }
    
      //----- Vector<complex<int> > message
      //////////////////////////////////////////////////////////////
      
      if(out_array_format == "Vector<complex<>>"){	
	if(cbf->deque_stdint32multiarray.size() == 0){
	  RCPtr<Vector<complex<float> > > vec_out(new Vector<complex<float> >(0));
	  (*(outputs[outputID].buffer))[count] = vec_out;
	}else{      
	  RCPtr<Vector<complex<float> > > vec_out(new Vector<complex<float> >(cbf->deque_stdint32multiarray[0].layout.dim[0].size));
	  (*(outputs[outputID].buffer))[count] = vec_out;
	  for (int i = 0; i < cbf->deque_stdint32multiarray[0].layout.dim[0].size; i++) {
	    (*vec_out)[0 + i * 2].real() = (float)cbf->deque_stdint32multiarray[0].data[0 + i * 2];	
	    (*vec_out)[1 + i * 2].imag() = (float)cbf->deque_stdint32multiarray[0].data[1 + i * 2];	
	  }
	  if(((cbf->deque_stdint32multiarray.size() >= 1)&&(!remain_latest))||((cbf->deque_stdint32multiarray.size() >= 2)&&(remain_latest))){
	    cbf->deque_stdint32multiarray.pop_front();    
	  }
	}
      }
      
      //----- Matrix<int> message
      //////////////////////////////////////////////////////////////
      
      if(out_array_format == "Matrix<>"){	
	if(cbf->deque_stdint32multiarray.size() == 0){
	  RCPtr<Matrix<int> > mat_out(new Matrix<int>(0,0));
	  (*(outputs[outputID].buffer))[count] = mat_out;
	}else{      
	  RCPtr<Matrix<int> > mat_out(new Matrix<int>(cbf->deque_stdint32multiarray[0].layout.dim[0].size, cbf->deque_stdint32multiarray[0].layout.dim[1].size));
	  (*(outputs[outputID].buffer))[count] = mat_out;
	  int rowm = cbf->deque_stdint32multiarray[0].layout.dim[0].size;
	  int colm = cbf->deque_stdint32multiarray[0].layout.dim[1].size;
	  for (int i = 0; i < rowm; i++) {
	    for (int j = 0; j < colm; j++) {
	      (*mat_out)(i,j) = cbf->deque_stdint32multiarray[0].data[j + i * colm];
	    }
	  }
	  if(((cbf->deque_stdint32multiarray.size() >= 1)&&(!remain_latest))||((cbf->deque_stdint32multiarray.size() >= 2)&&(remain_latest))){
	    cbf->deque_stdint32multiarray.pop_front();    
	  }
	}
      }
      
      //----- Matrix<complex<int> > message
      //////////////////////////////////////////////////////////////
      
      if(out_array_format == "Matrix<complex<>>"){	
	if(cbf->deque_stdint32multiarray.size() == 0){
	  RCPtr<Matrix<complex<float> > > mat_out(new Matrix<complex<float> >(0,0));
	  (*(outputs[outputID].buffer))[count] = mat_out;
	}else{      
	  RCPtr<Matrix<complex<float> > > mat_out(new Matrix<complex<float> >(cbf->deque_stdint32multiarray[0].layout.dim[0].size, cbf->deque_stdint32multiarray[0].layout.dim[1].size));
	  (*(outputs[outputID].buffer))[count] = mat_out;
	  int rowm = cbf->deque_stdint32multiarray[0].layout.dim[0].size;
	  int colm = cbf->deque_stdint32multiarray[0].layout.dim[1].size;
	  for (int i = 0; i < rowm; i++) {
	    for (int j = 0; j < colm; j++) {
	      (*mat_out)(i,j).real() = (float)cbf->deque_stdint32multiarray[0].data[0 + j * 2 + i * colm * 2];
	      (*mat_out)(i,j).imag() = (float)cbf->deque_stdint32multiarray[0].data[1 + j * 2 + i * colm * 2];
	    }
	  }
	  if(((cbf->deque_stdint32multiarray.size() >= 1)&&(!remain_latest))||((cbf->deque_stdint32multiarray.size() >= 2)&&(remain_latest))){
	    cbf->deque_stdint32multiarray.pop_front();    
	  }
	}
      }
            
    }

    //----- Float32MultiArray message
    //////////////////////////////////////////////////////////////

    if(out_message_type == "Float32MultiArray"){	

      //----- Vector<float> message
      //////////////////////////////////////////////////////////////
      
      if(out_array_format == "Vector<>"){	
	if(cbf->deque_stdfloat32multiarray.size() == 0){
	  RCPtr<Vector<float> > vec_out(new Vector<float>(0));
	  (*(outputs[outputID].buffer))[count] = vec_out;
	}else{      
	  RCPtr<Vector<float> > vec_out(new Vector<float>(cbf->deque_stdfloat32multiarray[0].layout.dim[0].size));
	  (*(outputs[outputID].buffer))[count] = vec_out;
	  for (int i = 0; i < cbf->deque_stdfloat32multiarray[0].layout.dim[0].size; i++) {
	    (*vec_out)[i] = cbf->deque_stdfloat32multiarray[0].data[i];
	  }
	  if(((cbf->deque_stdfloat32multiarray.size() >= 1)&&(!remain_latest))||((cbf->deque_stdfloat32multiarray.size() >= 2)&&(remain_latest))){
	    cbf->deque_stdfloat32multiarray.pop_front();    
	  }
	}
      }

      //----- Vector<complex<float> > message
      //////////////////////////////////////////////////////////////
      
      if(out_array_format == "Vector<complex<>>"){	
	if(cbf->deque_stdfloat32multiarray.size() == 0){
	  RCPtr<Vector<complex<float> > > vec_out(new Vector<complex<float> >(0));
	  (*(outputs[outputID].buffer))[count] = vec_out;
	}else{      
	  RCPtr<Vector<complex<float> > > vec_out(new Vector<complex<float> >(cbf->deque_stdfloat32multiarray[0].layout.dim[0].size));
	  (*(outputs[outputID].buffer))[count] = vec_out;
	  for (int i = 0; i < cbf->deque_stdfloat32multiarray[0].layout.dim[0].size; i++) {
	    (*vec_out)[0 + i * 2].real() = cbf->deque_stdfloat32multiarray[0].data[0 + i * 2];	
	    (*vec_out)[1 + i * 2].imag() = cbf->deque_stdfloat32multiarray[0].data[1 + i * 2];	
	  }
	  if(((cbf->deque_stdfloat32multiarray.size() >= 1)&&(!remain_latest))||((cbf->deque_stdfloat32multiarray.size() >= 2)&&(remain_latest))){
	    cbf->deque_stdfloat32multiarray.pop_front();    
	  }
	}
      }
      
      //----- Matrix<float> message
      //////////////////////////////////////////////////////////////
      
      if(out_array_format == "Matrix<>"){	
	if(cbf->deque_stdfloat32multiarray.size() == 0){
	  RCPtr<Matrix<float> > mat_out(new Matrix<float>(0,0));
	  (*(outputs[outputID].buffer))[count] = mat_out;
	}else{      
	  RCPtr<Matrix<float> > mat_out(new Matrix<float>(cbf->deque_stdfloat32multiarray[0].layout.dim[0].size, cbf->deque_stdfloat32multiarray[0].layout.dim[1].size));
	  (*(outputs[outputID].buffer))[count] = mat_out;
	  int rowm = cbf->deque_stdfloat32multiarray[0].layout.dim[0].size;
	  int colm = cbf->deque_stdfloat32multiarray[0].layout.dim[1].size;
	  for (int i = 0; i < rowm; i++) {
	    for (int j = 0; j < colm; j++) {
	      (*mat_out)(i,j) = cbf->deque_stdfloat32multiarray[0].data[j + i * colm];
	    }
	  }
	  if(((cbf->deque_stdfloat32multiarray.size() >= 1)&&(!remain_latest))||((cbf->deque_stdfloat32multiarray.size() >= 2)&&(remain_latest))){
	    cbf->deque_stdfloat32multiarray.pop_front();    
	  }
	}
      }
      
      //----- Matrix<complex<float> > message
      //////////////////////////////////////////////////////////////
      
      if(out_array_format == "Matrix<complex<>>"){	
	if(cbf->deque_stdfloat32multiarray.size() == 0){
	  RCPtr<Matrix<complex<float> > > mat_out(new Matrix<complex<float> >(0,0));
	  (*(outputs[outputID].buffer))[count] = mat_out;
	}else{      
	  RCPtr<Matrix<complex<float> > > mat_out(new Matrix<complex<float> >(cbf->deque_stdfloat32multiarray[0].layout.dim[0].size, cbf->deque_stdfloat32multiarray[0].layout.dim[1].size));
	  (*(outputs[outputID].buffer))[count] = mat_out;
	  int rowm = cbf->deque_stdfloat32multiarray[0].layout.dim[0].size;
	  int colm = cbf->deque_stdfloat32multiarray[0].layout.dim[1].size;
	  for (int i = 0; i < rowm; i++) {
	    for (int j = 0; j < colm; j++) {
	      (*mat_out)(i,j).real() = cbf->deque_stdfloat32multiarray[0].data[0 + j * 2 + i * colm * 2];
	      (*mat_out)(i,j).imag() = cbf->deque_stdfloat32multiarray[0].data[1 + j * 2 + i * colm * 2];
	    }
	  }
	  if(((cbf->deque_stdfloat32multiarray.size() >= 1)&&(!remain_latest))||((cbf->deque_stdfloat32multiarray.size() >= 2)&&(remain_latest))){
	    cbf->deque_stdfloat32multiarray.pop_front();    
	  }
	}
      }

    }
    
    loop_rate.sleep();
    
    // Main loop routine ends here.

  }
};

#endif
