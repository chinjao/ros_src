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

#include "BufferedNode.h"
#include "Buffer.h"
#include "Vector.h"
#include "Map.h"
#include "Source.h"
#include "Matrix.h"
#include <sstream>
#include <iomanip>

#include <sys/time.h>

#include <boost/shared_ptr.hpp>

#include <../config.h>

#ifdef ENABLE_ROS
#include <ros/ros.h>
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
#include <string>

using namespace std;
using namespace FD;

class RosStdMsgsPublisher;

DECLARE_NODE(RosStdMsgsPublisher)
/*Node
 *
 * @name RosStdMsgsPublisher
 * @category HARK:ROS:IO
 * @description ROS msg publisher node for ros standard messages (std_msgs).
 *
 * @parameter_name ENABLE_DEBUG
 * @parameter_type bool
 * @parameter_value false
 * @parameter_list true:false
 * @parameter_description Set to true to print debug messages.
 * 
 * @parameter_name ROS_MESSAGE_TYPE
 * @parameter_type string
 * @parameter_list Bool:String:Int8:Int8MultiArray:Int16:Int16MultiArray:Int32:Int32MultiArray:Int64:Int64MultiArray:UInt8:UInt8MultiArray:UInt16:UInt16MultiArray:UInt32:UInt32MultiArray:UInt64:UInt64MultiArray:Float32:Float32MultiArray:Float64:Float64MultiArray
 * @parameter_value Int32
 * @parameter_description Preferred ROS message type when publishing. If the type is not matched to the input, the input will not be published.
 * 
 * @parameter_name TOPIC_NAME
 * @parameter_type string
 * @parameter_value HarkStdMsgs
 * @parameter_description Published topic name for ROS (The message type can be defined by ROS_MESSAGE_TYPE.)
 * 
 * @parameter_name BUFFER_NUM
 * @parameter_type int
 * @parameter_value 100
 * @parameter_description Buffer size for a ROS published message
 * 
 * @parameter_name ROS_LOOP_RATE
 * @parameter_type float
 * @parameter_value 100000
 * @parameter_description This allows you to specify a frequency that you would like to loop at [Hz]. Keep this value large. (If ROS interval is shorter than HARK interval, ROS interval is overwritten.)
 *
 * @parameter_name MAX_FREQUENCY
 * @parameter_type float
 * @parameter_value -1
 * @parameter_description Frequency not to be exceded by the messages in the ROS topic, in Hz. Use -1 for unlimited. Warning - This will cause packet loss if the HARK frame rate is higher than the frequency set here.
 *
 * @input_name INPUT
 * @input_type any
 * @input_description The valiable for publishing
 *
 * @output_name OUTPUT
 * @output_type ObjectRef
 * @output_description This is a dummy output. It has no purpose other than activating this node.
 *
 END*/

class RosStdMsgsPublisher : public BufferedNode {
  bool enable_debug;
  
  int inputID;
  int outputID;
  
  // ROS related parameters
  string ros_message_type;
  string topic_name;
  int buffer_num;
  float ros_loop_rate;
  float max_frequency;
  
private:
  ros::Publisher publisher;  
  ros::Time last_message_time;

public:
  RosStdMsgsPublisher(string nodeName, ParameterSet params)
    : BufferedNode(nodeName, params), inputID( -1 )
  {
    enable_debug   = dereference_cast<bool>(parameters.get("ENABLE_DEBUG"));
    ros_message_type = object_cast<String>(parameters.get("ROS_MESSAGE_TYPE"));
    topic_name     = object_cast<String>(parameters.get("TOPIC_NAME"));
    buffer_num  = dereference_cast<int>(parameters.get("BUFFER_NUM"));
    max_frequency  = dereference_cast<float>(parameters.get("MAX_FREQUENCY"));
    ros_loop_rate  = dereference_cast<float>(parameters.get("ROS_LOOP_RATE"));
    
    outputID = addOutput("OUTPUT");
    inOrder = true;
    cout << getName().c_str() << " constructor end..." << endl;
        
  }

  virtual void initialize()
  {

    if (inputID != -1){

      if(ros_message_type == "Bool")   publisher = __MasterRosNodeHandler__->advertise<std_msgs::Bool>(topic_name, buffer_num, false);

      if(ros_message_type == "String") publisher = __MasterRosNodeHandler__->advertise<std_msgs::String>(topic_name, buffer_num, false);

      if(ros_message_type == "Int8") publisher = __MasterRosNodeHandler__->advertise<std_msgs::Int8>(topic_name, buffer_num, false);
      if(ros_message_type == "Int8MultiArray") publisher = __MasterRosNodeHandler__->advertise<std_msgs::Int8MultiArray>(topic_name, buffer_num, false);
      if(ros_message_type == "Int16") publisher = __MasterRosNodeHandler__->advertise<std_msgs::Int16>(topic_name, buffer_num, false);
      if(ros_message_type == "Int16MultiArray") publisher = __MasterRosNodeHandler__->advertise<std_msgs::Int16MultiArray>(topic_name, buffer_num, false);
      if(ros_message_type == "Int32") publisher = __MasterRosNodeHandler__->advertise<std_msgs::Int32>(topic_name, buffer_num, false);
      if(ros_message_type == "Int32MultiArray") publisher = __MasterRosNodeHandler__->advertise<std_msgs::Int32MultiArray>(topic_name, buffer_num, false);
      if(ros_message_type == "Int64") publisher = __MasterRosNodeHandler__->advertise<std_msgs::Int64>(topic_name, buffer_num, false);
      if(ros_message_type == "Int64MultiArray") publisher = __MasterRosNodeHandler__->advertise<std_msgs::Int64MultiArray>(topic_name, buffer_num, false);

      if(ros_message_type == "UInt8") publisher = __MasterRosNodeHandler__->advertise<std_msgs::UInt8>(topic_name, buffer_num, false);
      if(ros_message_type == "UInt8MultiArray") publisher = __MasterRosNodeHandler__->advertise<std_msgs::UInt8MultiArray>(topic_name, buffer_num, false);
      if(ros_message_type == "UInt16") publisher = __MasterRosNodeHandler__->advertise<std_msgs::UInt16>(topic_name, buffer_num, false);
      if(ros_message_type == "UInt16MultiArray") publisher = __MasterRosNodeHandler__->advertise<std_msgs::UInt16MultiArray>(topic_name, buffer_num, false);
      if(ros_message_type == "UInt32") publisher = __MasterRosNodeHandler__->advertise<std_msgs::UInt32>(topic_name, buffer_num, false);
      if(ros_message_type == "UInt32MultiArray") publisher = __MasterRosNodeHandler__->advertise<std_msgs::UInt32MultiArray>(topic_name, buffer_num, false);
      if(ros_message_type == "UInt64") publisher = __MasterRosNodeHandler__->advertise<std_msgs::UInt64>(topic_name, buffer_num, false);
      if(ros_message_type == "UInt64MultiArray") publisher = __MasterRosNodeHandler__->advertise<std_msgs::UInt64MultiArray>(topic_name, buffer_num, false);

      if(ros_message_type == "Float32") publisher = __MasterRosNodeHandler__->advertise<std_msgs::Float32>(topic_name, buffer_num, false);
      if(ros_message_type == "Float32MultiArray") publisher = __MasterRosNodeHandler__->advertise<std_msgs::Float32MultiArray>(topic_name, buffer_num, false);
      if(ros_message_type == "Float64") publisher = __MasterRosNodeHandler__->advertise<std_msgs::Float64>(topic_name, buffer_num, false);
      if(ros_message_type == "Float64MultiArray") publisher = __MasterRosNodeHandler__->advertise<std_msgs::Float64MultiArray>(topic_name, buffer_num, false);

    }

    last_message_time = ros::Time::now();    
    this->BufferedNode::initialize();
    cout << getName().c_str() << " initialized" << endl;
  }


  // dynamic input-port translation
  virtual int translateInput(string inputName) 
  {
    if (inputName == "INPUT") {
      return inputID = addInput(inputName);
    }
    else {
      throw new NodeException(this, inputName + " is not supported.", __FILE__, __LINE__);
    }
  }

  // process per one iteration
  void calculate(int output_id, int count, Buffer &out)  
  {    

    ObjectRef input_ptr;
    
    if (inputID != -1){
      input_ptr = getInput(inputID, count);
      out[count] = input_ptr;
    }else{
      return;
    }
    
    ros::spinOnce();
    ros::Time current_time = ros::Time::now();
    ros::Rate loop_rate(ros_loop_rate);
    
    if( max_frequency > 0 && (current_time - last_message_time).toSec() < 1.0/max_frequency ) return;
    

    last_message_time = current_time;

    if (typeid(*input_ptr) == typeid(Bool)) {
      if(ros_message_type == "Bool"){      
	std_msgs::Bool Msg;
	Msg.data = dereference_cast<bool>(input_ptr);
	publisher.publish(Msg);
	if (enable_debug == true) printf("%s Published %s\n", getName().c_str(), ros_message_type.c_str());
      }else{
	if (enable_debug == true) printf("%s node : Input is not matched to %s.\n", getName().c_str(), ros_message_type.c_str());
      }
    }

    if (typeid(*input_ptr) == typeid(String)) {
      if(ros_message_type == "String"){      
	std_msgs::String Msg;
	Msg.data = object_cast<String>(input_ptr);
	publisher.publish(Msg);
	if (enable_debug == true) printf("%s Published %s\n", getName().c_str(), ros_message_type.c_str());
      }else{
	if (enable_debug == true) printf("%s node : Input is not matched to %s.\n", getName().c_str(), ros_message_type.c_str());
      }
    }
    
    if (typeid(*input_ptr) == typeid(Int)) {

      bool published_flag = false;

      if(ros_message_type == "Int8"){      
	std_msgs::Int8 Msg;
	Msg.data = (signed char)(dereference_cast<int>(input_ptr));
	publisher.publish(Msg);
	if (enable_debug == true) printf("%s Published %s\n", getName().c_str(), ros_message_type.c_str());
	published_flag = true;
      }
      if(ros_message_type == "Int16"){      
	std_msgs::Int16 Msg;
	Msg.data = (signed short)(dereference_cast<int>(input_ptr));
	publisher.publish(Msg);
	if (enable_debug == true) printf("%s Published %s\n", getName().c_str(), ros_message_type.c_str());
	published_flag = true;
      }
      if(ros_message_type == "Int32"){      
	std_msgs::Int32 Msg;
	Msg.data = (signed int)(dereference_cast<int>(input_ptr));
	publisher.publish(Msg);
	if (enable_debug == true) printf("%s Published %s\n", getName().c_str(), ros_message_type.c_str());
	published_flag = true;
      }
      if(ros_message_type == "Int64"){      
	std_msgs::Int64 Msg;
	Msg.data = (signed long)(dereference_cast<int>(input_ptr));
	publisher.publish(Msg);
	if (enable_debug == true) printf("%s Published %s\n", getName().c_str(), ros_message_type.c_str());
	published_flag = true;
      }

      if(ros_message_type == "UInt8"){      
	std_msgs::UInt8 Msg;
	Msg.data = (unsigned char)(dereference_cast<int>(input_ptr));
	publisher.publish(Msg);
	if (enable_debug == true) printf("%s Published %s\n", getName().c_str(), ros_message_type.c_str());
	published_flag = true;
      }
      if(ros_message_type == "UInt16"){      
	std_msgs::UInt16 Msg;
	Msg.data = (unsigned short)(dereference_cast<int>(input_ptr));
	publisher.publish(Msg);
	if (enable_debug == true) printf("%s Published %s\n", getName().c_str(), ros_message_type.c_str());
	published_flag = true;
      }
      if(ros_message_type == "UInt32"){      
	std_msgs::UInt32 Msg;
	Msg.data = (unsigned int)(dereference_cast<int>(input_ptr));
	publisher.publish(Msg);
	if (enable_debug == true) printf("%s Published %s\n", getName().c_str(), ros_message_type.c_str());
	published_flag = true;
      }
      if(ros_message_type == "UInt64"){      
	std_msgs::UInt64 Msg;
	Msg.data = (unsigned long)(dereference_cast<int>(input_ptr));
	publisher.publish(Msg);
	if (enable_debug == true) printf("%s Published %s\n", getName().c_str(), ros_message_type.c_str());
	published_flag = true;
      }

      if(published_flag == false){
	if (enable_debug == true) printf("%s node : Input is not matched to %s.\n", getName().c_str(), ros_message_type.c_str());
      }

    }
    
    if (typeid(*input_ptr) == typeid(Float)) {

      bool published_flag = false;

      if(ros_message_type == "Float32"){      
	std_msgs::Float32 Msg;
	Msg.data = (float)(dereference_cast<float>(input_ptr));
	publisher.publish(Msg);
	if (enable_debug == true) printf("%s Published %s\n", getName().c_str(), ros_message_type.c_str());
	published_flag = true;
      }

      if(ros_message_type == "Float64"){      
	std_msgs::Float64 Msg;
	Msg.data = (double)(dereference_cast<float>(input_ptr));
	publisher.publish(Msg);
	if (enable_debug == true) printf("%s Published %s\n", getName().c_str(), ros_message_type.c_str());
	published_flag = true;
      }

      if(published_flag == false){
	if (enable_debug == true) printf("%s node : Input is not matched to %s.\n", getName().c_str(), ros_message_type.c_str());
      }

    }

    if (typeid(*input_ptr) == typeid(Vector<int>)) {

      bool published_flag = false;
      RCPtr<Vector<int> > input = input_ptr;

      if(ros_message_type == "Int8MultiArray"){      
	std_msgs::Int8MultiArray Msg;
	Msg.data.resize((*input).size());
	for(int cnt = 0; cnt < (*input).size(); cnt++) Msg.data[cnt] = (signed char)(*input)[cnt];
	Msg.layout.data_offset = 0;
	Msg.layout.dim.resize(1);
	Msg.layout.dim[0].label = "vector_data";
	Msg.layout.dim[0].size = int(Msg.data.size());
	Msg.layout.dim[0].stride = int(Msg.data.size());
	publisher.publish(Msg);
	if (enable_debug == true) printf("%s Published %s\n", getName().c_str(), ros_message_type.c_str());
	published_flag = true;
      }

      if(ros_message_type == "Int16MultiArray"){      
	std_msgs::Int16MultiArray Msg;
	Msg.data.resize((*input).size());
	for(int cnt = 0; cnt < (*input).size(); cnt++) Msg.data[cnt] = (signed short)(*input)[cnt];
	Msg.layout.data_offset = 0;
	Msg.layout.dim.resize(1);
	Msg.layout.dim[0].label = "vector_data";
	Msg.layout.dim[0].size = int(Msg.data.size());
	Msg.layout.dim[0].stride = int(Msg.data.size());
	publisher.publish(Msg);
	if (enable_debug == true) printf("%s Published %s\n", getName().c_str(), ros_message_type.c_str());
	published_flag = true;
      }

      if(ros_message_type == "Int32MultiArray"){      
	std_msgs::Int32MultiArray Msg;
	Msg.data.resize((*input).size());
	for(int cnt = 0; cnt < (*input).size(); cnt++) Msg.data[cnt] = (signed int)(*input)[cnt];
	Msg.layout.data_offset = 0;
	Msg.layout.dim.resize(1);
	Msg.layout.dim[0].label = "vector_data";
	Msg.layout.dim[0].size = int(Msg.data.size());
	Msg.layout.dim[0].stride = int(Msg.data.size());
	publisher.publish(Msg);
	if (enable_debug == true) printf("%s Published %s\n", getName().c_str(), ros_message_type.c_str());
	published_flag = true;
      }

      if(ros_message_type == "Int64MultiArray"){      
	std_msgs::Int64MultiArray Msg;
	Msg.data.resize((*input).size());
	for(int cnt = 0; cnt < (*input).size(); cnt++) Msg.data[cnt] = (signed long)(*input)[cnt];
	Msg.layout.data_offset = 0;
	Msg.layout.dim.resize(1);
	Msg.layout.dim[0].label = "vector_data";
	Msg.layout.dim[0].size = int(Msg.data.size());
	Msg.layout.dim[0].stride = int(Msg.data.size());
	publisher.publish(Msg);
	if (enable_debug == true) printf("%s Published %s\n", getName().c_str(), ros_message_type.c_str());
	published_flag = true;
      }

      if(ros_message_type == "UInt8MultiArray"){      
	std_msgs::UInt8MultiArray Msg;
	Msg.data.resize((*input).size());
	for(int cnt = 0; cnt < (*input).size(); cnt++) Msg.data[cnt] = (unsigned char)(*input)[cnt];
	Msg.layout.data_offset = 0;
	Msg.layout.dim.resize(1);
	Msg.layout.dim[0].label = "vector_data";
	Msg.layout.dim[0].size = int(Msg.data.size());
	Msg.layout.dim[0].stride = int(Msg.data.size());
	publisher.publish(Msg);
	if (enable_debug == true) printf("%s Published %s\n", getName().c_str(), ros_message_type.c_str());
	published_flag = true;
      }

      if(ros_message_type == "UInt16MultiArray"){      
	std_msgs::UInt16MultiArray Msg;
	Msg.data.resize((*input).size());
	for(int cnt = 0; cnt < (*input).size(); cnt++) Msg.data[cnt] = (unsigned short)(*input)[cnt];
	Msg.layout.data_offset = 0;
	Msg.layout.dim.resize(1);
	Msg.layout.dim[0].label = "vector_data";
	Msg.layout.dim[0].size = int(Msg.data.size());
	Msg.layout.dim[0].stride = int(Msg.data.size());
	publisher.publish(Msg);
	if (enable_debug == true) printf("%s Published %s\n", getName().c_str(), ros_message_type.c_str());
	published_flag = true;
      }

      if(ros_message_type == "UInt32MultiArray"){      
	std_msgs::UInt32MultiArray Msg;
	Msg.data.resize((*input).size());
	for(int cnt = 0; cnt < (*input).size(); cnt++) Msg.data[cnt] = (unsigned int)(*input)[cnt];
	Msg.layout.data_offset = 0;
	Msg.layout.dim.resize(1);
	Msg.layout.dim[0].label = "vector_data";
	Msg.layout.dim[0].size = int(Msg.data.size());
	Msg.layout.dim[0].stride = int(Msg.data.size());
	publisher.publish(Msg);
	if (enable_debug == true) printf("%s Published %s\n", getName().c_str(), ros_message_type.c_str());
	published_flag = true;
      }

      if(ros_message_type == "UInt64MultiArray"){      
	std_msgs::UInt64MultiArray Msg;
	Msg.data.resize((*input).size());
	for(int cnt = 0; cnt < (*input).size(); cnt++) Msg.data[cnt] = (unsigned long)(*input)[cnt];
	Msg.layout.data_offset = 0;
	Msg.layout.dim.resize(1);
	Msg.layout.dim[0].label = "vector_data";
	Msg.layout.dim[0].size = int(Msg.data.size());
	Msg.layout.dim[0].stride = int(Msg.data.size());
	publisher.publish(Msg);
	if (enable_debug == true) printf("%s Published %s\n", getName().c_str(), ros_message_type.c_str());
	published_flag = true;
      }

      if(published_flag == false){
	if (enable_debug == true) printf("%s node : Input is not matched to %s.\n", getName().c_str(), ros_message_type.c_str());
      }

    }

    if (typeid(*input_ptr) == typeid(Vector<float>)) {

      bool published_flag = false;
      RCPtr<Vector<float> > input = input_ptr;

      if(ros_message_type == "Float32MultiArray"){      
	std_msgs::Float32MultiArray Msg;
	Msg.data.resize((*input).size());
	for(int cnt = 0; cnt < (*input).size(); cnt++) Msg.data[cnt] = (float)(*input)[cnt];
	Msg.layout.data_offset = 0;
	Msg.layout.dim.resize(1);
	Msg.layout.dim[0].label = "vector_data";
	Msg.layout.dim[0].size = int(Msg.data.size());
	Msg.layout.dim[0].stride = int(Msg.data.size());
	publisher.publish(Msg);
	if (enable_debug == true) printf("%s Published %s\n", getName().c_str(), ros_message_type.c_str());
	published_flag = true;
      }

      if(ros_message_type == "Float64MultiArray"){      
	std_msgs::Float64MultiArray Msg;
	Msg.data.resize((*input).size());
	for(int cnt = 0; cnt < (*input).size(); cnt++) Msg.data[cnt] = (double)(*input)[cnt];
	Msg.layout.data_offset = 0;
	Msg.layout.dim.resize(1);
	Msg.layout.dim[0].label = "vector_data";
	Msg.layout.dim[0].size = int(Msg.data.size());
	Msg.layout.dim[0].stride = int(Msg.data.size());
	publisher.publish(Msg);
	if (enable_debug == true) printf("%s Published %s\n", getName().c_str(), ros_message_type.c_str());
	published_flag = true;
      }

      if(published_flag == false){
	if (enable_debug == true) printf("%s node : Input is not matched to %s.\n", getName().c_str(), ros_message_type.c_str());
      }

    }

    if (typeid(*input_ptr) == typeid(Matrix<int>)) {

      bool published_flag = false;
      RCPtr<Matrix<int> > input = input_ptr;

      if(ros_message_type == "Int8MultiArray"){      
	std_msgs::Int8MultiArray Msg;
	Msg.layout.data_offset = 0;
	Msg.layout.dim.resize(2);
	Msg.layout.dim[0].label = "rows";
	Msg.layout.dim[0].size = int(input->nrows());
	Msg.layout.dim[0].stride = int(input->ncols()) * int(input->nrows());
	Msg.layout.dim[1].label = "cols";
	Msg.layout.dim[1].size = int(input->ncols());
	Msg.layout.dim[1].stride = int(input->ncols());
	Msg.data.resize(Msg.layout.dim[0].stride);
	for(int row = 0; row < input->nrows(); row++){
	  for(int col = 0; col < input->ncols(); col++){
	    Msg.data[col + row * input->ncols()] = (signed char)(*input)[row][col];
	  }	
	}	
	publisher.publish(Msg);
	if (enable_debug == true) printf("%s Published %s\n", getName().c_str(), ros_message_type.c_str());
	published_flag = true;
      }

      if(ros_message_type == "Int16MultiArray"){      
	std_msgs::Int16MultiArray Msg;
	Msg.layout.data_offset = 0;
	Msg.layout.dim.resize(2);
	Msg.layout.dim[0].label = "rows";
	Msg.layout.dim[0].size = int(input->nrows());
	Msg.layout.dim[0].stride = int(input->ncols()) * int(input->nrows());
	Msg.layout.dim[1].label = "cols";
	Msg.layout.dim[1].size = int(input->ncols());
	Msg.layout.dim[1].stride = int(input->ncols());
	Msg.data.resize(Msg.layout.dim[0].stride);
	for(int row = 0; row < input->nrows(); row++){
	  for(int col = 0; col < input->ncols(); col++){
	    Msg.data[col + row * input->ncols()] = (signed short)(*input)[row][col];
	  }	
	}	
	publisher.publish(Msg);
	if (enable_debug == true) printf("%s Published %s\n", getName().c_str(), ros_message_type.c_str());
	published_flag = true;
      }

      if(ros_message_type == "Int32MultiArray"){      
	std_msgs::Int32MultiArray Msg;
	Msg.layout.data_offset = 0;
	Msg.layout.dim.resize(2);
	Msg.layout.dim[0].label = "rows";
	Msg.layout.dim[0].size = int(input->nrows());
	Msg.layout.dim[0].stride = int(input->ncols()) * int(input->nrows());
	Msg.layout.dim[1].label = "cols";
	Msg.layout.dim[1].size = int(input->ncols());
	Msg.layout.dim[1].stride = int(input->ncols());
	Msg.data.resize(Msg.layout.dim[0].stride);
	for(int row = 0; row < input->nrows(); row++){
	  for(int col = 0; col < input->ncols(); col++){
	    Msg.data[col + row * input->ncols()] = (signed int)(*input)[row][col];
	  }	
	}	
	publisher.publish(Msg);
	if (enable_debug == true) printf("%s Published %s\n", getName().c_str(), ros_message_type.c_str());
	published_flag = true;
      }

      if(ros_message_type == "Int64MultiArray"){      
	std_msgs::Int64MultiArray Msg;
	Msg.layout.data_offset = 0;
	Msg.layout.dim.resize(2);
	Msg.layout.dim[0].label = "rows";
	Msg.layout.dim[0].size = int(input->nrows());
	Msg.layout.dim[0].stride = int(input->ncols()) * int(input->nrows());
	Msg.layout.dim[1].label = "cols";
	Msg.layout.dim[1].size = int(input->ncols());
	Msg.layout.dim[1].stride = int(input->ncols());
	Msg.data.resize(Msg.layout.dim[0].stride);
	for(int row = 0; row < input->nrows(); row++){
	  for(int col = 0; col < input->ncols(); col++){
	    Msg.data[col + row * input->ncols()] = (signed long)(*input)[row][col];
	  }	
	}	
	publisher.publish(Msg);
	if (enable_debug == true) printf("%s Published %s\n", getName().c_str(), ros_message_type.c_str());
	published_flag = true;
      }

      if(ros_message_type == "UInt8MultiArray"){      
	std_msgs::UInt8MultiArray Msg;
	Msg.layout.data_offset = 0;
	Msg.layout.dim.resize(2);
	Msg.layout.dim[0].label = "rows";
	Msg.layout.dim[0].size = int(input->nrows());
	Msg.layout.dim[0].stride = int(input->ncols()) * int(input->nrows());
	Msg.layout.dim[1].label = "cols";
	Msg.layout.dim[1].size = int(input->ncols());
	Msg.layout.dim[1].stride = int(input->ncols());
	Msg.data.resize(Msg.layout.dim[0].stride);
	for(int row = 0; row < input->nrows(); row++){
	  for(int col = 0; col < input->ncols(); col++){
	    Msg.data[col + row * input->ncols()] = (unsigned char)(*input)[row][col];
	  }	
	}	
	publisher.publish(Msg);
	if (enable_debug == true) printf("%s Published %s\n", getName().c_str(), ros_message_type.c_str());
	published_flag = true;
      }

      if(ros_message_type == "UInt16MultiArray"){      
	std_msgs::UInt16MultiArray Msg;
	Msg.layout.data_offset = 0;
	Msg.layout.dim.resize(2);
	Msg.layout.dim[0].label = "rows";
	Msg.layout.dim[0].size = int(input->nrows());
	Msg.layout.dim[0].stride = int(input->ncols()) * int(input->nrows());
	Msg.layout.dim[1].label = "cols";
	Msg.layout.dim[1].size = int(input->ncols());
	Msg.layout.dim[1].stride = int(input->ncols());
	Msg.data.resize(Msg.layout.dim[0].stride);
	for(int row = 0; row < input->nrows(); row++){
	  for(int col = 0; col < input->ncols(); col++){
	    Msg.data[col + row * input->ncols()] = (unsigned short)(*input)[row][col];
	  }	
	}	
	publisher.publish(Msg);
	if (enable_debug == true) printf("%s Published %s\n", getName().c_str(), ros_message_type.c_str());
	published_flag = true;
      }

      if(ros_message_type == "UInt32MultiArray"){      
	std_msgs::UInt32MultiArray Msg;
	Msg.layout.data_offset = 0;
	Msg.layout.dim.resize(2);
	Msg.layout.dim[0].label = "rows";
	Msg.layout.dim[0].size = int(input->nrows());
	Msg.layout.dim[0].stride = int(input->ncols()) * int(input->nrows());
	Msg.layout.dim[1].label = "cols";
	Msg.layout.dim[1].size = int(input->ncols());
	Msg.layout.dim[1].stride = int(input->ncols());
	Msg.data.resize(Msg.layout.dim[0].stride);
	for(int row = 0; row < input->nrows(); row++){
	  for(int col = 0; col < input->ncols(); col++){
	    Msg.data[col + row * input->ncols()] = (unsigned int)(*input)[row][col];
	  }	
	}	
	publisher.publish(Msg);
	if (enable_debug == true) printf("%s Published %s\n", getName().c_str(), ros_message_type.c_str());
	published_flag = true;
      }

      if(ros_message_type == "UInt64MultiArray"){      
	std_msgs::UInt64MultiArray Msg;
	Msg.layout.data_offset = 0;
	Msg.layout.dim.resize(2);
	Msg.layout.dim[0].label = "rows";
	Msg.layout.dim[0].size = int(input->nrows());
	Msg.layout.dim[0].stride = int(input->ncols()) * int(input->nrows());
	Msg.layout.dim[1].label = "cols";
	Msg.layout.dim[1].size = int(input->ncols());
	Msg.layout.dim[1].stride = int(input->ncols());
	Msg.data.resize(Msg.layout.dim[0].stride);
	for(int row = 0; row < input->nrows(); row++){
	  for(int col = 0; col < input->ncols(); col++){
	    Msg.data[col + row * input->ncols()] = (unsigned long)(*input)[row][col];
	  }	
	}	
	publisher.publish(Msg);
	if (enable_debug == true) printf("%s Published %s\n", getName().c_str(), ros_message_type.c_str());
	published_flag = true;
      }

      if(published_flag == false){
	if (enable_debug == true) printf("%s node : Input is not matched to %s.\n", getName().c_str(), ros_message_type.c_str());
      }

    }

    if (typeid(*input_ptr) == typeid(Matrix<float>)) {

      bool published_flag = false;
      RCPtr<Matrix<float> > input = input_ptr;

      if(ros_message_type == "Float32MultiArray"){      
	std_msgs::Float32MultiArray Msg;
	Msg.layout.data_offset = 0;
	Msg.layout.dim.resize(2);
	Msg.layout.dim[0].label = "rows";
	Msg.layout.dim[0].size = int(input->nrows());
	Msg.layout.dim[0].stride = int(input->ncols()) * int(input->nrows());
	Msg.layout.dim[1].label = "cols";
	Msg.layout.dim[1].size = int(input->ncols());
	Msg.layout.dim[1].stride = int(input->ncols());
	Msg.data.resize(Msg.layout.dim[0].stride);
	for(int row = 0; row < input->nrows(); row++){
	  for(int col = 0; col < input->ncols(); col++){
	    Msg.data[col + row * input->ncols()] = (float)(*input)[row][col];
	  }	
	}	
	publisher.publish(Msg);
	if (enable_debug == true) printf("%s Published %s\n", getName().c_str(), ros_message_type.c_str());
	published_flag = true;
      }

      if(ros_message_type == "Float64MultiArray"){      
	std_msgs::Float64MultiArray Msg;
	Msg.layout.data_offset = 0;
	Msg.layout.dim.resize(2);
	Msg.layout.dim[0].label = "rows";
	Msg.layout.dim[0].size = int(input->nrows());
	Msg.layout.dim[0].stride = int(input->ncols()) * int(input->nrows());
	Msg.layout.dim[1].label = "cols";
	Msg.layout.dim[1].size = int(input->ncols());
	Msg.layout.dim[1].stride = int(input->ncols());
	Msg.data.resize(Msg.layout.dim[0].stride);
	for(int row = 0; row < input->nrows(); row++){
	  for(int col = 0; col < input->ncols(); col++){
	    Msg.data[col + row * input->ncols()] = (double)(*input)[row][col];
	  }	
	}	
	publisher.publish(Msg);
	if (enable_debug == true) printf("%s Published %s\n", getName().c_str(), ros_message_type.c_str());
	published_flag = true;
      }

      if(published_flag == false){
	if (enable_debug == true) printf("%s node : Input is not matched to %s.\n", getName().c_str(), ros_message_type.c_str());
      }

    }


    if (typeid(*input_ptr) == typeid(Matrix<complex<float> >)) {

      bool published_flag = false;
      RCPtr<Matrix<complex<float> > > input = input_ptr;

      if(ros_message_type == "Float32MultiArray"){      
	std_msgs::Float32MultiArray Msg;
	Msg.layout.data_offset = 0;
	Msg.layout.dim.resize(3);
	Msg.layout.dim[0].label = "rows";
	Msg.layout.dim[0].size = int(input->nrows());
	Msg.layout.dim[0].stride = int(input->ncols()) * int(input->nrows());
	Msg.layout.dim[1].label = "cols";
	Msg.layout.dim[1].size = int(input->ncols());
	Msg.layout.dim[1].stride = int(input->ncols());
	Msg.layout.dim[2].label = "complex";
	Msg.layout.dim[2].size = 2;
	Msg.layout.dim[2].stride = 2;
	Msg.data.resize(Msg.layout.dim[0].stride);
	for(int row = 0; row < input->nrows(); row++){
	  for(int col = 0; col < input->ncols(); col++){
	    Msg.data[col * 2 + row * 2 * input->ncols() + 0] = (float)(*input)[row][col].real();
	    Msg.data[col * 2 + row * 2 * input->ncols() + 1] = (float)(*input)[row][col].imag();
	  }	
	}	
	publisher.publish(Msg);
	if (enable_debug == true) printf("%s Published %s\n", getName().c_str(), ros_message_type.c_str());
	published_flag = true;
      }

      if(ros_message_type == "Float64MultiArray"){      
	std_msgs::Float64MultiArray Msg;
	Msg.layout.data_offset = 0;
	Msg.layout.dim.resize(3);
	Msg.layout.dim[0].label = "rows";
	Msg.layout.dim[0].size = int(input->nrows());
	Msg.layout.dim[0].stride = int(input->ncols()) * int(input->nrows());
	Msg.layout.dim[1].label = "cols";
	Msg.layout.dim[1].size = int(input->ncols());
	Msg.layout.dim[1].stride = int(input->ncols());
	Msg.layout.dim[2].label = "complex";
	Msg.layout.dim[2].size = 2;
	Msg.layout.dim[2].stride = 2;
	Msg.data.resize(Msg.layout.dim[0].stride);
	for(int row = 0; row < input->nrows(); row++){
	  for(int col = 0; col < input->ncols(); col++){
	    Msg.data[col * 2 + row * 2 * input->ncols() + 0] = (double)(*input)[row][col].real();
	    Msg.data[col * 2 + row * 2 * input->ncols() + 1] = (double)(*input)[row][col].imag();
	  }	
	}	
	publisher.publish(Msg);
	if (enable_debug == true) printf("%s Published %s\n", getName().c_str(), ros_message_type.c_str());
	published_flag = true;
      }

      if(published_flag == false){
	if (enable_debug == true) printf("%s node : Input is not matched to %s.\n", getName().c_str(), ros_message_type.c_str());
      }

    }

    
    loop_rate.sleep();

  }
      
  IN_ORDER_NODE_SPEEDUP(RosStdMsgsPublisher)
    
};

#endif
