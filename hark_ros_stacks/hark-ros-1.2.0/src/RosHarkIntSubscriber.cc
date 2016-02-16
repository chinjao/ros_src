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
#include <../config.h>

#ifdef ENABLE_ROS

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include "hark_msgs/HarkInt.h"
#include "HarkRosGlobals.h"
#include <boost/thread.hpp>

using namespace std;
using namespace FD;

class callbackFuncHarkInt
{
public:
  hark_msgs::HarkInt value;
  ros::Duration *dur;
  callbackFuncHarkInt(float duration){
    value.data = 0;
    dur = new ros::Duration(duration);
  }
  void cb(const hark_msgs::HarkInt::ConstPtr& msg)
  {
    ROS_INFO_STREAM("Received [" << msg->data << "] [thread=" << boost::this_thread::get_id() << "]");
    value.data = (int)msg->data;
    dur->sleep();
  }
};

class RosHarkIntSubscriber;

DECLARE_NODE(RosHarkIntSubscriber);
/*Node
 *
 * @name RosHarkIntSubscriber
 * @category HARK:ROS:Sample
 * @description ROS subscriber node example. This subscribes HarkInt type message.
 *
 * @output_name OUTPUT1
 * @output_type int
 * @output_description This output the count.
 * 
 * @parameter_name TOPIC_NAME
 * @parameter_type string
 * @parameter_value HarkInt
 * @parameter_description Subscribed topic name for ROS
 * 
 * @parameter_name BUFFER_NUM
 * @parameter_type int
 * @parameter_value 100
 * @parameter_description Buffer size for a ROS published message
 * 
 * @parameter_name ROS_DURATION
 * @parameter_type float
 * @parameter_value 0.001
 * @parameter_description This allows you to specify a duration that you would like to loop at [sec]. Keep this value small.
 *
END*/

class RosHarkIntSubscriber : public BufferedNode {
  int output1ID;
  string topic_name;
  int buffer_num;
  float ros_duration;
  callbackFuncHarkInt *cbf;

private:
  ros::Subscriber _sub;
  ros::AsyncSpinner *_snum;
  
public:
  RosHarkIntSubscriber(string nodeName, ParameterSet params)
    : BufferedNode(nodeName, params)
  {
    output1ID    = addOutput("OUTPUT1");
    topic_name  = object_cast<String>(parameters.get("TOPIC_NAME"));
    buffer_num  = dereference_cast<int>(parameters.get("BUFFER_NUM"));
    ros_duration = dereference_cast<float>(parameters.get("ROS_DURATION"));
    cbf = new callbackFuncHarkInt(ros_duration);
    inOrder = true;
    cout << getName() << " constructor end..." << endl;
  }

  ~RosHarkIntSubscriber()
  {
    _snum->stop();
  }

  virtual void initialize()
  {
    cout << getName() << " initialized..." << endl;
    _sub = __MasterRosNodeHandler__->subscribe<hark_msgs::HarkInt>(topic_name, buffer_num, &callbackFuncHarkInt::cb, cbf);

    this->BufferedNode::initialize();
    _snum = new ros::AsyncSpinner(1);
    _snum->start();
  }

  void calculate(int output_id, int count, Buffer &out)
  {
    
    // Main loop routine starts here.

    cout << getName() <<" Subscribed : [" << cbf->value.data << "]" << endl;
    (*(outputs[output1ID].buffer))[count] = ObjectRef(Int::alloc(cbf->value.data));

    // ros::spinOnce();
    
    // Main loop routine ends here.

  }
};

#endif
