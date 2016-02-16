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
#include "hark_msgs/HarkIntSrv.h"
#include "HarkRosGlobals.h"
#include <boost/thread.hpp>

using namespace std;
using namespace FD;

class callbackFuncAddHarkIntServer
{
public:
  int value;
  ros::Duration *dur;
  callbackFuncAddHarkIntServer(float duration){
    value = 0;
    dur = new ros::Duration(duration);
  }
  bool cb(hark_msgs::HarkIntSrv::Request& req,
	  hark_msgs::HarkIntSrv::Response& res)
  {
    res.result = req.a + req.b;
    ROS_INFO_STREAM("Request [" << (long int)req.a << " + " << (long int)req.b << " = " << res.result << "] [thread=" << boost::this_thread::get_id() << "]");
    value = (int)res.result;
    dur->sleep();
    return true;
  }
};

class RosAddHarkIntServer;

DECLARE_NODE(RosAddHarkIntServer);
/*Node
 *
 * @name RosAddHarkIntServer
 * @category HARK:ROS:Sample
 * @description ROS server node example. This server takes two integer values from a client and calculates the total.
 *
 * @output_name OUTPUT1
 * @output_type int
 * @output_description Outputs the total value.
 * 
 * @parameter_name TOPIC_NAME
 * @parameter_type string
 * @parameter_value HarkIntSrv
 * @parameter_description Subscribed ROS srv-topic name
 * 
 * @parameter_name ROS_DURATION
 * @parameter_type float
 * @parameter_value 0.001
 * @parameter_description This allows you to specify a duration that you would like to loop at [sec]. Keep this value small.
 *
END*/

class RosAddHarkIntServer : public BufferedNode {
  int output1ID;
  string topic_name;
  float ros_duration;
  callbackFuncAddHarkIntServer *cbf;

private:
  ros::ServiceServer _service;
  ros::AsyncSpinner *_snum;
  
public:
  RosAddHarkIntServer(string nodeName, ParameterSet params)
    : BufferedNode(nodeName, params)
  {
    output1ID    = addOutput("OUTPUT1");
    topic_name  = object_cast<String>(parameters.get("TOPIC_NAME"));
    ros_duration = dereference_cast<float>(parameters.get("ROS_DURATION"));
    cbf = new callbackFuncAddHarkIntServer(ros_duration);
    inOrder = true;
  }

  ~RosAddHarkIntServer()
  {
    _snum->stop();
  }

  virtual void initialize()
  {
    cout << getName() << " initialized..." << endl;
    _service = __MasterRosNodeHandler__->advertiseService(topic_name, &callbackFuncAddHarkIntServer::cb, cbf);
    this->BufferedNode::initialize();
    _snum = new ros::AsyncSpinner(1);
    _snum->start();
  }

  void calculate(int output_id, int count, Buffer &out)
  {
    
    // Main loop routine starts here.

    cout << getName() << " Output : [" << cbf->value << "]" << endl;
    (*(outputs[output1ID].buffer))[count] = ObjectRef(Int::alloc(cbf->value));
    
    // Main loop routine ends here.

  }
};

#endif
