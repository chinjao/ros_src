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
#include <dynamic_reconfigure/server.h>
#include <hark_params/DynReconfHarkIntConfig.h>
#include <algorithm>
#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>

using namespace std;
using namespace FD;

class callbackFuncHarkIntDynReconf
{
public:
  hark_msgs::HarkInt value;
  ros::Duration *dur;
  callbackFuncHarkIntDynReconf(float duration){
    value.data = 0;
    dur = new ros::Duration(duration);
  }
  void cb(hark_params::DynReconfHarkIntConfig &config, uint32_t level)
  {
    ROS_INFO_STREAM("Received [" << config.number << "] [thread=" << boost::this_thread::get_id() << "]");
    value.data = (int)config.number;
    dur->sleep();
  }
};

class RosHarkIntDynReconf;

DECLARE_NODE(RosHarkIntDynReconf);
/*Node
 *
 * @name RosHarkIntDynReconf
 * @category HARK:ROS:Sample
 * @description ROS dynamic reconfigure node example. DO NOT PUT THIS NODE MORE THAN TWO IN THE SAME NETWORK FILE (since single process can generate only one dynamic reconfigure node).
 * 
 * @parameter_name ROS_DURATION
 * @parameter_type float
 * @parameter_value 0.001
 * @parameter_description This allows you to specify a duration that you would like to loop at [sec]. Keep this value small.
 *
 * @output_name OUTPUT1
 * @output_type int
 * @output_description Outputs an integer comming from dynamic reconfigure.
 *
END*/

class RosHarkIntDynReconf : public BufferedNode {
  int output1ID;
  float ros_duration;
  callbackFuncHarkIntDynReconf *cbf;

  typedef boost::shared_ptr<dynamic_reconfigure::Server<hark_params::DynReconfHarkIntConfig> > SmartPtr;

private:
  SmartPtr srvDyn;
  ros::AsyncSpinner *_snum;

public:
  RosHarkIntDynReconf(string nodeName, ParameterSet params)
    : BufferedNode(nodeName, params)
  {
    output1ID    = addOutput("OUTPUT1");
    ros_duration = dereference_cast<float>(parameters.get("ROS_DURATION"));
    cbf = new callbackFuncHarkIntDynReconf(ros_duration);
    inOrder = true;  
  }

  ~RosHarkIntDynReconf()
  {
    _snum->stop();
  }
  
  virtual void initialize()
  {
    cout << getName() << " initialized..." << endl;

    srvDyn = SmartPtr(new dynamic_reconfigure::Server<hark_params::DynReconfHarkIntConfig>);
    dynamic_reconfigure::Server<hark_params::DynReconfHarkIntConfig>::CallbackType fDyn;
    fDyn = boost::bind(&callbackFuncHarkIntDynReconf::cb, cbf, _1, _2);
    srvDyn->setCallback(fDyn);

    this->BufferedNode::initialize();
    _snum = new ros::AsyncSpinner(1);
    _snum->start();
  }

  void calculate(int output_id, int count, Buffer &out)
  {
    
    // Main loop routine starts here.

    cout << getName() << " Subscribed [" << cbf->value.data << "]" << endl;
    (*(outputs[output1ID].buffer))[count] = ObjectRef(Int::alloc(cbf->value.data));

    // Main loop routine ends here.

  }
};

#endif
