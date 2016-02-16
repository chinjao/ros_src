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
#include <Vector.h>
#include <../config.h>

#ifdef ENABLE_ROS

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include "hark_params/LocalizeMUSIC.h"
#include "hark_params/SourceTracker.h"
#include "hark_params/HRLE.h"
#include <boost/thread.hpp>
#include "HarkParamsObjects.h"
#include "HarkRosGlobals.h"

using namespace std;
using namespace FD;


class callbackFuncHarkParamsSubscriber
{
public:
  Vector<ObjectRef> vec_LocalizeMUSIC;
  Vector<ObjectRef> vec_SourceTracker;
  Vector<ObjectRef> vec_HRLE;
  bool enable_debug;
  ros::Duration *dur;
  callbackFuncHarkParamsSubscriber(float duration, bool debug){
    dur = new ros::Duration(duration);
    enable_debug = debug;
    vec_LocalizeMUSIC = *new Vector<ObjectRef>;
    vec_SourceTracker = *new Vector<ObjectRef>;
    vec_HRLE = *new Vector<ObjectRef>;
    vec_LocalizeMUSIC.resize(0);
    vec_SourceTracker.resize(0);
    vec_HRLE.resize(0);    
  }
  void cb_LocalizeMUSIC(const hark_params::LocalizeMUSIC::ConstPtr& msg)
  {
    if(enable_debug)
      ROS_INFO_STREAM("Received [thread=" << boost::this_thread::get_id() << "]");
    RCPtr<ParamsLocalizeMUSIC> value_LocalizeMUSIC(new ParamsLocalizeMUSIC());
    value_LocalizeMUSIC->num_source            = (int)msg->num_source;
    value_LocalizeMUSIC->min_deg               = (int)msg->min_deg;
    value_LocalizeMUSIC->max_deg               = (int)msg->max_deg;
    value_LocalizeMUSIC->lower_bound_frequency = (int)msg->lower_bound_frequency;
    value_LocalizeMUSIC->upper_bound_frequency = (int)msg->upper_bound_frequency;
    vec_LocalizeMUSIC.resize(0);
    vec_LocalizeMUSIC.push_back(value_LocalizeMUSIC);      
    dur->sleep();
  }
  void cb_SourceTracker(const hark_params::SourceTracker::ConstPtr& msg)
  {
    if(enable_debug)
      ROS_INFO_STREAM("Received [thread=" << boost::this_thread::get_id() << "]");
    RCPtr<ParamsSourceTracker> value_SourceTracker(new ParamsSourceTracker());
    value_SourceTracker->thresh                = (float)msg->thresh;
    value_SourceTracker->pause_length          = (float)msg->pause_length;
    value_SourceTracker->min_src_interval      = (float)msg->min_src_interval;
    value_SourceTracker->min_tfindex_interval  = (float)msg->min_tfindex_interval;
    value_SourceTracker->compare_mode          = (Source::CompareMode)msg->compare_mode;
    vec_SourceTracker.resize(0);
    vec_SourceTracker.push_back(value_SourceTracker);      
    dur->sleep();
  }
  void cb_HRLE(const hark_params::HRLE::ConstPtr& msg)
  {
    if(enable_debug)
      ROS_INFO_STREAM("Received [thread=" << boost::this_thread::get_id() << "]");
    RCPtr<ParamsHRLE> value_HRLE(new ParamsHRLE());
    value_HRLE->lx                             = (float)msg->lx;
    value_HRLE->time_constant                  = (float)msg->time_constant;
    vec_HRLE.resize(0);
    vec_HRLE.push_back(value_HRLE);      
    dur->sleep();
  }
};


class RosHarkParamsSubscriber;

DECLARE_NODE(RosHarkParamsSubscriber);
/*Node
 *
 * @name RosHarkParamsSubscriber
 * @category HARK:ROS:Utility
 * @description ROS HarkParams subscriber node. 
 * 
 * @parameter_name TOPIC_NAME_LocalizeMUSIC
 * @parameter_type string
 * @parameter_value HarkParamsLocalizeMUSIC
 * @parameter_description Subscribed topic name for ROS
 * 
 * @parameter_name TOPIC_NAME_SourceTracker
 * @parameter_type string
 * @parameter_value HarkParamsSourceTracker
 * @parameter_description Subscribed topic name for ROS
 * 
 * @parameter_name TOPIC_NAME_HRLE
 * @parameter_type string
 * @parameter_value HarkParamsHRLE
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
 * @parameter_name ENABLE_DEBUG
 * @parameter_type bool
 * @parameter_value false
 * @parameter_list true:false
 * @parameter_description print debug message of this module in case of true.
 *
 * @output_name LocalizeMUSIC
 * @output_type Vector<ObjectRef>
 * @output_description HarkParams of LocalizeMUSIC (ParamsLocalizeMUSIC)
 *
 * @output_name SourceTracker
 * @output_type Vector<ObjectRef>
 * @output_description HarkParams of SourceTracker (ParamsSourceTracker)
 *
 * @output_name HRLE
 * @output_type Vector<ObjectRef>
 * @output_description HarkParams of HRLE (ParamsHRLE)
 *
END*/

class RosHarkParamsSubscriber : public BufferedNode {
  int localizemusicID;
  int sourcetrackerID;
  int hrleID;
  string topic_name_LocalizeMUSIC;
  string topic_name_SourceTracker;
  string topic_name_HRLE;
  int buffer_num;
  float ros_duration;
  bool enable_debug;
  callbackFuncHarkParamsSubscriber *cbf;

private:
  ros::Subscriber _sub_LocalizeMUSIC;
  ros::Subscriber _sub_SourceTracker;
  ros::Subscriber _sub_HRLE;
  ros::AsyncSpinner *_snum;

public:
  RosHarkParamsSubscriber(string nodeName, ParameterSet params)
    : BufferedNode(nodeName, params)
  {
    localizemusicID    = addOutput("LocalizeMUSIC");
    sourcetrackerID    = addOutput("SourceTracker");
    hrleID             = addOutput("HRLE");
    topic_name_LocalizeMUSIC  = object_cast<String>(parameters.get("TOPIC_NAME_LocalizeMUSIC"));
    topic_name_SourceTracker  = object_cast<String>(parameters.get("TOPIC_NAME_SourceTracker"));
    topic_name_HRLE  = object_cast<String>(parameters.get("TOPIC_NAME_HRLE"));
    buffer_num  = dereference_cast<int>(parameters.get("BUFFER_NUM"));    
    ros_duration = dereference_cast<float>(parameters.get("ROS_DURATION"));
    enable_debug = dereference_cast<bool>(parameters.get("ENABLE_DEBUG"));
    cbf = new callbackFuncHarkParamsSubscriber(ros_duration, enable_debug);
    inOrder = true;  
    cout << getName() << " constructor end..." << endl;
  }

  ~RosHarkParamsSubscriber()
  {
    _snum->stop();
  }

  virtual void initialize()
  {
    cout << getName() << " initialized..." << endl;
    _sub_LocalizeMUSIC = __MasterRosNodeHandler__->subscribe<hark_params::LocalizeMUSIC>(topic_name_LocalizeMUSIC, buffer_num, &callbackFuncHarkParamsSubscriber::cb_LocalizeMUSIC, cbf);
    _sub_SourceTracker = __MasterRosNodeHandler__->subscribe<hark_params::SourceTracker>(topic_name_SourceTracker, buffer_num, &callbackFuncHarkParamsSubscriber::cb_SourceTracker, cbf);
    _sub_HRLE          = __MasterRosNodeHandler__->subscribe<hark_params::HRLE>(topic_name_HRLE, buffer_num, &callbackFuncHarkParamsSubscriber::cb_HRLE, cbf);
    this->BufferedNode::initialize();
    _snum = new ros::AsyncSpinner(3);
    _snum->start();

  }

  void calculate(int output_id, int count, Buffer &out)
  {
    
    // Main loop routine starts here.

    Vector<ObjectRef>& out_LocalizeMUSIC = *new Vector<ObjectRef>;
    (*(outputs[localizemusicID].buffer))[count] = &out_LocalizeMUSIC;

    Vector<ObjectRef>& out_SourceTracker = *new Vector<ObjectRef>;
    (*(outputs[sourcetrackerID].buffer))[count] = &out_SourceTracker;

    Vector<ObjectRef>& out_HRLE = *new Vector<ObjectRef>;
    (*(outputs[hrleID].buffer))[count] = &out_HRLE;

    if(cbf->vec_LocalizeMUSIC.size()){
      out_LocalizeMUSIC.push_back(cbf->vec_LocalizeMUSIC[0]);
      cbf->vec_LocalizeMUSIC.resize(0);
    }

    if(cbf->vec_SourceTracker.size()){
      out_SourceTracker.push_back(cbf->vec_SourceTracker[0]);
      cbf->vec_SourceTracker.resize(0);
    }

    if(cbf->vec_HRLE.size()){
      out_HRLE.push_back(cbf->vec_HRLE[0]);
      cbf->vec_HRLE.resize(0);
    }
    
    if(enable_debug) cout << getName() << " Subscribed [" << count << "]" << endl;

    // Main loop routine ends here.

  }
};

#endif
