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
#include <dynamic_reconfigure/server.h>
#include <hark_params/DynReconfHarkParamsConfig.h>
#include "HarkRosGlobals.h"
#include <algorithm>
#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>
#include "HarkParamsObjects.h"

using namespace std;
using namespace FD;

class callbackFuncHarkParamsDynReconf
{
public:
  Vector<ObjectRef> vec_LocalizeMUSIC;
  Vector<ObjectRef> vec_SourceTracker;
  Vector<ObjectRef> vec_HRLE;
  ros::Duration *dur;
  callbackFuncHarkParamsDynReconf(float duration){
    dur = new ros::Duration(duration);
    vec_LocalizeMUSIC = *new Vector<ObjectRef>;
    vec_SourceTracker = *new Vector<ObjectRef>;
    vec_HRLE = *new Vector<ObjectRef>;
    vec_LocalizeMUSIC.resize(0);
    vec_SourceTracker.resize(0);
    vec_HRLE.resize(0);    
  }
  void cb(hark_params::DynReconfHarkParamsConfig &config, uint32_t level)
  {
    RCPtr<ParamsLocalizeMUSIC> value_LocalizeMUSIC(new ParamsLocalizeMUSIC());
    RCPtr<ParamsSourceTracker> value_SourceTracker(new ParamsSourceTracker());
    RCPtr<ParamsHRLE> value_HRLE(new ParamsHRLE());
    value_LocalizeMUSIC->num_source            = (int)config.LocalizeMUSIC_num_source;
    value_LocalizeMUSIC->min_deg               = (int)config.LocalizeMUSIC_min_deg;
    value_LocalizeMUSIC->max_deg               = (int)config.LocalizeMUSIC_max_deg;
    value_LocalizeMUSIC->lower_bound_frequency = (int)config.LocalizeMUSIC_lower_bound_frequency;
    value_LocalizeMUSIC->upper_bound_frequency = (int)config.LocalizeMUSIC_upper_bound_frequency;
    value_SourceTracker->thresh                = (float)config.SourceTracker_thresh;
    value_SourceTracker->pause_length          = (float)config.SourceTracker_pause_length;
    value_SourceTracker->min_src_interval      = (float)config.SourceTracker_min_src_interval;
    value_HRLE->lx                             = (float)config.HRLE_lx;
    value_HRLE->time_constant                  = (float)config.HRLE_time_constant;
    vec_LocalizeMUSIC.resize(0);
    vec_SourceTracker.resize(0);
    vec_HRLE.resize(0);
    if((bool)config.HarkParamsDynReconfEnable){	
      vec_LocalizeMUSIC.push_back(value_LocalizeMUSIC);      
      vec_SourceTracker.push_back(value_SourceTracker);      
      vec_HRLE.push_back(value_HRLE);
    }
    dur->sleep();
  }
};

class RosHarkParamsDynReconf;

DECLARE_NODE(RosHarkParamsDynReconf);
/*Node
 *
 * @name RosHarkParamsDynReconf
 * @category HARK:ROS:Utility
 * @description ROS HarkParams dynamic reconfigure node. DO NOT PUT THIS NODE MORE THAN TWO IN THE SAME NETWORK FILE (since single process can generate only one dynamic reconfigure node).
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

class RosHarkParamsDynReconf : public BufferedNode {
  int localizemusicID;
  int sourcetrackerID;
  int hrleID;
  float ros_duration;
  bool enable_debug;
  callbackFuncHarkParamsDynReconf *cbf;

  typedef boost::shared_ptr<dynamic_reconfigure::Server<hark_params::DynReconfHarkParamsConfig> > SmartPtr;

private:
  SmartPtr srvDyn;
  ros::AsyncSpinner *_snum;

public:
  RosHarkParamsDynReconf(string nodeName, ParameterSet params)
    : BufferedNode(nodeName, params)
  {
    localizemusicID    = addOutput("LocalizeMUSIC");
    sourcetrackerID    = addOutput("SourceTracker");
    hrleID             = addOutput("HRLE");
    ros_duration = dereference_cast<float>(parameters.get("ROS_DURATION"));
    enable_debug = dereference_cast<bool>(parameters.get("ENABLE_DEBUG"));
    cbf = new callbackFuncHarkParamsDynReconf(ros_duration);
    inOrder = true;  
  }

  ~RosHarkParamsDynReconf()
  {
    _snum->stop();
  }
  
  virtual void initialize()
  {

    cout << getName() << " initialized..." << endl;

    srvDyn = SmartPtr(new dynamic_reconfigure::Server<hark_params::DynReconfHarkParamsConfig>);
    dynamic_reconfigure::Server<hark_params::DynReconfHarkParamsConfig>::CallbackType fDyn;
    fDyn = boost::bind(&callbackFuncHarkParamsDynReconf::cb, cbf, _1, _2);
    srvDyn->setCallback(fDyn);

    this->BufferedNode::initialize();
    _snum = new ros::AsyncSpinner(1);
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
