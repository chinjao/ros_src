/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2011, Kyoto University and Honda Motor Co.,Ltd. All rights reserved.
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
#include <Vector.h>
#include <math.h>
#include <Source.h>
#include <string>
#include <stdlib.h>
#include <stdio.h>

#include <typeinfo>

#include <vector>
#include <boost/shared_ptr.hpp>
#include <complex>

#ifdef ENABLE_KROS
#include <ros/ros.h>
#include "audioFeatures/AudioFeatures.h"
#include "HarkRosGlobals.h"
#endif

using namespace std;
using namespace boost;
using namespace FD;

class STPMPublisher;

DECLARE_NODE(STPMPublisher);
/*Node
 *
 * @name STPMPublisher
 * @category HARK:ROS:Sample
 * @description STPMVector Publisher (w/ BaetTracker in HARK MUSIC & KROS)
 *
 * @input_name ONSETVector
 * @input_type Vector<float>
 * @input_description beat tempo
 *
 * @input_name STPMVector
 * @input_type Vector<float>
 * @input_description beat tempo
 *
 * @output_name OUTPUT
 * @output_type ObjectRef
 * @output_description This is a dummy output, and it has no mean. Only for an activation of this module.
 * 
 * @parameter_name TOPIC_NAME_AUDIOFEATURES
 * @parameter_type string
 * @parameter_value audioFeatures
 * @parameter_description Published topic name for ROS (AudioFeatures type message)
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
END*/

class STPMPublisher : public BufferedNode {
    int STPMVectorID;
    int OnsetVectorID;
	int outputID;

#ifdef ENABLE_KROS
	string         topic_name_AudioFeatures_;
	int            buffer_num_;
	float          ros_loop_rate_;
	ros::Publisher pubAudioFeatures_;
#endif

public:
    STPMPublisher(string nodeName, ParameterSet params)
        : BufferedNode(nodeName, params),
          STPMVectorID(-1), OnsetVectorID(-1)
    {
        std::cout << "STPMPublisher::STPMPublisher" << endl;

        STPMVectorID  = addInput("STPMVector");
        OnsetVectorID = addInput("ONSETVector");
        outputID      = addOutput("OUTPUT");
        
	#ifdef ENABLE_KROS
		topic_name_AudioFeatures_  = object_cast<String>    (parameters.get("TOPIC_NAME_AUDIOFEATURES"));
		buffer_num_                = dereference_cast<int>  (parameters.get("BUFFER_NUM"));
		ros_loop_rate_             = dereference_cast<float>(parameters.get("ROS_LOOP_RATE"));
	#endif
    }

    virtual ~STPMPublisher()
    {
        std::cout << "STPMPublisher::~STPMPublisher" << endl;
    }

	virtual void initialize()
	{
		cout << "STPMPublisher initialized..." << endl;
	#ifdef ENABLE_KROS
		if (OnsetVectorID != -1){
			pubAudioFeatures_ = __MasterRosNodeHandler__->
				                  advertise<audioFeatures::AudioFeatures>
				                  (topic_name_AudioFeatures_, buffer_num_, false);
		}
	#endif
			
    	this->BufferedNode::initialize();
	}

    void calculate(int output_id, int count, Buffer &out)
    {
		if (OnsetVectorID != -1){
		  ObjectRef input_ptr = getInput(OnsetVectorID, count);
		  out[count] = input_ptr;
		}else{
		  return;
		}
    	
	#ifdef ENABLE_KROS
		audioFeatures::AudioFeatures audioFeatrues;
		ros::spinOnce();
		audioFeatrues.header.stamp    = ros::Time::now();
		ros::Rate loop_rate(ros_loop_rate_);
		
        RCPtr<Vector<float> > OnsetVectorPtr = this->getInput(this->OnsetVectorID, count);
        Vector<float>& OnsetVector = *OnsetVectorPtr;
        for(int i=0; i < OnsetVector.size(); ++i)
        	audioFeatrues.onset.push_back( OnsetVector[i] );
        
		if (STPMVectorID != -1){
		    RCPtr<Vector<float> > STPMVectorPtr = this->getInput(this->STPMVectorID, count);
		    Vector<float>& STPMVector = *STPMVectorPtr;
			for(int i=0; i < STPMVector.size();  ++i)
				audioFeatrues.stpm.push_back( STPMVector[i] );
		}
		
        pubAudioFeatures_.publish(audioFeatrues);
    	loop_rate.sleep();
	#endif
	}
	//IN_ORDER_NODE_SPEEDUP(STPMPublisher)
};
