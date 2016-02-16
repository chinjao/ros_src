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
 *    list of conditions a
nd the following disclaimer in the documentation and/or other 
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
#include <Matrix.h>
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
#include "trackerCombination/BeatCandidate.h"
#include "HarkRosGlobals.h"
#endif

using namespace std;
using namespace boost;
using namespace FD;

class BeatCandidate;

DECLARE_NODE(BeatCandidate);
/*Node
 *
 * @name BeatCandidate
 * @category HARK:ROS:Sample
 * @description Beat Inter-Onset Interval Publisher
 *
 * @input_name TEMPO
 * @input_type float
 * @input_description beat tempo
 *
 * @input_name BEATDETECTED
 * @input_type bool
 * @input_description beat detected
 *
 * @output_name TEMPO
 * @output_type float
 * @output_description beat tempo
 *
 * @parameter_name TOPIC_NAME
 * @parameter_type string
 * @parameter_value beatCandidate
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
 * @parameter_name TRACKER_ID
 * @parameter_type int
 * @parameter_value 0
 * @parameter_description Published topic name for ROS (AudioFeatures type message)
 * 
END*/

class BeatCandidate : public BufferedNode {
    int TempoID;
    bool BeatDetectedID;

    int TempoOutputID;

#ifdef ENABLE_KROS
	string         topic_name_BeatCandidate_;
	int            buffer_num_;
	float          ros_loop_rate_;
	int            tracker_ID_;
	ros::Publisher pubBeatCandidate_;
#endif

public:
    BeatCandidate(string nodeName, ParameterSet params)
        : BufferedNode(nodeName, params),
          TempoID(-1)
    {
        std::cout << "BeatCandidate::BeatCandidate" << endl;

        TempoID      = addInput("TEMPO");
        BeatDetectedID = addInput("BEATDETECTED");
        TempoOutputID  = addOutput("TEMPO");
  
	#ifdef ENABLE_KROS
				cout << "defined :: KROS " << endl;
		topic_name_BeatCandidate_  = object_cast<String>    (parameters.get("TOPIC_NAME"));
		buffer_num_          = dereference_cast<int>  (parameters.get("BUFFER_NUM"));
		ros_loop_rate_       = dereference_cast<float>(parameters.get("ROS_LOOP_RATE"));
		tracker_ID_          = dereference_cast<int>  (parameters.get("TRACKER_ID"));

	#endif
    }

    virtual ~BeatCandidate()
    {
        std::cout << "BeatCandidate::~BeatCandidate" << endl;
    }

	virtual void initialize()
	{
		cout << " initialized..." << endl;
	#ifdef ENABLE_KROS
		if (TempoID != -1){
			cout<<"debug: "<<topic_name_BeatCandidate_<<" "<<buffer_num_<<endl;
			pubBeatCandidate_ = __MasterRosNodeHandler__->
				advertise<trackerCombination::BeatCandidate>
				(topic_name_BeatCandidate_, buffer_num_);
		}
	#endif

    	this->BufferedNode::initialize();
	}

    void calculate(int output_id, int count, Buffer &out)
    {

			ObjectRef input = this->getInput(this->TempoID, count);
			float tempoInput = dereference_cast<float> (input);
			ObjectRef detected = this->getInput(this->BeatDetectedID, count);
			bool b_beatDetected = dereference_cast<bool> (detected);
    	
	#ifdef ENABLE_KROS
		trackerCombination::BeatCandidate beatCandidate;
		ros::spinOnce();
		beatCandidate.header.stamp    = ros::Time::now();
		ros::Rate loop_rate(ros_loop_rate_);
		
		if(b_beatDetected)
		{
			//cout << "test" << endl;
			beatCandidate.trackerID = tracker_ID_;
			beatCandidate.tempo     = tempoInput;
			beatCandidate.beatFrame = 0;
			pubBeatCandidate_.publish(beatCandidate);
		}
	#endif

        (*outputs[this->TempoOutputID].buffer)[count]  = ObjectRef(Float::alloc((float)tempoInput)) ;
       
	#ifdef ENABLE_KROS 
    	loop_rate.sleep();
	#endif
	}
	IN_ORDER_NODE_SPEEDUP(BeatCandidate)
};
