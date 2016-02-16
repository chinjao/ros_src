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
#include "TimeStamp.h"
#include <math.h>
#include <vector>

using namespace std;
using namespace FD;

class RosHarkMsgsPublisher;

DECLARE_NODE(RosHarkMsgsPublisher)
/*Node
 *
 * @name RosHarkMsgsPublisher
 * @category HARK:ROS:IO
 * @description ROS msg publisher node for hark common messages (hark_msgs).
 *
 * @parameter_name ADVANCE
 * @parameter_type int
 * @parameter_value 160
 * @parameter_description Shift sample number for sliding spectrum analysis.
 *
 * @parameter_name ENABLE_DEBUG
 * @parameter_type bool
 * @parameter_value false
 * @parameter_list true:false
 * @parameter_description print debug message of this module in case of true.
 * 
 * @parameter_name TOPIC_NAME_HARKWAVE
 * @parameter_type string
 * @parameter_value HarkWave
 * @parameter_description Published topic name for ROS (HarkWave type message)
 * 
 * @parameter_name TOPIC_NAME_HARKFFT
 * @parameter_type string
 * @parameter_value HarkFFT
 * @parameter_description Published topic name for ROS (HarkFFT type message)
 * 
 * @parameter_name TOPIC_NAME_HARKFEATURE
 * @parameter_type string
 * @parameter_value HarkFeature
 * @parameter_description Published topic name for ROS (HarkFeature type message)
 * 
 * @parameter_name TOPIC_NAME_HARKSOURCE
 * @parameter_type string
 * @parameter_value HarkSource
 * @parameter_description Published topic name for ROS (HarkSource type message)
 * 
 * @parameter_name TOPIC_NAME_HARKSRCWAVE
 * @parameter_type string
 * @parameter_value HarkSrcWave
 * @parameter_description Published topic name for ROS (HarkSrcWave type message)
 * 
 * @parameter_name TOPIC_NAME_HARKSRCFFT
 * @parameter_type string
 * @parameter_value HarkSrcFFT
 * @parameter_description Published topic name for ROS (HarkSrcFFT type message)
 * 
 * @parameter_name TOPIC_NAME_HARKSRCFEATURE
 * @parameter_type string
 * @parameter_value HarkSrcFeature
 * @parameter_description Published topic name for ROS (HarkSrcFeature type message)
 * 
 * @parameter_name TOPIC_NAME_HARKSRCFEATUREMFM
 * @parameter_type string
 * @parameter_value HarkSrcFeatureMFM
 * @parameter_description Published topic name for ROS (HarkSrcFeatureMFM type message)
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
 * @parameter_name TIMESTAMP_TYPE
 * @parameter_type string
 * @parameter_value ROS_TIME_NOW
 * @parameter_list ROS_TIME_NOW:CONSTANT_INCREMENT
 * @parameter_description Time stamp type. If TIMESTAMP is connected, this is ignored.
 * 
 * @parameter_name SAMPLING_RATE
 * @parameter_type int
 * @parameter_value 16000
 * @parameter_valid TIMESTAMP_TYPE=CONSTANT_INCREMENT
 * @parameter_description The time increment is caluculated as ADVANCE / SAMPLING_RATE
 * 
 * @parameter_name ROS_FRAME_ID
 * @parameter_type string
 * @parameter_value HarkRosFrameID
 * @parameter_description ROS frame_id of the message header
 *
 * @input_name MIC_WAVE
 * @input_type Matrix<float>
 * @input_description Microphone input signals. 
 *
 * @input_name MIC_FFT
 * @input_type Matrix<complex<float> >
 * @input_description Fourier transformed signal before sound source separation.
 *
 * @input_name SRC_INFO
 * @input_type Vector<ObjectRef>
 * @input_description Source IDs and locations.
 *
 * @input_name SRC_WAVE
 * @input_type Map<int,ObjectRef>
 * @input_description Audio stream by each ID. The key is a source ID, and the value is a signal vector (Vector<float>).
 *
 * @input_name SRC_FFT
 * @input_type Map<int,ObjectRef>
 * @input_description Fourier transformed signal after sound source separation. 
 *
 * @input_name SRC_FEATURE
 * @input_type Map<int,ObjectRef>
 * @input_description Feature vectors. The key is a source ID, and the value is a feature vector (Vector<float>).
 *
 * @input_name SRC_RELIABILITY
 * @input_type Map<int,ObjectRef>
 * @input_description Reliability vectors. The key is a source ID, and the value is a reliability vector (Vector<float>).
 *
 * @input_name TIMESTAMP
 * @input_type TimeStamp
 * @input_description TimeStamp for published messages. This is optional input. If not connected, ros::Time::now() is stamped in the messages.
 *
 * @output_name OUTPUT
 * @output_type ObjectRef
 * @output_description This is a dummy output, and it has no mean. Only for an activation of this module.
 *
 *
 END*/

class RosHarkMsgsPublisher : public BufferedNode {
  bool enable_debug;     // flag whether print debug message or not.
  int advance;          // shift length of STFT.
  
  int mic_waveID;
  int mic_fftID;
  int src_infoID;
  int src_waveID;
  int src_fftID;
  int src_featureID;
  int src_reliabilityID;
  int TimeStampID;
  int outputID;
  
  // ROS related parameters
  string topic_name_harkwave;
  string topic_name_harkfft;
  string topic_name_harkfeature;
  string topic_name_harksource;
  string topic_name_harksrcwave;
  string topic_name_harksrcfft;
  string topic_name_harksrcfeature;
  string topic_name_harksrcfeaturemfm;
  int buffer_num;
  float ros_loop_rate;
  string ros_frame_id;
  string timestamp_type;
  int sampling_rate;
  float const_in_sec;
  ros::Time oHarkTimeBase;

private:
  ros::Publisher _pub_harkwave;
  ros::Publisher _pub_harkfft;
  ros::Publisher _pub_harkfeature;
  ros::Publisher _pub_harksource;
  ros::Publisher _pub_harksrcwave;
  ros::Publisher _pub_harksrcfft;
  ros::Publisher _pub_harksrcfeature;
  ros::Publisher _pub_harksrcfeaturemfm;

public:
  RosHarkMsgsPublisher(string nodeName, ParameterSet params)
    : BufferedNode(nodeName, params),
      mic_waveID(-1), mic_fftID(-1),
      src_infoID(-1), src_waveID(-1), src_fftID(-1), 
      src_featureID(-1), src_reliabilityID(-1), TimeStampID(-1)
  {
    
    advance = dereference_cast<int>(parameters.get("ADVANCE"));
    enable_debug = dereference_cast<bool>(parameters.get("ENABLE_DEBUG"));
    topic_name_harkwave           = object_cast<String>(parameters.get("TOPIC_NAME_HARKWAVE"));
    topic_name_harkfft            = object_cast<String>(parameters.get("TOPIC_NAME_HARKFFT"));
    topic_name_harkfeature        = object_cast<String>(parameters.get("TOPIC_NAME_HARKFEATURE"));
    topic_name_harksource         = object_cast<String>(parameters.get("TOPIC_NAME_HARKSOURCE"));
    topic_name_harksrcwave        = object_cast<String>(parameters.get("TOPIC_NAME_HARKSRCWAVE"));
    topic_name_harksrcfft         = object_cast<String>(parameters.get("TOPIC_NAME_HARKSRCFFT"));
    topic_name_harksrcfeature     = object_cast<String>(parameters.get("TOPIC_NAME_HARKSRCFEATURE"));
    topic_name_harksrcfeaturemfm  = object_cast<String>(parameters.get("TOPIC_NAME_HARKSRCFEATUREMFM"));
    buffer_num  = dereference_cast<int>(parameters.get("BUFFER_NUM"));
    ros_loop_rate  = dereference_cast<float>(parameters.get("ROS_LOOP_RATE"));
    ros_frame_id = object_cast<String>(parameters.get("ROS_FRAME_ID"));
    timestamp_type = object_cast<String>(parameters.get("TIMESTAMP_TYPE"));
    sampling_rate = dereference_cast<int>(parameters.get("SAMPLING_RATE"));
    const_in_sec = (float)advance / (float)sampling_rate;

    outputID = addOutput("OUTPUT");
    
    inOrder = true;
    cout << getName() << " constructor end..." << endl;
        
  }

  virtual void initialize()
  {
    cout << getName() << " initialized..." << endl;
    if (mic_waveID != -1){
      _pub_harkwave          = __MasterRosNodeHandler__->advertise<hark_msgs::HarkWave>(topic_name_harkwave, buffer_num, false);
    }
    if (mic_fftID != -1){
      _pub_harkfft           = __MasterRosNodeHandler__->advertise<hark_msgs::HarkFFT>(topic_name_harkfft, buffer_num, false);
    }
    if ((src_featureID != -1)&&(src_infoID == -1)){
      _pub_harkfeature       = __MasterRosNodeHandler__->advertise<hark_msgs::HarkFeature>(topic_name_harkfeature, buffer_num, false);
    }    
    if (src_infoID != -1){
      _pub_harksource        = __MasterRosNodeHandler__->advertise<hark_msgs::HarkSource>(topic_name_harksource, buffer_num, false);
    }
    if ((src_infoID != -1)&&(src_waveID != -1)){    
      _pub_harksrcwave       = __MasterRosNodeHandler__->advertise<hark_msgs::HarkSrcWave>(topic_name_harksrcwave, buffer_num, false);
    }
    if ((src_infoID != -1)&&(src_fftID != -1)){    
      _pub_harksrcfft        = __MasterRosNodeHandler__->advertise<hark_msgs::HarkSrcFFT>(topic_name_harksrcfft, buffer_num, false);
    }
    if ((src_infoID != -1)&&(src_featureID != -1)){    
      _pub_harksrcfeature    = __MasterRosNodeHandler__->advertise<hark_msgs::HarkSrcFeature>(topic_name_harksrcfeature, buffer_num, false);
    }
    if ((src_infoID != -1)&&(src_reliabilityID != -1)){    
      _pub_harksrcfeaturemfm = __MasterRosNodeHandler__->advertise<hark_msgs::HarkSrcFeatureMFM>(topic_name_harksrcfeaturemfm, buffer_num, false);
    }
    this->BufferedNode::initialize();
  }


  // dynamic input-port translation
  virtual int translateInput(string inputName) {
    //if (inputName == "STRING") {
    //    return strID = addInput(inputName);
    //}
    //else
    // check whether the input ports have arc from other module.
    if (inputName == "MIC_WAVE") {
      return mic_waveID = addInput(inputName);
    }
    else if (inputName == "MIC_FFT") {
      return mic_fftID = addInput(inputName);
    }
    else if (inputName == "SRC_INFO") {
      return src_infoID = addInput(inputName);
    }
    else if (inputName == "SRC_WAVE") {
      return src_waveID = addInput(inputName);
    }
    else if (inputName == "SRC_FFT") {
      return src_fftID = addInput(inputName);
    }
    else if (inputName == "SRC_FEATURE") {
      return src_featureID = addInput(inputName);
    }
    else if (inputName == "SRC_RELIABILITY") {
      return src_reliabilityID = addInput(inputName);
    }
    else if (inputName == "TIMESTAMP") {
      return TimeStampID = addInput(inputName);
    }
    else {
      throw new NodeException(this, inputName
			      + " is not supported.", __FILE__, __LINE__);
    }
  }

  // process per one iteration
  void calculate(int output_id, int count, Buffer &out) {

    int bytes;    
    RCPtr<Matrix<float> > mic_wave_ptr;
    RCPtr<Matrix<complex<float> > > mic_fft_ptr;
    RCPtr<Vector<ObjectRef> > src_info_ptr;
    RCPtr<Map<int, ObjectRef> > src_wave_ptr;
    RCPtr<Map<int, ObjectRef> > src_fft_ptr;
    RCPtr<Map<int, ObjectRef> > src_feature_ptr;
    RCPtr<Map<int, ObjectRef> > src_reliability_ptr;

    // bind objects of input-port to local variable
    if (mic_waveID != -1){
      mic_wave_ptr = getInput(mic_waveID, count);
      out[count] = mic_wave_ptr;
    }
    if (mic_fftID != -1){
      mic_fft_ptr = getInput(mic_fftID, count);
      out[count] = mic_fft_ptr;
    }
    if (src_infoID != -1){
      src_info_ptr = getInput(src_infoID, count);
      out[count] = src_info_ptr;
    }
    if (src_waveID != -1) {
      src_wave_ptr = getInput(src_waveID, count);
      out[count] = src_wave_ptr;
    }
    if (src_fftID != -1) {
      src_fft_ptr = getInput(src_fftID, count);
      out[count] = src_fft_ptr;
    }
    if (src_featureID != -1){
      src_feature_ptr = getInput(src_featureID, count);
      out[count] = src_feature_ptr; 
    }
    if (src_reliabilityID != -1){
      src_reliability_ptr = getInput(src_reliabilityID, count);
      out[count] = src_reliability_ptr;
    }
    
    hark_msgs::HarkSource HarkSourceMsg;
    hark_msgs::HarkFeature HarkFeatureMsg;
    hark_msgs::HarkSrcFeature HarkSrcFeatureMsg;
    hark_msgs::HarkSrcFeatureMFM HarkSrcFeatureMFMMsg;
    hark_msgs::HarkSrcWave HarkSrcWaveMsg;
    hark_msgs::HarkSrcFFT HarkSrcFFTMsg;
    hark_msgs::HarkWave HarkWaveMsg;
    hark_msgs::HarkFFT HarkFFTMsg;

    HarkSourceMsg.count = count;
    HarkFeatureMsg.count = count;
    HarkSrcFeatureMsg.count = count;
    HarkSrcFeatureMFMMsg.count = count;
    HarkSrcWaveMsg.count = count;
    HarkSrcFFTMsg.count = count;
    HarkWaveMsg.count = count;
    HarkFFTMsg.count = count;

    if(TimeStampID == -1){
      if (timestamp_type == "ROS_TIME_NOW") {      
	ros::Time oHarkTime = ros::Time::now();
	HarkSourceMsg.header.stamp = oHarkTime;
	HarkFeatureMsg.header.stamp = oHarkTime;
	HarkSrcFeatureMsg.header.stamp = oHarkTime;
	HarkSrcFeatureMFMMsg.header.stamp = oHarkTime;
	HarkSrcWaveMsg.header.stamp = oHarkTime;
	HarkSrcFFTMsg.header.stamp = oHarkTime;
	HarkWaveMsg.header.stamp = oHarkTime;
	HarkFFTMsg.header.stamp = oHarkTime;
      } else if (timestamp_type == "CONSTANT_INCREMENT"){
	if(count == 0) oHarkTimeBase = ros::Time::now();
	ros::Time oHarkTime = oHarkTimeBase + ros::Duration(const_in_sec * (float)count);
	HarkSourceMsg.header.stamp = oHarkTime;
	HarkFeatureMsg.header.stamp = oHarkTime;
	HarkSrcFeatureMsg.header.stamp = oHarkTime;
	HarkSrcFeatureMFMMsg.header.stamp = oHarkTime;
	HarkSrcWaveMsg.header.stamp = oHarkTime;
	HarkSrcFFTMsg.header.stamp = oHarkTime;
	HarkWaveMsg.header.stamp = oHarkTime;
	HarkFFTMsg.header.stamp = oHarkTime;	
      }      
    }else{
      ObjectRef oBaseTimeStamp = getInput(TimeStampID,count);
      const TimeStamp& BaseTime = object_cast<TimeStamp> (oBaseTimeStamp);
      HarkSourceMsg.header.stamp.sec        = BaseTime.time.tv_sec;
      HarkFeatureMsg.header.stamp.sec       = BaseTime.time.tv_sec;
      HarkSrcFeatureMsg.header.stamp.sec    = BaseTime.time.tv_sec;
      HarkSrcFeatureMFMMsg.header.stamp.sec = BaseTime.time.tv_sec;
      HarkSrcWaveMsg.header.stamp.sec       = BaseTime.time.tv_sec;
      HarkSrcFFTMsg.header.stamp.sec        = BaseTime.time.tv_sec;
      HarkWaveMsg.header.stamp.sec          = BaseTime.time.tv_sec;
      HarkFFTMsg.header.stamp.sec           = BaseTime.time.tv_sec;
      HarkSourceMsg.header.stamp.nsec        = BaseTime.time.tv_usec * 1000;
      HarkFeatureMsg.header.stamp.nsec       = BaseTime.time.tv_usec * 1000;
      HarkSrcFeatureMsg.header.stamp.nsec    = BaseTime.time.tv_usec * 1000;
      HarkSrcFeatureMFMMsg.header.stamp.nsec = BaseTime.time.tv_usec * 1000;
      HarkSrcWaveMsg.header.stamp.nsec       = BaseTime.time.tv_usec * 1000;
      HarkSrcFFTMsg.header.stamp.nsec        = BaseTime.time.tv_usec * 1000;
      HarkWaveMsg.header.stamp.nsec          = BaseTime.time.tv_usec * 1000;
      HarkFFTMsg.header.stamp.nsec           = BaseTime.time.tv_usec * 1000;
    }

    HarkSourceMsg.header.frame_id = ros_frame_id;
    HarkFeatureMsg.header.frame_id = ros_frame_id;
    HarkSrcFeatureMsg.header.frame_id = ros_frame_id;
    HarkSrcFeatureMFMMsg.header.frame_id = ros_frame_id;
    HarkSrcWaveMsg.header.frame_id = ros_frame_id;
    HarkSrcFFTMsg.header.frame_id = ros_frame_id;
    HarkWaveMsg.header.frame_id = ros_frame_id;
    HarkFFTMsg.header.frame_id = ros_frame_id;

    ros::Rate loop_rate(ros_loop_rate);

    ///////////////////////////////////////////////////////////////
    //
    //      bodies
    //
    ///////////////////////////////////////////////////////////////

    //----- for microphone array input signals
    //////////////////////////////////////////////////////////////
    if (!mic_wave_ptr.isNil()) {

      int nch = mic_wave_ptr->nrows();
      int length = mic_wave_ptr->ncols();

      if (enable_debug == true) printf("MIC_WAVE: %d %d\n", nch, length);
      bytes = nch * length * sizeof(float);

      HarkWaveMsg.nch = nch;
      HarkWaveMsg.length = length;
      HarkWaveMsg.data_bytes = bytes;
      HarkWaveMsg.src.resize(nch);
      for (int c = 0; c < nch; c++) {
      HarkWaveMsg.src[c].wavedata.resize(length);
	for (int t = 0; t < length; t++) {
	  HarkWaveMsg.src[c].wavedata[t] = (float)(*mic_wave_ptr)(c, t);
	}
      }
      _pub_harkwave.publish(HarkWaveMsg);
	
    }

    //----- for microphone array FFT signals
    //////////////////////////////////////////////////////////////
    if (!mic_fft_ptr.isNil()) {

      int nch = mic_fft_ptr->nrows();
      int length = mic_fft_ptr->ncols();

      if (enable_debug == true) printf("MIC_FFT: %d %d\n", nch, length);
      bytes = nch * length * sizeof(float);

      HarkFFTMsg.nch = nch;
      HarkFFTMsg.length = length;
      HarkFFTMsg.src.resize(nch);
      HarkFFTMsg.src.resize(nch);
      for (int c = 0; c < nch; c++) {
	HarkFFTMsg.src[c].fftdata_real.resize(length);
	HarkFFTMsg.src[c].fftdata_imag.resize(length);
	for (int t = 0; t < length; t++) {
	  HarkFFTMsg.src[c].fftdata_real[t] = (float)(*mic_fft_ptr)(c, t).real();
	  HarkFFTMsg.src[c].fftdata_imag[t] = (float)(*mic_fft_ptr)(c, t).imag();
	}
      }
      _pub_harkfft.publish(HarkFFTMsg);
	
    }

    //----- for feature vectors without sourceID
    //////////////////////////////////////////////////////////////
    if ((!src_feature_ptr.isNil())&&(src_info_ptr.isNil())) {

      int exist_src_num = 0;
      Map<int, ObjectRef>::const_iterator it0;
      for (it0 = (*src_feature_ptr).begin(); it0 != (*src_feature_ptr).end(); ++it0) {
	exist_src_num++;
      }
      HarkFeatureMsg.exist_src_num = exist_src_num;
      Map<int, ObjectRef>::const_iterator it;
      for (it = (*src_feature_ptr).begin(); it != (*src_feature_ptr).end(); ++it) {
	hark_msgs::HarkFeatureVal HarkFeatureValMsg;
        const Vector<float>& feature = object_cast<Vector<float> >(it->second);
      	int length = feature.size();
	HarkFeatureValMsg.id         = it->first;
	HarkFeatureValMsg.length     = length;
	HarkFeatureValMsg.data_bytes = length * sizeof(float);
	HarkFeatureValMsg.featuredata.resize(length);
	for (int cnt = 0; cnt < length; cnt++) {
	  HarkFeatureValMsg.featuredata[cnt] = feature[cnt];
	}
	HarkFeatureMsg.src.push_back(HarkFeatureValMsg);
      }
      _pub_harkfeature.publish(HarkFeatureMsg);	
    }

    //
    // ----- for separated source signals
    /////////////////////////////////////////////////////////////
    if (!src_info_ptr.isNil()) {
      int exist_src_num;
      const Vector<ObjectRef> &srcInfos = *src_info_ptr;

      // send number of source
      exist_src_num = srcInfos.size();

      HarkSourceMsg.exist_src_num        = exist_src_num;
      HarkSrcFeatureMsg.exist_src_num    = exist_src_num;
      HarkSrcFeatureMFMMsg.exist_src_num = exist_src_num;
      HarkSrcWaveMsg.exist_src_num       = exist_src_num;
      HarkSrcFFTMsg.exist_src_num       = exist_src_num;
	
      if (enable_debug == true) printf("SRC_NUM: %d\n", exist_src_num);

      // for each source
      for (int i = 0; i < srcInfos.size(); i++) {

	RCPtr<Source> src = srcInfos[i];

	hark_msgs::HarkSourceVal HarkSourceValMsg;
	hark_msgs::HarkSrcFeatureVal HarkSrcFeatureValMsg;
	hark_msgs::HarkSrcFeatureMFMVal HarkSrcFeatureMFMValMsg;
	hark_msgs::HarkSrcWaveVal HarkSrcWaveValMsg;
	hark_msgs::HarkSrcFFTVal HarkSrcFFTValMsg;

	//
	//   header
	/////////////////////////////////////////////////////
		
	HarkSourceValMsg.id    = src->id;
	HarkSourceValMsg.power = src->power;
	HarkSourceValMsg.x     = src->x[0];
	HarkSourceValMsg.y     = src->x[1];
	HarkSourceValMsg.z     = src->x[2];
	HarkSourceValMsg.theta = 180.0 / M_PI * atan2(src->x[1], src->x[0]);
	HarkSourceMsg.src.push_back(HarkSourceValMsg);

	HarkSrcFeatureValMsg.id    = src->id;
	HarkSrcFeatureValMsg.power = src->power;
	HarkSrcFeatureValMsg.x     = src->x[0];
	HarkSrcFeatureValMsg.y     = src->x[1];
	HarkSrcFeatureValMsg.z     = src->x[2];
	HarkSrcFeatureValMsg.theta = 180.0 / M_PI * atan2(src->x[1], src->x[0]);

	HarkSrcFeatureMFMValMsg.id    = src->id;
	HarkSrcFeatureMFMValMsg.power = src->power;
	HarkSrcFeatureMFMValMsg.x     = src->x[0];
	HarkSrcFeatureMFMValMsg.y     = src->x[1];
	HarkSrcFeatureMFMValMsg.z     = src->x[2];
	HarkSrcFeatureMFMValMsg.theta = 180.0 / M_PI * atan2(src->x[1], src->x[0]);

	HarkSrcWaveValMsg.id    = src->id;
	HarkSrcWaveValMsg.power = src->power;
	HarkSrcWaveValMsg.x     = src->x[0];
	HarkSrcWaveValMsg.y     = src->x[1];
	HarkSrcWaveValMsg.z     = src->x[2];
	HarkSrcWaveValMsg.theta = 180.0 / M_PI * atan2(src->x[1], src->x[0]);

	HarkSrcFFTValMsg.id    = src->id;
	HarkSrcFFTValMsg.power = src->power;
	HarkSrcFFTValMsg.x     = src->x[0];
	HarkSrcFFTValMsg.y     = src->x[1];
	HarkSrcFFTValMsg.z     = src->x[2];
	HarkSrcFFTValMsg.theta = 180.0 / M_PI * atan2(src->x[1], src->x[0]);

	if (enable_debug == true) printf("\tSRC_INFO[%d]: id %d, x %f %f %f, power %f\n",
					i, src->id, src->x[0], src->x[1], src->x[2], src->power);

	// send wave data
	/////////////////////////////////////////////////////
	if (!src_wave_ptr.isNil()) {

	  const Vector<float>& src_wave = object_cast<Vector<float> >((*src_wave_ptr).find(src->id)->second);
	  int length = src_wave.size();
	  //int bytes = length * sizeof(float);
	  int bytes = advance * sizeof(short int);
	
	  if (enable_debug == true) printf("\t\tSWAVE: %d %d; %d\n", advance, bytes, length);

	  //---- body -----------------------------------//
	  vector<short int> shrt_buf;
	  shrt_buf.resize(length);
	  for (int cnt = 0; cnt < length; cnt++) {
	    shrt_buf[cnt] = (short int)src_wave[cnt];
	  }

	  HarkSrcWaveValMsg.length    = advance; // length;
	  HarkSrcWaveValMsg.data_bytes= bytes;
	  HarkSrcWaveValMsg.wavedata.resize(advance);
	  for (int cnt = 0; cnt < advance; cnt++) {
	    HarkSrcWaveValMsg.wavedata[cnt] = (float)shrt_buf[length - advance + cnt];
	  }
	  HarkSrcWaveMsg.src.push_back(HarkSrcWaveValMsg);
	  
	}

	// send fft data
	/////////////////////////////////////////////////////
	if (!src_fft_ptr.isNil()) {

	  const Vector<complex<float> >& src_fft = object_cast<Vector<complex<float> > >((*src_fft_ptr).find(src->id)->second);
	  int length = src_fft.size();
	
	  if (enable_debug == true) printf("\t\tSFFT: %d\n", length);

	  HarkSrcFFTValMsg.length    = length;
	  HarkSrcFFTValMsg.fftdata_real.resize(length);
	  HarkSrcFFTValMsg.fftdata_imag.resize(length);
	  for (int cnt = 0; cnt < length; cnt++) {
	    HarkSrcFFTValMsg.fftdata_real[cnt] = (float)src_fft[cnt].real();
	    HarkSrcFFTValMsg.fftdata_imag[cnt] = (float)src_fft[cnt].imag();
	  }
	  HarkSrcFFTMsg.src.push_back(HarkSrcFFTValMsg);
	  
	}

	// send feature data
	/////////////////////////////////////////////////////
	if (!src_feature_ptr.isNil()) {

	  const Vector<float>& src_feature = object_cast<Vector<float> >((*src_feature_ptr).find(src->id)->second);
	  int length = src_feature.size();
	  int bytes = length * sizeof(float);
	
	  HarkSrcFeatureValMsg.length    = length;
	  HarkSrcFeatureValMsg.data_bytes   = bytes;
	  HarkSrcFeatureValMsg.featuredata.resize(length);
	  for (int cnt = 0; cnt < length; cnt++) {
	    HarkSrcFeatureValMsg.featuredata[cnt] = src_feature[cnt];
	  }
	  HarkSrcFeatureMsg.src.push_back(HarkSrcFeatureValMsg);	  

	  HarkSrcFeatureMFMValMsg.length_feature    = length;
	  HarkSrcFeatureMFMValMsg.data_bytes_feature= bytes;
	  HarkSrcFeatureMFMValMsg.featuredata_feature.resize(length);
	  for (int cnt = 0; cnt < length; cnt++) {
	    HarkSrcFeatureMFMValMsg.featuredata_feature[cnt] = src_feature[cnt];
	  }
	  
	}

	// send reliability data
	/////////////////////////////////////////////////////
	if (!src_reliability_ptr.isNil()) {

	  const Vector<float>& src_rel = object_cast<Vector<float> >((*src_reliability_ptr).find(src->id)->second);
	  int length = src_rel.size();
	  int bytes = length * sizeof(float);
	
	  HarkSrcFeatureMFMValMsg.length_mfm    = length;
	  HarkSrcFeatureMFMValMsg.data_bytes_mfm= bytes;
	  HarkSrcFeatureMFMValMsg.featuredata_mfm.resize(length);
	  for (int cnt = 0; cnt < length; cnt++) {
	    HarkSrcFeatureMFMValMsg.featuredata_mfm[cnt] = src_rel[cnt];
	  }
	  HarkSrcFeatureMFMMsg.src.push_back(HarkSrcFeatureMFMValMsg);
	  
	}
		
      }

      if (!src_info_ptr.isNil()) {
	_pub_harksource.publish(HarkSourceMsg);
	if (!src_wave_ptr.isNil()) {
	  _pub_harksrcwave.publish(HarkSrcWaveMsg);
	}
	if (!src_fft_ptr.isNil()) {	
	  _pub_harksrcfft.publish(HarkSrcFFTMsg);
	}
	if (!src_feature_ptr.isNil()) {	
	  _pub_harksrcfeature.publish(HarkSrcFeatureMsg);
	}
	if (!src_reliability_ptr.isNil()) {	
	  _pub_harksrcfeaturemfm.publish(HarkSrcFeatureMFMMsg);
	}
      }
      
    }

    loop_rate.sleep();

  }
    
  IN_ORDER_NODE_SPEEDUP(RosHarkMsgsPublisher)
    
};

#endif
