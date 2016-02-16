/*
 * Copyright 2008 Kyoto University and Honda Motor Co.,Ltd.
 * All rights reserved.
 * HARK was developed by researchers in Okuno Laboratory
 * at the Kyoto University and Honda Research Institute Japan Co.,Ltd.
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
#include <sys/socket.h>
#include <unistd.h>
#include <netdb.h>
#include <strings.h>

#include <boost/shared_ptr.hpp>

#include "HarkData.h"

using namespace std;
using namespace FD;

class HarkDataStreamSender;

DECLARE_NODE(HarkDataStreamSender)
/*Node
 *
 * @name HarkDataStreamSender
 * @category HARK:AudioIO
 * @description Hark data sender through socket.
 *
 * @parameter_name HOST
 * @parameter_type string
 * @parameter_value localhost
 * @parameter_description Hostname or IP of a receiver
 *
 * @parameter_name PORT
 * @parameter_type int
 * @parameter_value 5530
 * @parameter_description Port number of a receiver
 *
 * @parameter_name ADVANCE
 * @parameter_type int
 * @parameter_value 160
 * @parameter_description Shift sample number for sliding spectrum analysis.
 *
 * @parameter_name BUFFER_SIZE
 * @parameter_type int
 * @parameter_value 512
 * @parameter_description Buffer size
 *
 * @parameter_name FRAMES_PER_SEND
 * @parameter_type int
 * @parameter_value 1
 * @parameter_description Frequency of sending data in frame unit. If 1, the data of every frame will be sent. If 10, the data will be sent when the frame count is a multiple of 10.
 *
 * @parameter_name DEBUG_PRINT
 * @parameter_type bool
 * @parameter_value false
 * @parameter_list true:false
 * @parameter_description print debug message of this module in case of true.
 *
 * @parameter_name SOCKET_ENABLE
 * @parameter_type bool
 * @parameter_list true:false
 * @parameter_description send data via socket in case of "true".
 * @parameter_value true
 *
 * @input_name MIC_WAVE
 * @input_type Matrix<float>
 * @input_description Microphone input signals. 
 *
 * @input_name MIC_FFT
 * @input_type Matrix<complex<float> >
 * @input_description Spectrum after STFT.
 *
 * @input_name SRC_INFO
 * @input_type Vector<ObjectRef>
 * @input_description Source IDs and locations.
 *
 * @input_name SRC_WAVE
 * @input_type Map<int,ObjectRef>
 * @input_description Audio Stream of each source. The key is a source ID, and the value is a signal vector (Vector<float>).
 *
 * @input_name SRC_FFT
 * @input_type Map<int,ObjectRef>
 * @input_description Spectrum after STFT. The key is a source ID, and the value is a spectrum vector (Vector<complex<float> >).
 *
 * @input_name SRC_FEATURE
 * @input_type Map<int,ObjectRef>
 * @input_description Feature vectors. The key is a source ID, and the value is a feature vector (Vector<float>).
 *
 * @input_name SRC_RELIABILITY
 * @input_type Map<int,ObjectRef>
 * @input_description Reliability vectors. The key is a source ID, and the value is a reliability vector (Vector<float>).
 *
 * @output_name OUTPUT
 * @output_type ObjectRef
 * @output_description This is a dummy output, and it has no mean. Only for an activation of this module.
 *
 *
END*/

//#define _DEBUG

class HarkDataStreamSender : public BufferedNode {
    bool debug_print;     // flag whether print debug message or not.
    string hostname;      // hostname of a receiver
    unsigned short port;  // service port number 

    int advance;          // shift length of STFT.
    int frames_per_send;

    int mic_waveID;
    int mic_fftID;
    int src_infoID;
    int src_waveID;
    int src_fftID;
    int src_featureID;
    int src_reliabilityID;
    int outputID;
    bool socket_enable;

    int socket_id;       // socket

    int max_bufsize;      // memory size of the maximum workspace.
    float *flt_buf;       // 
    short int *shrt_buf;  //  

public:
    HarkDataStreamSender(string nodeName, ParameterSet params)
	: BufferedNode(nodeName, params),
	  // get IDs of input port
	  mic_waveID(-1), mic_fftID(-1),
	  src_infoID(-1), src_waveID(-1), src_fftID(-1), src_featureID(-1),
	  src_reliabilityID(-1) {
	
	// bind parameters to local memory
	hostname = object_cast<String>(parameters.get("HOST"));
	port = dereference_cast<int>(parameters.get("PORT"));
	advance = dereference_cast<int>(parameters.get("ADVANCE"));
	frames_per_send = dereference_cast<int>(parameters.get("FRAMES_PER_SEND"));
	max_bufsize = dereference_cast<int>(parameters.get("BUFFER_SIZE"));
	max_bufsize *= 1024;
	debug_print = dereference_cast<bool>(parameters.get("DEBUG_PRINT"));
	socket_enable = dereference_cast<bool>(parameters.get("SOCKET_ENABLE"));

	// get ID of output port
	outputID = addOutput("OUTPUT");

	if(socket_enable){
	  // socket connection
	  socket_id = connectToServer();
	  if (socket_id == -1) {
	    cout << "connection error !" << endl;
	    exit(0);
	  }
	  else {
	    cout << "connected.\n" << endl;
	  }
	}
	
	// buffer memory allocation
	flt_buf = (float *)calloc(sizeof(float), max_bufsize);
	shrt_buf = (short int *)calloc(sizeof(short int), max_bufsize);
    }


    // dynamic input-port translation
    virtual int translateInput(string inputName) {
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
	else {
	  throw new NodeException(this, inputName
				  + " is not supported.", __FILE__, __LINE__);
	}
    }

    // process per one iteration
    void calculate(int output_id, int count, Buffer &out) {
	int bytes;
	static int allsize = 0;
	HD_Header hdHeader;
	    
	RCPtr<Matrix<float> > mic_waves_ptr;
	RCPtr<Matrix<complex<float> > > mic_fft_ptr;
	RCPtr<Vector<ObjectRef> > src_info_ptr;
	RCPtr<Map<int, ObjectRef> > src_wave_ptr;
	RCPtr<Map<int, ObjectRef> > src_fft_ptr;
	RCPtr<Map<int, ObjectRef> > src_feature_ptr;
	RCPtr<Map<int, ObjectRef> > src_reliability_ptr;

	// bind objects of input-port to local variable
	if (mic_waveID != -1) mic_waves_ptr = getInput(mic_waveID, count);
	if (mic_fftID != -1) mic_fft_ptr = getInput(mic_fftID, count);
	if (src_infoID != -1) src_info_ptr = getInput(src_infoID, count);
	if (src_waveID != -1) src_wave_ptr = getInput(src_waveID, count);
	if (src_fftID != -1) src_fft_ptr = getInput(src_fftID, count);
	if (src_featureID != -1) src_feature_ptr = getInput(src_featureID, count);
	if (src_reliabilityID != -1) src_reliability_ptr = getInput(src_reliabilityID, count);

	// define the output
	if (!mic_waves_ptr.isNil()) {
	    out[count] = mic_waves_ptr;
	}
	if (!mic_fft_ptr.isNil()) {
	    out[count] = mic_fft_ptr;
	}
	if (!src_info_ptr.isNil()) {
	    out[count] = src_info_ptr;
	}

	// skip the publish
	if (count % frames_per_send != 0) return;

	//----- check input-ports and make header
	hdHeader.type = 0x0;

	if (!mic_waves_ptr.isNil()) {
	    hdHeader.type |= PACKET_MIC_WAVE;
	}
	if (!mic_fft_ptr.isNil()) {
	    hdHeader.type |= PACKET_MIC_FFT;
	}

	// source id related data
	if (!src_info_ptr.isNil()) {
	    hdHeader.type |= PACKET_SRC_INFO;
	    
	    if (!src_wave_ptr.isNil()) {
		hdHeader.type |= PACKET_SRC_WAVE;
	    }
	    if (!src_fft_ptr.isNil()) {
		hdHeader.type |= PACKET_SRC_FFT;
	    }
	    if (!src_feature_ptr.isNil()) {
		hdHeader.type |= PACKET_SRC_FEATURE;
	    }
	    if (!src_reliability_ptr.isNil()) {
		hdHeader.type |= PACKET_SRC_RELIABILITY;
	    }
	}

	///////////////////////////////////////////////////////////////
	//
	//     header
	//
	///////////////////////////////////////////////////////////////
	bytes = sizeof(HD_Header);
	hdHeader.count = count;
	hdHeader.advance = advance;
	
	if(socket_enable){
	  write(socket_id, &hdHeader, sizeof(HD_Header));
	}
	
	if (debug_print == true) printf("send header %d: %d.\n", hdHeader.type, allsize++);

	///////////////////////////////////////////////////////////////
	//
	//      bodies
	//
	///////////////////////////////////////////////////////////////

	//
	//----- for microphone array input signals
	//////////////////////////////////////////////////////////////
	if (!mic_waves_ptr.isNil()) {
	    HDH_MicData hdhMicWave;
	    int nch = mic_waves_ptr->nrows();
	    int length = mic_waves_ptr->ncols();

	    if (debug_print == true) printf("MIC_WAVE: %d %d\n", nch, length);
	    bytes = nch * length * sizeof(float);
	    
	    //----- header
	    hdhMicWave.nch = nch;
	    hdhMicWave.length = length;
	    hdhMicWave.data_bytes = bytes;

	    if(socket_enable){
	      // write header
	      write(socket_id, &hdhMicWave, sizeof(HDH_MicData));
	    }
	    
	    //----- body
	    for (int c = 0; c < nch; c++) {
	      for (int t = 0; t < length; t++) {
		flt_buf[t + c * length] = (*mic_waves_ptr)(c, t);
	      }
	    }

	    if(socket_enable){
	      // write data
	      write(socket_id, flt_buf, hdhMicWave.data_bytes);
	    }	
	}

	//
	//----- for spectrum after STFT
	//////////////////////////////////////////////////////////////
	if (!mic_fft_ptr.isNil()) {
	    HDH_MicData hdhMicFFT;
	    int nch = mic_fft_ptr->nrows();
	    int length = mic_fft_ptr->ncols();

	    if (debug_print == true) printf("MIC_FFT: %d %d\n", nch, length);
	    bytes = nch * length * sizeof(float);
	    
	    //----- header
	    hdhMicFFT.nch = nch;
	    hdhMicFFT.length = length;
	    hdhMicFFT.data_bytes = bytes;

	    if(socket_enable){
	      // write header
	      write(socket_id, &hdhMicFFT, sizeof(HDH_MicData));
	    }
	    
	    //----- body
	    // real
	    for (int c = 0; c < nch; c++) {
	      for (int t = 0; t < length; t++) {
		flt_buf[t + c * length] = (*mic_fft_ptr)(c, t).real();
	      }
	    }
	    if(socket_enable){
	      write(socket_id, flt_buf, hdhMicFFT.data_bytes);
	    }
	    // imaginal
	    for (int c = 0; c < nch; c++) {
	      for (int t = 0; t < length; t++) {
		flt_buf[t + c * length] = (*mic_fft_ptr)(c, t).imag();
	      }
	    }
	    if(socket_enable){
	      write(socket_id, flt_buf, hdhMicFFT.data_bytes);
	    }	
	}

	//
	// ----- for separated source signals
	/////////////////////////////////////////////////////////////
	if (!src_info_ptr.isNil()) {
	    int exist_src_num;
	    const Vector<ObjectRef> &srcInfos = *src_info_ptr;

	    // send number of source
	    exist_src_num = srcInfos.size();

	    if(socket_enable){
	      write(socket_id, &exist_src_num, sizeof(int));
	    }
	
	    if (debug_print == true) printf("SRC_NUM: %d\n", exist_src_num);

	    // for each source
	    for (int i = 0; i < srcInfos.size(); i++) {
		HDH_SrcInfo hdhSrcInfo;
		RCPtr<Source> src = srcInfos[i];

		//
		//   header
		/////////////////////////////////////////////////////
		
		// send source information
		//gettimeofday(&(hdhSrcInfo.time), NULL);
		hdhSrcInfo.src_id = src->id;
		hdhSrcInfo.x[0] = src->x[0];
		hdhSrcInfo.x[1] = src->x[1];
		hdhSrcInfo.x[2] = src->x[2];
		hdhSrcInfo.power = src->power;
		//hdhSrcInfo.azimuth = 180 / M_PI * atan2f(src->x[1], src->x[0]);
		//hdhSrcInfo.elevation = 180 / M_PI * asin(src->x[2]);

		if(socket_enable){
		  write(socket_id, &hdhSrcInfo, sizeof(HDH_SrcInfo));
		}
	
		if (debug_print == true) printf("\tSRC_INFO[%d]: id %d, x %f %f %f, power %f\n",
		       i, hdhSrcInfo.src_id,
		       hdhSrcInfo.x[0], hdhSrcInfo.x[1], hdhSrcInfo.x[2], hdhSrcInfo.power);

		//
		// send wave data
		/////////////////////////////////////////////////////
		if (!src_wave_ptr.isNil()) {
		    HDH_SrcData srcData;
		    const Vector<float>& src_wave = object_cast<Vector<float> >((*src_wave_ptr).find(src->id)->second);
		    int length = src_wave.size();
		    //int bytes = length * sizeof(float);
		    int bytes = advance * sizeof(short int);

		    //---- header ---------------------------------//
		    srcData.length = advance;//length;
		    srcData.data_bytes = bytes;

		    if(socket_enable){
		      write(socket_id, &srcData, sizeof(HDH_SrcData));
		    }
	
		    if (debug_print == true) printf("\t\tSWAVE: %d %d; %d\n",
						    srcData.length, srcData.data_bytes, length);

		    //---- body -----------------------------------//
		    for (int i = 0; i < length; i++) {
		      //flt_buf[i] = src_wave[i];
		      shrt_buf[i] = (short int)src_wave[i];
		    }

		    if(socket_enable){
		      write(socket_id, &shrt_buf[length-advance], bytes);
		      //write(socket_id, flt_buf, bytes);     
		    }
		}

		//
		// send fft data
		/////////////////////////////////////////////////////
		if (!src_fft_ptr.isNil()) {
		    HDH_SrcData srcData;
		    const Vector<complex<float> >& src_fft = object_cast<Vector<complex<float> > >((*src_fft_ptr).find(src->id)->second);
		    int length = src_fft.size();
		    int bytes = length * sizeof(float);

		    //---- header ---------------------------------//
		    srcData.length = length;
		    srcData.data_bytes = bytes;

		    if(socket_enable){
		      write(socket_id, &srcData, sizeof(HDH_SrcData));
		    }
	
		    if (debug_print == true) printf("\t\tSFFT: %d %d\n", srcData.length, srcData.data_bytes);

		    //---- body -----------------------------------//
		    // real
		    for (int i = 0; i < length; i++) {
		      flt_buf[i] = src_fft[i].real();
		    }
		    if(socket_enable){
		      write(socket_id, flt_buf, bytes);     
		    }
		    // imaginal
		    for (int i = 0; i < length; i++) {
		      flt_buf[i] = src_fft[i].imag();
		    }
		    if(socket_enable){
		      write(socket_id, flt_buf, bytes);     
		    }
		}

		//
		// send feature data
		/////////////////////////////////////////////////////
		if (!src_feature_ptr.isNil()) {
		    HDH_SrcData srcData;
		    const Vector<float>& src_feature = object_cast<Vector<float> >((*src_feature_ptr).find(src->id)->second);
		    int length = src_feature.size();
		    int bytes = length * sizeof(float);

		    //---- header ----------------------------------//
		    srcData.length = length;
		    srcData.data_bytes = bytes;
		    if(socket_enable){
		      write(socket_id, &srcData, sizeof(HDH_SrcData));
		    }

		    if (debug_print == true) printf("\t\tS_FEAT: %d %d\n", srcData.length, srcData.data_bytes);
	
		    //---- body ------------------------------------//
		    for (int i = 0; i < length; i++) {
			flt_buf[i] = src_feature[i];

		    }
		    if(socket_enable){
		      write(socket_id, flt_buf, bytes);
		    }
		}

		//
		// send reliability data
		/////////////////////////////////////////////////////
		if (!src_reliability_ptr.isNil()) {
		    HDH_SrcData srcData;
		    const Vector<float>& src_rel = object_cast<Vector<float> >((*src_reliability_ptr).find(src->id)->second);
		    
		    int length = src_rel.size();
		    int bytes = length * sizeof(float);

		    //---- header ----------------------------------//
		    srcData.length = length;
		    srcData.data_bytes = bytes;

		    if(socket_enable){
		      write(socket_id, &srcData, sizeof(HDH_SrcData));
		    }

		    if (debug_print == true) printf("\t\tS_REL: %d %d\n", srcData.length, srcData.data_bytes);
	
		    //---- body ------------------------------------//
		    for (int i = 0; i < length; i++) {
			flt_buf[i] = src_rel[i];
		    }
		    if(socket_enable){
		      write(socket_id, flt_buf, bytes);
		    }
		}
		
	    }
	}
    }

    ///////////////////////////////////////////////////////////////
    //
    // connect to a socket server
    //
    ///////////////////////////////////////////////////////////////
    int connectToServer() {
	int socket_id;
	
	hostent* connect_host;
	if ((connect_host = gethostbyname(hostname.c_str())) == NULL) {
	    cerr << "cannot find the host." << endl;
	    return -3;
	}
	
	sockaddr_in desthost;
	bzero((char*)&desthost, sizeof(desthost));
	desthost.sin_family = AF_INET;
	desthost.sin_port = htons(port);
	bcopy(connect_host->h_addr, (char*)&desthost.sin_addr, connect_host->h_length);
	
	if ((socket_id = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
	    cerr << "cannot get socket." << endl;
	    return -2;
	}
	
	if (connect(socket_id, (sockaddr*)&desthost, sizeof(desthost)) == -1) {
	    cerr << "connection refused." << endl;
	    close(socket_id);
	    return -1;
	}
	
	return socket_id;
    }
    
    IN_ORDER_NODE_SPEEDUP(HarkDataStreamSender)
    
};
