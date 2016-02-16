/*
 * Copyright 2008 Kyoto University and Honda Motor Co.,Ltd.
 * All rights reserved.
 * HARK was developed by researchers in Okuno Laboratory
 * at the Kyoto University and Honda Research Institute Japan Co.,Ltd.
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
#include "spline.hpp"
#include <deque> 

#include <boost/scoped_ptr.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

#include "../config.h"

#ifdef ENABLE_HARKIO

#include "LocalizationMUSIC.hpp"
#include "HarkParamsObjects.h"

using namespace std;
using namespace boost;
using namespace FD;
using namespace boost::numeric;
using namespace boost::posix_time;

class LocalizeMUSIC;

DECLARE_NODE(LocalizeMUSIC);
/*Node
 *
 * @name LocalizeMUSIC
 * @category HARK:Localization
 * @description Sound Source Localization by MUSIC
 *
 * @input_name INPUT
 * @input_type Matrix<complex<float> >
 * @input_description Multi-channel audio signals. In this matrix, a row is a channel, and a column is a sample.
 *
 * @input_name NOISECM
 * @input_type Matrix<complex<float> >
 * @input_description  Optional correlation matrix. This is a 2D matrix. The row denotes the frequency series.  The col denotes the correlation matrix of each frequency. The col should be transformed to a matrix in the receiver. If this is not connected, an identity matrix would be used.
 *
 * @output_name OUTPUT
 * @output_type Vector<ObjectRef>
 * @output_description Source location. A type of an element of the vector is Source.
 *
 * @parameter_name MUSIC_ALGORITHM
 * @parameter_type string
 * @parameter_value SEVD
 * @parameter_list SEVD:GEVD:GSVD
 * @parameter_description Sound Source Localization Algorithm. If SEVD, NOISECM will be ignored
 * 
 * @parameter_name TF_CHANNEL_SELECTION
 * @parameter_type object
 * @parameter_value <Vector<int> 0 1 2 3 4 5 6 7>
 * @parameter_description Microphone channels for localization
 * 
 * @parameter_name LENGTH
 * @parameter_type int
 * @parameter_value 512
 * @parameter_description The length of a frame (per channel).
 * 
 * @parameter_name SAMPLING_RATE
 * @parameter_type int
 * @parameter_value 16000
 * @parameter_description Sampling Rate (Hz).
 * 
 * @parameter_name A_MATRIX
 * @parameter_type string
 * @parameter_description Filename of a transfer function matrix.
 * 
 * @parameter_name WINDOW
 * @parameter_type int
 * @parameter_value 50
 * @parameter_description The number of frames used for calculating a correlation function.
 * 
 * @parameter_name WINDOW_TYPE
 * @parameter_type string
 * @parameter_list PAST:MIDDLE:FUTURE
 * @parameter_value FUTURE
 * @parameter_description Window selection to accumulate a correlation function. If PAST, the past WINDOW frames from the current frame are used for the accumulation. If MIDDLE, the current frame will be the middle of the accumulated frames. If FUTURE, the future WINDOW frames from the current frame are used for the accumulation. FUTURE is the default from version 1.0, but this makes a delay since we have to wait for the future information. PAST generates a internal buffers for the accumulation, which realizes no delay for localization. 
 * 
 * @parameter_name PERIOD
 * @parameter_type int
 * @parameter_value 50
 * @parameter_description The period in which the source localization is processed.
 * 
 * @parameter_name NUM_SOURCE
 * @parameter_type int
 * @parameter_value 2
 * @parameter_description Number of sources, which should be less than number of channels.
 * 
 * @parameter_name MIN_DEG
 * @parameter_type int
 * @parameter_description source direction (lower).
 * @parameter_value -180
 * 
 * @parameter_name MAX_DEG
 * @parameter_type int
 * @parameter_description source direction (higher).
 * @parameter_value 180
 * 
 * @parameter_name LOWER_BOUND_FREQUENCY
 * @parameter_type int
 * @parameter_value 500
 * @parameter_description Lower bound of frequency (Hz) used for correlation function calculation.
 * 
 * @parameter_name UPPER_BOUND_FREQUENCY
 * @parameter_type int
 * @parameter_value 2800
 * @parameter_description Upper bound of frequency (Hz) used for correlation function calculation.
 * 
 * @parameter_name SPECTRUM_WEIGHT_TYPE
 * @parameter_type string
 * @parameter_list Uniform:A_Characteristic:Manual_Spline:Manual_Square
 * @parameter_value Uniform
 * @parameter_description MUSIC spectrum weight for each frequency bin.
 *
 * @parameter_name A_CHAR_SCALING
 * @parameter_type float
 * @parameter_value 1.0
 * @parameter_valid SPECTRUM_WEIGHT_TYPE=A_Characteristic
 * @parameter_description Scaling factor of the A-Weight with respect to frequency
 * 
 * @parameter_name MANUAL_WEIGHT_SPLINE
 * @parameter_type object
 * @parameter_value <Matrix<float> <rows 2> <cols 5> <data 0.0 2000.0 4000.0 6000.0 8000.0 1.0 1.0 1.0 1.0 1.0> >
 * @parameter_valid SPECTRUM_WEIGHT_TYPE=Manual_Spline 
 * @parameter_description MUSIC spectrum weight for each frequency bin. This is a 2 by M matrix. The first row represents the frequency, and the second row represents the weight gain. "M" represents the number of key points for the spectrum weight. The frequency range between M key points will be interpolated by spline manner. The format is "<Matrix<float> <rows 2> <cols 2> <data 1 2 3 4> >". 
 *  
 * @parameter_name MANUAL_WEIGHT_SQUARE 
 * @parameter_type object 
 * @parameter_value <Vector<float> 0.0 2000.0 4000.0 6000.0 8000.0> 
 * @parameter_valid SPECTRUM_WEIGHT_TYPE=Manual_Square 
 * @parameter_description MUSIC spectrum weight for each frequency bin. This is a M order vector. The element represents the frequency points for the square wave. "M" represents the number of key points for the square wave weight. The format is "<Vector<float> 1 2 3 4>". 
 * 
 * @parameter_name ENABLE_EIGENVALUE_WEIGHT 
 * @parameter_type bool 
 * @parameter_value true 
 * @parameter_list true:false 
 * @parameter_description If true, the spatial spectrum is weighted depending on the eigenvalues of a correlation matrix. We do not suggest to use this function with GEVD and GSVD, because the NOISECM changes the eigenvalue drastically. Only useful for SEVD. 
 *
 * @parameter_name DEBUG
 * @parameter_type bool
 * @parameter_value false
 * @parameter_list true:false
 * @parameter_description Debug option. If the parameter is true, this node outputs sound localization results to a standard output.
 *
END*/

class LocalizeMUSIC : public BufferedNode {

  int inputID;
  int paramsID;
  int noisecmID;

  int outputID;

  int spectrumID;

  bool bspectrumID; 

  string music_algorithm;
  int nb_channels;
  Vector<int> channel_selection;
  vector<int> ch_selection;
  int length;
  int samplingRate;
  string FilenameAMatrix;
  int num_source;
  int mindeg;
  int maxdeg;
  int lower_bound;
  int upper_bound;
  float t_s;
  string spectrum_weight_type;
  Matrix<float> SSW;
  Vector<float> SSW_SQ;
  ublas::vector<float> MSSW;
  bool enable_eigenvalue_weight;
  bool is_debug;

  int pslength;
  fcmatrix x;
  deque<fcmatrix> xbuf;
  vector<fcmatrix> Rxx;
  vector<fcmatrix> RxxN;
  int sum_count;
  int period;
  int window;
  string window_type;
  int look_ahead;
  int look_back;

  int nh, nd, nr;

  int RxxNinit_flag;

  scoped_ptr<LocalizationMUSIC> localizer;

  typedef Matrix<complex<float> > FCMatrix; // FlowdesignerÍÑ

public:
  LocalizeMUSIC(string nodeName, ParameterSet params)
    : BufferedNode(nodeName, params),
      paramsID(-1), noisecmID(-1), spectrumID(-1), bspectrumID(false)
  {
    inputID = addInput("INPUT");
    outputID = addOutput("OUTPUT");

    music_algorithm = object_cast<string>(parameters.get("MUSIC_ALGORITHM"));
    channel_selection = object_cast<Vector<int> >(parameters.get("TF_CHANNEL_SELECTION"));      
    nb_channels = (int)channel_selection.size();
    ch_selection.resize(nb_channels);
    for(unsigned int i = 0; i < channel_selection.size(); i++) ch_selection[i] = channel_selection[i];
    length = dereference_cast<int>(parameters.get("LENGTH"));
    samplingRate = dereference_cast<int>(parameters.get("SAMPLING_RATE"));
    FilenameAMatrix = object_cast<String>(parameters.get("A_MATRIX"));
    window = dereference_cast<int>(parameters.get("WINDOW"));
    window_type = object_cast<String>(parameters.get("WINDOW_TYPE"));
    period = dereference_cast<int>(parameters.get("PERIOD"));
    num_source = dereference_cast<int>(parameters.get("NUM_SOURCE"));
    mindeg = dereference_cast<int>(parameters.get("MIN_DEG"));
    maxdeg = dereference_cast<int>(parameters.get("MAX_DEG"));
    lower_bound = dereference_cast<int>(parameters.get("LOWER_BOUND_FREQUENCY"));
    upper_bound = dereference_cast<int>(parameters.get("UPPER_BOUND_FREQUENCY"));
    t_s = dereference_cast<float>(parameters.get("A_CHAR_SCALING"));
    spectrum_weight_type = object_cast<String>(parameters.get("SPECTRUM_WEIGHT_TYPE"));
    enable_eigenvalue_weight = dereference_cast<bool>(parameters.get("ENABLE_EIGENVALUE_WEIGHT"));
    is_debug = dereference_cast<bool>(parameters.get("DEBUG"));

    pslength = length / 2 + 1;

    if(spectrum_weight_type == "Manual_Square"){ 
      SSW_SQ = object_cast<Vector<float> >(parameters.get("MANUAL_WEIGHT_SQUARE"));       
      MSSW.resize(pslength); 
      for(unsigned int i = 0; i < pslength; i++){ 
	int SSW_j = -1; 
	for(unsigned int j = 0; j < SSW_SQ.size(); j++){ 
	  if(SSW_SQ[j] <= samplingRate * (float)i / (float)length) SSW_j = j;   
	} 
	if(SSW_j % 2 == 0) MSSW[i] = 1.0; else MSSW[i] = 0.0; 
      } 
    } else { 
      if(spectrum_weight_type == "Manual_Spline"){ 
	SSW = object_cast<Matrix<float> >(parameters.get("MANUAL_WEIGHT_SPLINE")); 
      }else if(spectrum_weight_type == "Uniform"){ 
	SSW.resize(2,5); 
	SSW(0,0) = 0.0 / 4.0 / 2.0 * (float)samplingRate; 
	SSW(0,1) = 1.0 / 4.0 / 2.0 * (float)samplingRate; 
	SSW(0,2) = 2.0 / 4.0 / 2.0 * (float)samplingRate; 
	SSW(0,3) = 3.0 / 4.0 / 2.0 * (float)samplingRate; 
	SSW(0,4) = 4.0 / 4.0 / 2.0 * (float)samplingRate; 
	SSW(1,0) = 1.0; SSW(1,1) = 1.0; SSW(1,2) = 1.0; SSW(1,3) = 1.0; SSW(1,4) = 1.0;  
      }else if(spectrum_weight_type == "A_Characteristic"){ 
	SSW.resize(2,16); 
	SSW(0,0)   = 10.0 * t_s;   SSW(0,1)  = 12.5 * t_s;   SSW(0,2)  = 16.0 * t_s;    SSW(0,3)  = 20.0 * t_s; 
	SSW(0,4)  = 31.5 * t_s;    SSW(0,5) = 63.0 * t_s;    SSW(0,6)  = 125.0 * t_s;   SSW(0,7)  = 250.0 * t_s;   
	SSW(0,8)  = 500.0 * t_s;   SSW(0,9)  = 1000.0 * t_s; SSW(0,10) = 2000.0 * t_s;  SSW(0,11) = 4000.0 * t_s;  
	SSW(0,12) = 8000.0 * t_s; SSW(0,13) = 12500.0 * t_s; SSW(0,14) = 16000.0 * t_s; SSW(0,15) = 20000.0 * t_s;  
	SSW(1,0)   = pow(10.0,-70.4/20.0);  SSW(1,1)  = pow(10.0,-63.4/20.0);  SSW(1,2)  = pow(10.0,-56.7/20.0);    
	SSW(1,3)   = pow(10.0,-50.5/20.0);  SSW(1,4)  = pow(10.0,-39.4/20.0);  SSW(1,5) = pow(10.0,-26.2/20.0);  
	SSW(1,6)   = pow(10.0,-16.1/20.0);  SSW(1,7)  = pow(10.0,-8.6/20.0);   SSW(1,8)  = pow(10.0,-3.2/20.0);     
	SSW(1,9)   = pow(10.0,0.0/20.0);    SSW(1,10) = pow(10.0,1.2/20.0);    SSW(1,11)  = pow(10.0,1.0/20.0);   
	SSW(1,12)  = pow(10.0,-1.1/20.0);   SSW(1,13) = pow(10.0,-4.3/20.0);   SSW(1,14) = pow(10.0,-6.6/20.0);     
	SSW(1,15)  = pow(10.0,-9.3/20.0);        
      } 
      ublas::vector<float> KEY_X(SSW.ncols()); 
      ublas::vector<float> KEY_Y(SSW.ncols()); 
      for(unsigned int i = 0; i < SSW.ncols(); i++){ 
	KEY_X[i] = SSW(0,i); 
	KEY_Y[i] = SSW(1,i); 
      } 
      ublas::vector<float> SPLINE_X(pslength); 
      ublas::vector<float> SPLINE_Y(pslength); 
      for(unsigned int i = 0; i < SPLINE_X.size(); i++){ 
	SPLINE_X[i] = samplingRate * (float)i / (float)length; 
	if((SPLINE_X[i] < SSW(0,0))||(SPLINE_X[i] > SSW(0,SSW.ncols()-1))){ 
	  SPLINE_Y[i] = 0.0; 
	  continue; 
	} 
	SPLINE_Y[i] = cubic_spline(KEY_X, KEY_Y, SPLINE_X[i]); 
      } 
      MSSW.resize(pslength); 
      // noalias(MSSW) = SPLINE_Y / (float)ublas::sum(SPLINE_Y) * (float)pslength; 
      noalias(MSSW) = SPLINE_Y;
    } 

    x.resize(nb_channels, pslength);
    xbuf.resize(0);
    Rxx.resize(pslength);
    RxxN.resize(pslength);
    for (int k = 0; k < pslength; k++) {
      //Rxx[k].resize(nb_channels, nb_channels);
      Rxx[k] = ublas::zero_matrix<complex<float> >(nb_channels, nb_channels);
      RxxN[k] = ublas::zero_matrix<complex<float> >(nb_channels, nb_channels);
    }

    RxxNinit_flag = 0;

    if (num_source >= nb_channels) {
      throw new NodeException(NULL, string("Number of sources should be less than number of channels."), __FILE__, __LINE__);
    }

    localizer.reset(new LocalizationMUSIC(nb_channels, samplingRate,
                                          length, length));
    cout << "reading A matrix\n";
    localizer->ReadTransferFunction(FilenameAMatrix.c_str(), ch_selection);
    cout << "done\n";
    localizer->SetNumSource(num_source);
    localizer->SetUsedFrequency(lower_bound, upper_bound);

    nh = localizer->GetSearchHeight().size();
    nd = localizer->GetSearchDirection().size();
    nr = localizer->GetSearchRange().size();    

    if(window_type == "FUTURE"){
      look_ahead = window - 1;
      look_back = 0;
    }else if(window_type == "MIDDLE"){
      look_ahead = window / 2;
      look_back = window / 2 + (window % 2) - 1;
    }else if(window_type == "PAST"){
      look_ahead = 0;
      look_back  = 0;
    }

    inputsCache[inputID].lookAhead = look_ahead;
    inputsCache[inputID].lookBack = look_back;

    sum_count = 0;

    inOrder = true;

  }

  virtual void initialize()
  {

    int max_lookAhead = outputs[outputID].lookAhead; 
    int max_lookBack  = outputs[outputID].lookBack; 
    
    if(spectrumID != -1){ 
      max_lookAhead = max(max_lookAhead, outputs[spectrumID].lookAhead); 
      max_lookBack  = max(max_lookBack,  outputs[spectrumID].lookBack);       
    } 
    
    outputs[outputID].lookAhead = 1 + max_lookAhead; 
    outputs[outputID].lookBack  = 1 + max_lookBack;     
    
    if(spectrumID != -1){ 
      outputs[spectrumID].lookAhead = 1 + max_lookAhead; 
      outputs[spectrumID].lookBack  = 1 + max_lookBack; 
    }
    
    this->BufferedNode::initialize();
  }

  virtual int translateInput(string inputName) {
    if (inputName == "PARAMS") {
      paramsID = addInput(inputName);
      return paramsID;
    }
    else if (inputName == "NOISECM") {
      noisecmID = addInput(inputName);
      return noisecmID;
    }
    else if (inputName == "INPUT") {
      for (unsigned int i = 0; i < inputs.size(); i++) {
	if (inputs[i].name == inputName) {
	  return i;
	}
      }
    }
    else {throw new NodeException(this, inputName+ " is not supported.", __FILE__, __LINE__);}
  }

  // dynamic output-port translation
  virtual int translateOutput (string outputName)
  {

    if (outputName == "SPECTRUM") {
      if(bspectrumID){
	return spectrumID;
      }else{
	bspectrumID = true;
	return spectrumID = addOutput(outputName);
      }
    }

    for (unsigned int i=0; i< outputNames.size(); i++) {	
      if (outputNames[i] == outputName) {
	return i;
      }
    }  
    
    return addOutput(outputName);
  }

  void calculate(int output_id, int count, Buffer &out)
  {
    
    // Load Noise Correlation Matrix

    if(music_algorithm != "SEVD"){

      if(noisecmID == -1){
	
	// Use identity matrix if NOISECM is not connected
	for(int k = 0 ; k < pslength ; k++){
	  for (int i = 0; i < nb_channels; i++) {
	    for (int j = 0; j < nb_channels; j++) {
	      if(i==j){
		RxxN[k](i, j) = complex<float>(1.0f,0.0f);
	      }else{
		RxxN[k](i, j) = complex<float>(0.0f,0.0f);
	      }
	    }
	  }
	}
	
      }
      else {
	
	RCPtr<FCMatrix> noise = getInput(noisecmID, count);
      
	int iNVecSrc = noise->nrows();  // num. of elements of frequency series (should be equal to pslength)
      
	if((noise->nrows() == 0)&&(RxxNinit_flag==0)){
	  // Use identity matrix if RxxN is not initialized
	  for(int k = 0 ; k < pslength ; k++){
	    for (int i = 0; i < nb_channels; i++) {
	      for (int j = 0; j < nb_channels; j++) {
		if(i==j){
		  RxxN[k](i, j) = complex<float>(1.0f,0.0f);
		}else{
		  RxxN[k](i, j) = complex<float>(0.0f,0.0f);
		}
	      }
	    }
	  }
	  RxxNinit_flag = 1;
	}else{
	  for(int k = 0 ; k < iNVecSrc ; k++){
	    for (int i = 0; i < nb_channels; i++) {
	      for (int j = 0; j < nb_channels; j++) {
		RxxN[k](i, j) = (*noise)(k, i*nb_channels + j);
	      }
	    }
	  }      
	}
      
      }

    }
        
    // Save data vector for spatial spectrum
    RCPtr<Vector<float > > spectrum(new Vector<float >(nh * nd * nr));    
    if(spectrumID != -1){    
      (*(outputs[spectrumID].buffer))[count] = spectrum;
      for (unsigned int i = 0; i < nh * nd * nr; i++) { 
	(*spectrum)[i] = 0.0f; 
      } 
    }

    // Current Correlation Matrix Generation

    RCPtr<Matrix<complex<float> > > inputp;

    // Vector<ObjectRef>& sources = *new Vector<ObjectRef>;
    // (*(outputs[outputID].buffer))[count] = &sources;
    RCPtr<Vector<ObjectRef> > sources(Vector<ObjectRef>::alloc(0));
    (*(outputs[outputID].buffer))[count] = sources;

    if(window_type == "PAST"){

      // inputCashe 
      inputp = getInput(inputID, count); 
      if (count == 0) { 
	if (!(channel_selection.size() == (*inputp).nrows())) { 
	  throw new NodeException(this, string("LocalizeMUSIC. Input channel size is not matched to the steering array."), __FILE__, __LINE__); 
	} 
      } 
      for (int chan = 0; chan < nb_channels; chan++) { 
	for (int k = 0; k < pslength; k++) { 
	  x(chan, k) = (*inputp)(chan, k); 
	} 
      } 
      if(xbuf.size() < window){ 
	xbuf.push_back(x); 
      }else{ 
	xbuf.push_back(x);       
	xbuf.pop_front();       
      } 
      
      if ((count % period == 0)&&((count != 0)||(period == 1))) {
	
	for (int i = 0; i < xbuf.size(); i++) { 
	  
	  for (int chan = 0; chan < nb_channels; chan++) {
	    for (int k = 0; k < pslength; k++) {
	      x(chan, k) = xbuf[i](chan,k);
	    }
	  }
	  
	  AddToCorrelation(Rxx, localizer->GetUsedFrequency(), x);
	  sum_count++;
	}
	NormalizeCorrelation(Rxx, localizer->GetUsedFrequency(), sum_count);
	sum_count = 0;
      }else {
	return;
      }
      
    }else{
      
      if (count % period == 0 && count >= look_back) { 
	for (int i = -look_back; i <= look_ahead; i++) { 
	  
	  inputp = getInput(inputID, count + i); 
	  
	  if (count == 0) { 
	    if (!(channel_selection.size() == (*inputp).nrows())) { 
	      throw new NodeException(this, string("LocalizeMUSIC. Input channel size is not matched to the steering array."), __FILE__, __LINE__); 
	    } 
	  } 
	  
	  for (int chan = 0; chan < nb_channels; chan++) {
	    for (int k = 0; k < pslength; k++) {
	      x(chan, k) = (*inputp)(chan, k);
	    }
	  }
	  
	  AddToCorrelation(Rxx, localizer->GetUsedFrequency(), x);
	  sum_count++;
	}
	NormalizeCorrelation(Rxx, localizer->GetUsedFrequency(), sum_count);
	sum_count = 0;
      }else {
	return;
      }
    }

#ifdef DEBUG
    ptime start_time;
    ptime end_time;
    start_time = microsec_clock::local_time();
#endif
    
    // MUSIC with GEVD
    
    localizer->Localize(Rxx, RxxN, mindeg, maxdeg, music_algorithm, MSSW, enable_eigenvalue_weight);
    
    int n_result = localizer->GetFoundSourceCount();

    for (int i = 0; i < n_result; i++) {
      if (is_debug) {
	  printf("%d: %7.2f %7.2f %7.2f %7.2f\t", i,
		 (float)(localizer->GetSearchHeight()[localizer->GetFoundHeight()[i]]),
		 (float)(localizer->GetSearchDirection()[localizer->GetFoundDirection()[i]]),
		 (float)(localizer->GetSearchRange()[localizer->GetFoundRange()[i]]),
		 localizer->GetP(localizer->GetFoundHeight()[i],localizer->GetFoundDirection()[i],localizer->GetFoundRange()[i]));
      }
                
      RCPtr<Source> src(new Source);
            
      const int found_height    = localizer->GetFoundHeight()[i];
      const int found_direction = localizer->GetFoundDirection()[i];
      const int found_range     = localizer->GetFoundRange()[i];
      src->id = 0;
      src->power = localizer->GetP(found_height,found_direction,found_range);
      float theta = localizer->GetSearchDirection()[found_direction] * M_PI / 180.0f;
      float r = localizer->GetSearchRange()[found_range];
      float phi = localizer->GetSearchHeight()[found_height] * M_PI / 180.0f;
      src->x[0] = r * cos(theta) * cos(phi);
      src->x[1] = r * sin(theta) * cos(phi);
      src->x[2] = r * sin(phi);
      sources->push_back(src);
      
    }

    if (is_debug) {
      printf("\n");
      printf("MUSIC spectrum: ");
      for (int i = 0; i < nh; i++) {
	for (int j = 0; j < nd; j++) {
	  for (int k = 0; k < nr; k++) {
	    printf("%f ", localizer->GetP(i,j,k));
	  }
	}
      }
      printf("\n");
    }

    // Output the spacial spectrum
    if(spectrumID != -1){
      for (int i = 0; i < nh; i++) {
	for (int j = 0; j < nd; j++) {
	  for (int k = 0; k < nr; k++) {
	    (*spectrum)[k + j * nr + i * nr * nd] = localizer->GetP(i,j,k);
	  }
	}
      }    
    }

    //cout << sources.size() << endl;

    ClearCorrelation(Rxx, localizer->GetUsedFrequency());
    ClearCorrelation(RxxN, localizer->GetUsedFrequency());

#ifdef DEBUG
      end_time = microsec_clock::local_time();
      cout << end_time - start_time << endl;
#endif

    if(paramsID != -1){
      RCPtr<Vector<ObjectRef> > param_ptr = getInput(paramsID, count);
      const Vector<ObjectRef>& param_vec = *param_ptr;      
      if(param_vec.size() != 0){	
	RCPtr<ParamsLocalizeMUSIC> value_LocalizeMUSIC = param_vec[0];
	num_source = (*value_LocalizeMUSIC).num_source;
	localizer->SetNumSource(num_source);
	mindeg = (*value_LocalizeMUSIC).min_deg;
	maxdeg = (*value_LocalizeMUSIC).max_deg;
	lower_bound = (*value_LocalizeMUSIC).lower_bound_frequency;
	upper_bound = (*value_LocalizeMUSIC).upper_bound_frequency;
	localizer->SetUsedFrequency(lower_bound, upper_bound);
      }
    }

  }

  //IN_ORDER_NODE_SPEEDUP(LocalizeMUSIC);
};

#endif
