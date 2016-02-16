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

#include <cstdlib>
#include <string.h>

#include "LocalizationFunction.hpp"

using namespace std;
using namespace boost;
using namespace FD;
using namespace boost::numeric;

class CMSave;

DECLARE_NODE(CMSave);
/*Node
 *
 * @name CMSave
 * @category HARK:Localization:CorrelationMatrix
 * @description This module saves a correlation matrix file when OPERATION_FLAG=1.
 *
 * @input_name INPUTCM
 * @input_type Matrix<complex<float> >
 * @input_description  Correlation matrix. This is a 2D matrix. The row denotes the frequency series.  The col denotes the correlation matrix of each frequency. The col should be transformed to a matrix in the receiver.
 *
 * @input_name FILENAMER
 * @input_type string
 * @input_description file name of the saved correlation matrix (real part)
 *
 * @input_name FILENAMEI
 * @input_type string
 * @input_description file name of the saved correlation matrix (imaginary part)
 *
 * @input_name OPERATION_FLAG
 * @input_type any
 * @input_description Integer or bool flag. The data saving is done if this flag is 1/true. Set this 0/false and make this 1/true at appropriate time since this block costs a lot of computation power. If this flag is 0/false, the operation is done only when this is 1/true. 
 *
 * @output_name OUTPUTCM
 * @output_type Matrix<complex<float> >
 * @output_description Same as INPUTCM
 *
 * @parameter_name ENABLE_DEBUG
 * @parameter_type bool
 * @parameter_list true:false
 * @parameter_value false
 * @parameter_description enable debug print
 *
END*/

class CMSave : public BufferedNode {
  int inputcmID;
  int filenamerID;
  int filenameiID;
  int operationID;
  int outputcmID;

  int nb_channels;

  int operation_flag;
  bool enable_debug;

  int pslength;
  vector<fcmatrix> Rxx;

  typedef Matrix<complex<float> > FCMatrix; // FlowdesignerÍÑ

public:
  CMSave(string nodeName, ParameterSet params)
    : BufferedNode(nodeName, params)
  {
    inputcmID = addInput("INPUTCM");
    filenamerID = addInput("FILENAMER");
    filenameiID = addInput("FILENAMEI");
    operationID = addInput("OPERATION_FLAG");
    outputcmID = addOutput("OUTPUTCM");

    enable_debug = dereference_cast<bool>(parameters.get("ENABLE_DEBUG"));

    operation_flag = 0;

    inOrder = true;
  }

  void calculate(int output_id, int count, Buffer &out)
  {
    
    RCPtr<FCMatrix> inputcm = getInput(inputcmID, count);
    (*(outputs[outputcmID].buffer))[count] = inputcm;

    if(count == 0){
      pslength = inputcm->nrows();
      nb_channels = (int)(floor(sqrtf((float)inputcm->ncols())));    
      if(inputcm->ncols() != nb_channels * nb_channels)
	throw new NodeException(this, string("CMSave. Incorrect correlation matrix size."), __FILE__, __LINE__);
      Rxx.resize(pslength);
      for (int k = 0; k < pslength; k++) {
	Rxx[k] = ublas::zero_matrix<complex<float> >(nb_channels, nb_channels);
      }
    }

    ObjectRef filenamerValue = getInput(filenamerID, count);
    const String &fileNamer = object_cast<String> (filenamerValue);

    ObjectRef filenameiValue = getInput(filenameiID, count);
    const String &fileNamei = object_cast<String> (filenameiValue);

    char charnamer[100] = "";
    char charnamei[100] = "";

    strcpy(charnamer,fileNamer.c_str());
    strcpy(charnamei,fileNamei.c_str());

    ObjectRef operation = getInput(operationID, count);
    if (typeid(*operation) == typeid(Bool)) { 
      bool bInput = dereference_cast<bool>(operation); 
      operation_flag = (bInput ? 1 : 0); 
    }else if(typeid(*operation) == typeid(Int)){ 
      operation_flag = dereference_cast<int> (operation); 
    }else{ 
      throw new NodeException(this, string("Input type is not appropriate in CMSave."), __FILE__, __LINE__); 
    } 

    if(operation_flag == 1){    

      for(int k = 0 ; k < pslength ; k++){
	for (int i = 0; i < nb_channels; i++) {
	  for (int j = 0; j < nb_channels; j++) {
	    Rxx[k](i, j) = (*inputcm)(k, i*nb_channels + j);
	  }
	}
      }

      if(enable_debug) printf("Count[%d] : CM Saved (Size : %d x %d x %d)\n", count, Rxx.size(), Rxx[0].size1(), Rxx[0].size2());
      
      SaveCorrelation(charnamer, charnamei, Rxx);

    }
    
  }

  //IN_ORDER_NODE_SPEEDUP(CMSave);
};
