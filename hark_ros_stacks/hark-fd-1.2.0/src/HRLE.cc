// ---------------------------------------------------------------------
/**
 Copyright (c) 2008 Honda Research Institute Japan, Co., Ltd.

 \file HRLE.cc
 \brief	\~English It executes HRLE.
 \brief	\~Japanese HRLE$B$N<B9T(J
 \author	Hirofumi Nakajima
 \date	5 Aug. 2009
 \version	$Id: HRLE.cc  $
*/
// ---------------------------------------------------------------------
//#include <complex.h>
#include <stdlib.h>
#include "BufferedNode.h" 
#include "Map.h" // map+Object
#include "Vector.h" // vector+BaseVector(Object) 
//#include <complex>
//#include <algorithm>
//#include "Matrix.h"

//#include "Source.h"
// ---------------------------------------------------------------------
//#include "MICARYHA_struct.h"
//#include "MICARY_struct.h"
//#include "MICARY_PF_Struct.h"
//#include "ArrayInit.h"
//#include "PF_Init.h"
#include "lppostv1forHARK.cpp"
#include "HarkParamsObjects.h"

// ---------------------------------------------------------------------

// namespace$B;HMQ$r@k8@$9$k$3$H(J
using namespace FD;
using namespace std;

//forward declaration of class VAdd for use with the DECLARE_NODE macro
class HRLE; // $B"+%N!<%I(J

//Declaration of the node. This definition is transformed into XML data for the GUI, as well as documentation for the node
// $B%N!<%I$NF~=PNOC<;R$r:n@.$9$k(J
DECLARE_NODE(HRLE) // $B"+%N!<%I(J

/*Node
 *  
 * @name HRLE
 * @category HARK:Separation
 * @description This executes HRLE.
 *
 * @input_name INPUT_SPEC
 * @input_type Map<int,ObjectRef>
 * @input_description Input spectrum.The key is a source ID,and the value is state Input power spectrum(Vector<float>)
 *
 * @output_name NOISE_SPEC
 * @output_type Map<int,ObjectRef>
 * @output_description Estimated Noise spectrum.The key is a source ID,and the value is the noise spectrum(Vector<float>)
 *   
 * @parameter_name LX
 * @parameter_type float
 * @parameter_value 0.85
 * @parameter_description Lx value of estimation, e.g. Lx=0 -> Minimum (MCRA), Lx=0.5 -> Median , Lx=1.0 -> Maximum [default:0.85]
 *
 * 
 * @parameter_name TIME_CONSTANT
 * @parameter_type float
 * @parameter_value 16000
 * @parameter_description Time constant for exponential decay window in samples [default:16000]
 *
 * @parameter_name NUM_BIN
 * @parameter_type float
 * @parameter_value 1000
 * @parameter_description Number of histogram bins [default:1000]
 *
 * 
 * @parameter_name MIN_LEVEL
 * @parameter_type float
 * @parameter_value -100
 * @parameter_description Minimum level of histogram bin in dB [default:-100]
 *
 * 
 * @parameter_name STEP_LEVEL
 * @parameter_type float
 * @parameter_value 0.2
 * @parameter_description Step level of histogram bin (Width of each histogram bin) in dB [default:0.2]
 *
 * @parameter_name DEBUG
 * @parameter_type bool
 * @parameter_value false
 * @parameter_list true:false
 * @parameter_description Prints the histogram for each 100 iterations.

END*/

// ---------------------------------------------------------------------

//Class definition/implementation. Note that because we won't need to derive from this class, we don't need a header file (.h) and we can put everything in the .cc. Our node, like most other nodes, derives from BufferedNode.

class HRLE : public BufferedNode 
{

  int m_iInputSpecID; // $BF~NO%9%Z%/%H%k(JID
  int paramsID;
  int m_iNoiseSpecID; // $B=PNO%9%Z%/%H%k(JID
  HPostv4 PF; // postfilter class (lppost1forHARK.cpp)
  bool isDebug;
public:
    // $B%3%s%9%H%i%/%?!J=i4|2=!K(J
	HRLE(string nodeName, ParameterSet params)
	  : BufferedNode(nodeName, params),
	    paramsID(-1)
	{ 
        inOrder = true; // BufferedNode$B$N%a%s%P(J

        // $BF~=PNO(JID$B$N<hF@(J
		m_iInputSpecID = addInput("INPUT_SPEC");
		m_iNoiseSpecID = addOutput("NOISE_SPEC");		
		
		//$B%Q%i%a!<%?$N<hF@(J
//		if (parameters.exist("Lx")){}else{} $B$H$9$k$H6uGr$N;~$N=hM}$,$G$-$k(J
        PF.Lx   = dereference_cast<float>(parameters.get("LX"));
        PF.tr   = 1 - 1./ (dereference_cast<float>(parameters.get("TIME_CONSTANT")));        
        PF.N    = dereference_cast<float>(parameters.get("NUM_BIN"));
        PF.Min  = dereference_cast<float>(parameters.get("MIN_LEVEL"));
        PF.Step = dereference_cast<float>(parameters.get("STEP_LEVEL"));
	isDebug = dereference_cast<bool>(parameters.get("DEBUG"));
	} 

	~HRLE(){} // some descriptions e.g. Free Memory

  virtual int translateInput(string inputName) {
    if (inputName == "PARAMS") {
      paramsID = addInput(inputName);
      return paramsID;
    }
    else if (inputName == "INPUT_SPEC") {
      for (unsigned int i = 0; i < inputs.size(); i++) {
	if (inputs[i].name == inputName) {
	  return i;
	}
      }
    }
    else {throw new NodeException(this, inputName+ " is not supported.", __FILE__, __LINE__);}
  }

  // $B7W;;%3%"(J
  void calculate(int output_id, int count, Buffer &out)
  {
		int iNFreq, iNSource;
		int nsrc, nfrq;
        Compmat X, X1, Y;
        vector<int> viID;
        vector<float> vftmp;

		//$BF~NO!"=PNO$N%P%C%U%!$r:n@.(J vector<int>$B7?(J 
		map<int,vector<float> > mInSpec; //$BF~NO(J
		map<int,vector<float> > mOutNoiseSpec; //$B=PNO(J


        RCPtr<Map<int,ObjectRef> > orInputSpecTemp(getInput(m_iInputSpecID, count)); // $B<BBN(J(Map)$B$r;X$9%]%$%s%?(J

	if(paramsID != -1){
	  RCPtr<Vector<ObjectRef> > param_ptr = getInput(paramsID, count);
	  const Vector<ObjectRef>& param_vec = *param_ptr;
	  if(param_vec.size() != 0){
	    RCPtr<ParamsHRLE> value_HRLE = param_vec[0];
	    PF.Lx = (*value_HRLE).lx;
	    PF.tr = 1.0 - 1.0 / (*value_HRLE).time_constant;
	  }
	}

		// Output$B$N=`Hw(J
        Buffer &NoiseSpecBuffer = *(outputs[m_iNoiseSpecID].buffer) ; // outputs[*ID].buffer$B$O(JFD$B$N=PNOMQJQ?t(J, ID$B$O%3%s%9%H%i%/%?$G@_Dj(J
        ObjectRef orNoiseSpecTemp(new Map<int,ObjectRef>) ; // $B6u$N(JObjectRef$B7?JQ?t!"=PNO$N3JG<MQ(J
        NoiseSpecBuffer[count] = orNoiseSpecTemp; // $B8=:_$N;~4V%+%&%s%H(Jcount$B$X!"=PNOJQ?t$X$N%]%$%s%?$rF~$l$k$N$,7h$^$j(J
        Map<int,ObjectRef> &mNoiseSpecvalue = object_cast<Map<int,ObjectRef> >(orNoiseSpecTemp); //$BL>A0$NIU$1D>$7$H7?JQ49(J(Map$B7?(J)

     
		Map<int,ObjectRef>::iterator       itTempOR; // Map$BMQ(J(FD$B$+$i$NF~NO(J)
        itTempOR = orInputSpecTemp->begin(); // FD$B$+$i$NF~NO%G!<%?(J(Map$B7?(J)$B$N%$%F%l!<%?(J($B=i$a$N%Z%"$r;X<((J)
        iNSource = orInputSpecTemp->size() ; //$B2;8;?t(J($B%Z%"?t(J)
		
		if (iNSource>0)
		{
			
			vector<float> &vfTemp = object_cast<Vector<float> >(itTempOR->second); // $B=i$a$N2;8;$NCf?H!J%9%Z%/%H%k!K(J
			iNFreq   = vfTemp.size() ;	//$B!!<~GH?t%S%s?t(J
			//cout << "#Freqnency bin = " << iNFreq << endl;


			X.zeros(iNSource, iNFreq); // $BF~NO%G!<%?NN0h3NJ](J
			viID.reserve(iNSource); // ID$BJ]B8MQNN0h3NJ](J
			for(nsrc = 0; itTempOR != orInputSpecTemp->end() ; itTempOR++, nsrc++){
				viID[nsrc]                   = itTempOR->first;
				vector<float> &vcfInputSpec = object_cast<Vector<float> >(itTempOR->second);
				//printf("Input[0]=%f ",vcfInputSpec[0]);
				for(nfrq = 0; nfrq < iNFreq; nfrq++){
					X(nsrc, nfrq) = vcfInputSpec[nfrq];
				}
			}
	        
		
			// postfilter$B$N<B9T!J3F2;8;$K$D$$$F!"FbIt?dDj;(2;%R%9%H%0%i%`$O6&DL$KMxMQ!K(J
			// $B!\=PNOMQJQ?t(JmNoiseSpecvalue( map<int, vector<float> >$B7?(J) $B$N3FMWAG$K%G!<%?$r5M$a$k(J
			for(nsrc = 0 ; nsrc < iNSource; nsrc++){ // $B2;8;$N%k!<%W(J

				ObjectRef         orOutTmp = ObjectRef(Vector<float>::alloc(iNFreq)) ; // $B=PNOMQJQ?t!J#12;8;J,!K(J, ObjectRef(Vector<float>)$B7?(J, $B6u(J
				vector<float> & vcfOutTmp = object_cast<vector<float> >(orOutTmp) ; // $B7?$r(Jvector<float>$B$KL>A0$O(JvcfOutputSpec, $B6u(J

				X1 = X.mid1(nsrc);
				Y = PF.execute(X1); // Y$B$K$O3F2;8;$G$N%N%$%:?dDjCM$,F~$k(J
				
				for(nfrq = 0; nfrq < iNFreq; nfrq++){
					vcfOutTmp[nfrq] = Y.r(nfrq); // vector<float>$B$K$D$a$k(J
				
				}
				mNoiseSpecvalue.insert(make_pair(viID[nsrc], orOutTmp)) ; // $B=PNOJQ?t$X$N<BBN$N3JG<(J
			}	
		}

		if(isDebug && count % 100 == 0){
		  PF.disp();
		}
	}

};
