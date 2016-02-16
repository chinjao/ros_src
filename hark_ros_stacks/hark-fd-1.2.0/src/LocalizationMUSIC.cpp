/*
 * Copyright 2008 Kyoto University and Honda Motor Co.,Ltd.
 * All rights reserved.
 * HARK was developed by researchers in Okuno Laboratory
 * at the Kyoto University and Honda Research Institute Japan Co.,Ltd.
 */

#define NDEBUG

#define ENABLE_TIMER 0

#include "../config.h"

#ifdef ENABLE_HARKIO

#include "LocalizationMUSIC.hpp"

#include <iostream>
#include <fstream>

#include <boost/bind.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include "evd.hpp"
#include "svd.hpp"
// #include "svdold.hpp" 

#ifdef ENABLE_HARKIO2
#include <harkio2/harkio.hpp>
#elif ENABLE_HARKIO1
#include <libharkio/libharkio.h> 
#endif

using namespace std;
using namespace boost;
using namespace boost::numeric;
using namespace boost::posix_time; 

#ifdef ENABLE_HARKIO2
using namespace harkio2;
#elif ENABLE_HARKIO1
using namespace harkio;
#endif

struct less_for_vec {
  bool operator()(vector<float>& v, int i, int j) { return v[i] < v[j]; }
};

LocalizationMUSIC::LocalizationMUSIC(int num_mic, int sampling_rate,
                                     int tlength, int shift) :
  NumMicrophones(num_mic), TimeLength(tlength), Shift(shift),
  SamplingRate(sampling_rate), R(num_mic, num_mic), V(num_mic, num_mic),
  RN(num_mic, num_mic), VN(num_mic, num_mic),
  TempCalc1(num_mic, num_mic), TempCalc2(num_mic, num_mic),
  TempCalc3(num_mic, num_mic), TempCalc4(num_mic, num_mic),
  TempCalc5(num_mic, num_mic), TempCalc6(num_mic, num_mic),
  TempCalc7(num_mic, num_mic), TempCalc8(num_mic, num_mic),
  TempCalc9(num_mic, num_mic),
  NumSource(num_mic)
{
  FreqLength = TimeLength / 2 + 1;

  UsesFreq.resize(FreqLength);

  EigenValue.resize(FreqLength);
  CEigenValue.resize(NumMicrophones);
  REigenValue.resize(NumMicrophones);
  MaxRxx.resize(FreqLength);
  MaxRxxN.resize(FreqLength);  
  E.resize(FreqLength);
  EN.resize(FreqLength);
  for (int k = 0; k < FreqLength; k++) {
    EigenValue[k].resize(NumMicrophones);
    E[k].resize(NumMicrophones, NumMicrophones);
    EN[k].resize(NumMicrophones, NumMicrophones);
  }

  FoundHeight.resize(NumSource);
  FoundDirection.resize(NumSource);
  FoundRange.resize(NumSource);

}

void LocalizationMUSIC::SetUsedFrequency(int lo, int up)
{
  int lower = (int)((float)lo * TimeLength / SamplingRate);
  int upper = (int)((float)up * TimeLength / SamplingRate);
  for (int k = lower; k < upper; k++) {
    UsesFreq[k] = true;
  }
}

void LocalizationMUSIC::Localize(vector<fcmatrix>& Rxx, vector<fcmatrix>& RxxN, 
				  int mindeg, int maxdeg, 
				  string music_alg, 
				  ublas::vector<float>& MSSW,
				  bool enable_eigenvalue_weight)
{

#if ENABLE_TIMER
  ptime start_time;
  ptime end_time;
  start_time = microsec_clock::local_time();
#endif

  for (int k = 0; k < UsesFreq.size(); k++) {

    MaxRxx[k] = 1.0;
    MaxRxxN[k] = 1.0;

    if (!UsesFreq[k]) {
      continue;
    }

    float max = MaxOfAbsValue(Rxx[k]);
    noalias(R) = Rxx[k] / (max + 1.0f);
    MaxRxx[k] = max + 1.0f;

    if(music_alg != "SEVD"){
      float maxN = MaxOfAbsValue(RxxN[k]);
      noalias(RN) = RxxN[k] / (maxN);
      MaxRxxN[k] = maxN;
    }

    //============= GEVD ==============
    //printf("%d ", k);
    
    if(music_alg == "SEVD"){
      evd(R, EigenValue[k], V);
    }
    else if (music_alg == "GEVD"){
      invMAT(RN,TempCalc1);
      evd(TempCalc1, REigenValue, VN);
      for(int i = 0; i < REigenValue.size(); i++) REigenValue[i] = pow(REigenValue[i], (float)0.5);    
      invMAT(VN,TempCalc2);
      for(int i = 0; i < TempCalc3.size1(); i++) {
	for(int j = 0; j < TempCalc3.size2(); j++) {
	  TempCalc3(i,j) = 0.0;
	  if(i==j) TempCalc3(i,j) = REigenValue[i];
	}
      }
      noalias(TempCalc6) = prod(TempCalc3,TempCalc2);
      noalias(TempCalc4) = prod(VN, TempCalc6);
      noalias(TempCalc7) = prod(R,TempCalc4);
      noalias(TempCalc5) = prod(TempCalc4, TempCalc7);
      evd(TempCalc5, EigenValue[k], V);
    }
    else if (music_alg == "GSVD"){
      invMAT(RN,TempCalc1);
      noalias(TempCalc2) = prod(TempCalc1, R);
      svd(TempCalc2, REigenValue, TempCalc4, TempCalc3);
      sort_eig_fvec_ascending(REigenValue, TempCalc4, EigenValue[k], V);
    }
    
    noalias(E[k]) = V;    
        
  }

  //printf("\n");
  
  
  CalcAveragePower(EigenValue, E, A, UsesFreq, NumSource, Pavg, Pavg_bin, MSSW, enable_eigenvalue_weight);

  for (int ih = 0; ih < SearchHeight.size(); ih++) {
    for (int id = 0; id < SearchDirection.size(); id++) {
      for (int ir = 0; ir < SearchRange.size(); ir++) {
	Pavg_sum[ih][id][ir] = 10 * log10(Pavg[ih][id][ir] + 1.0f);
      }
    }
  }
  
  FoundSourceCount = SearchLocation(Pavg_sum, Pavg, NumSource,
                                    FoundHeight, FoundDirection, FoundRange, SearchDirection, mindeg, maxdeg);

#if ENABLE_TIMER
  end_time = microsec_clock::local_time();
  cout << end_time - start_time << endl;
#endif

}

void LocalizationMUSIC::ReadTransferFunction(const char* filename,
					     vector<int>& ch)
{

#ifdef ENABLE_HARKIO2

  try
    {
      ifstream harkFileData(filename);
      shared_ptr<hark::HarkFileReader> reader(new hark::HarkFileBinaryReader(harkFileData));
      shared_ptr<hark::HarkFile> hark = reader->Read();
      
      shared_ptr<hark::hgtf::Fmt1_0> fmt = dynamic_pointer_cast<hark::hgtf::Fmt1_0>(hark->GetDataSet("hgtf::Fmt"));
      if (fmt == 0)
	{
	  cout << "this is not include Fmt dataset version 1.0" << endl;
	  exit(1);
	}
      shared_ptr<hark::hgtf::M2PG1_0> m2pg = dynamic_pointer_cast<hark::hgtf::M2PG1_0>(hark->GetDataSet("hgtf::M2PG"));
      if (m2pg == 0)
	{
	  cout << "this is not include M2PG dataset version 1.0" << endl;
	  exit(1);
	}
      
      int slength = fmt->GetNFFT() / 2 + 1;
      int nmic = fmt->GetNMic();
      int nmic2 = ch.size();
      for (int c = 0; c < ch.size(); c++) {
	if (!(ch[c] >= 0 && ch[c] < nmic)) {
	cerr << "LocalizationMUSIC. Nonexistent channel is selected." << endl;
	exit(1);
	}
      }

// if ND means number of TFs
#if 1

      // Calculate the number of heights
      vector<float> allHeight(m2pg->GetND());
      for (int ih = 0; ih < m2pg->GetND(); ih++) {
	allHeight[ih] = (float)m2pg->GetSourcePositions()[ih][0].GetZ();
      }
      sort(allHeight.begin(),allHeight.end());
      int numHeight = 1;
      for (int ih = 1; ih < allHeight.size(); ih++) {
	if(allHeight[ih] - allHeight[ih-1] >= 0.001) numHeight++;
      }
      int m2pgGetNR;
      m2pgGetNR = numHeight;
      int m2pgGetND;
      m2pgGetND = floor(m2pg->GetND() / numHeight);

      A.resize(extents[m2pgGetNR][m2pgGetND][1]); // This should include radius
      for (int ih = 0; ih < m2pgGetNR; ih++) {
	for (int id = 0; id < m2pgGetND; id++) {
	  for (int ir = 0; ir < 1; ir++) {
	    A[ih][id][ir].resize(slength, nmic2);
	  }
	}
      }

      SearchHeight.resize(m2pgGetNR);
      for (int ih = 0; ih < m2pgGetNR; ih++) {
	SearchHeight[ih] = (float)atan2(m2pg->GetSourcePositions()[ih][0].GetZ(),
					sqrt(m2pg->GetSourcePositions()[ih][0].GetX() * m2pg->GetSourcePositions()[ih][0].GetX() +
					     m2pg->GetSourcePositions()[ih][0].GetY() * m2pg->GetSourcePositions()[ih][0].GetY())) * 180.0 / M_PI;
      }
      SearchDirection.resize(m2pgGetND);
      for (int id = 0; id < m2pgGetND; id++) {
	SearchDirection[id] = (float)atan2(m2pg->GetSourcePositions()[id * m2pgGetNR][0].GetY(),
						 m2pg->GetSourcePositions()[id * m2pgGetNR][0].GetX()) * 180.0 / M_PI;
      }
      SearchRange.resize(1);
      for (int ir = 0; ir < 1; ir++) {
	SearchRange[ir] = (float)sqrt(m2pg->GetSourcePositions()[0][0].GetX() * m2pg->GetSourcePositions()[0][0].GetX() +
				    m2pg->GetSourcePositions()[0][0].GetY() * m2pg->GetSourcePositions()[0][0].GetY() +
				    m2pg->GetSourcePositions()[0][0].GetZ() * m2pg->GetSourcePositions()[0][0].GetZ());
      }
      
      for (int ih = 0; ih < m2pgGetNR; ih++) {
	for (int id = 0; id < m2pgGetND; id++) {
	  for (int ir = 0; ir < 1; ir++) {
	    for (int m = 0; m < nmic2; m++) {
	      for (int k = 0; k < slength; k++) {
		A[ih][id][ir](k, m) = m2pg->GetTFs()[m2pgGetNR * id + ih][ir][ch[m]][k];
	      }
	    }
	  }
	}
      }

// if NR means radius
#else

      A.resize(extents[1][m2pg->GetND()][m2pg->GetNR()]); // This should include height
      for (int ih = 0; ih < 1; ih++) {
	for (int id = 0; id < m2pg->GetND(); id++) {
	  for (int ir = 0; ir < m2pg->GetNR(); ir++) {
	    A[ih][id][ir].resize(slength, nmic2);
	  }
	}
      }

      SearchHeight.resize(1);
      for (int ih = 0; ih < 1; ih++) {
	SearchHeight[ih] = (float)atan2(m2pg->GetSourcePositions()[0][0].GetZ(),
					sqrt(m2pg->GetSourcePositions()[0][0].GetX() * m2pg->GetSourcePositions()[0][0].GetX() +
					     m2pg->GetSourcePositions()[0][0].GetY() * m2pg->GetSourcePositions()[0][0].GetY())) * 180.0 / M_PI;
      }
      SearchDirection.resize(m2pg->GetND());
      for (int id = 0; id < m2pg->GetND(); id++) {
	SearchDirection[id] = (float)atan2(m2pg->GetSourcePositions()[id][0].GetY(),
					   m2pg->GetSourcePositions()[id][0].GetX()) * 180 / M_PI;
      }
      SearchRange.resize(m2pg->GetNR());
      for (int ir = 0; ir < m2pg->GetNR(); ir++) {
	SearchRange[ir] = (float)sqrt(m2pg->GetSourcePositions()[0][ir].GetX() * m2pg->GetSourcePositions()[0][ir].GetX() +
				    m2pg->GetSourcePositions()[0][ir].GetY() * m2pg->GetSourcePositions()[0][ir].GetY() +
				    m2pg->GetSourcePositions()[0][ir].GetZ() * m2pg->GetSourcePositions()[0][ir].GetZ());
      }
      
      for (int ih = 0; ih < 1; ih++) {
	for (int id = 0; id < m2pg->GetND(); id++) {
	  for (int ir = 0; ir < m2pg->GetNR(); ir++) {
	    for (int m = 0; m < nmic2; m++) {
	      for (int k = 0; k < slength; k++) {
		A[ih][id][ir](k, m) = m2pg->GetTFs()[id][ir][ch[m]][k];
	      }
	    }
	  }
	}
      }

#endif
      cerr << "TF was loaded by libharkio2." << endl;

    }
  catch (const harkio2::AnalyzeError& e)
    {
      cerr << "Try to read " << filename << " as header-less MUSIC transfer function format.\n";    
      ::ReadTransferFunction(filename, A, SearchHeight, SearchDirection, SearchRange, ch);
    }

#elif ENABLE_HARKIO1

  if (HARK::checkIdentifier(filename) == HARKIO_TRUE) {
    HARK hark_read;
    hark_read.readFile(filename);

    cerr << "Try to read " << filename << " as HARK format." << endl;

    Chunk::VERSION_CHECK version_check;
    FMT* fmt = hark_read.getData()->getChunk<FMT>();
    // TODO: version check
    if (fmt->getFormatId() != FMT::M2PG) {
      // TODO: エラー処理
    }

    M2PG* m2pg = hark_read.getData()->getChunk<M2PG>();
    int slength = fmt->getNFFT() / 2 + 1;
    int nmic = fmt->getNMic();
    int nmic2 = ch.size(); 
    for (int c = 0; c < ch.size(); c++) { 
      if (!(ch[c] >= 0 && ch[c] < nmic)) { 
	cerr << "LocalizationMUSIC. Nonexistent channel is selected." << endl; 
	exit(1); 
      } 
    } 

// if ND means number of TFs
#if 1

    // Calculate the number of heights
    vector<float> allHeight(m2pg->getND());
    M2PG::SRC_LIST_TYPE* srch = m2pg->getSrc();
    for (int ih = 0; ih < m2pg->getND(); ih++) {
      allHeight[ih] = (float)(*srch)[ih][0].z;
    }
    sort(allHeight.begin(),allHeight.end());
    int numHeight = 1;
    for (int ih = 1; ih < allHeight.size(); ih++) {
      if(allHeight[ih] - allHeight[ih-1] >= 0.001) numHeight++;
    }
    int m2pggetNR;
    m2pggetNR = numHeight;
    int m2pggetND;
    m2pggetND = floor(m2pg->getND() / numHeight);
      
    A.resize(extents[m2pggetNR][m2pggetND][1]); // This should include radius
    for (int ih = 0; ih < m2pggetNR; ih++) {
      for (int id = 0; id < m2pggetND; id++) {
	for (int ir = 0; ir < 1; ir++) {
	  A[ih][id][ir].resize(slength, nmic2);
	}
      }
    }
    M2PG::SRC_LIST_TYPE* src = m2pg->getSrc();

    SearchHeight.resize(m2pggetNR);
    for (int ih = 0; ih < m2pggetNR; ih++) {
      SearchHeight[ih] = (float)atan2((*src)[ih][0].z, 
				      sqrt((*src)[ih][0].x * (*src)[ih][0].x +
					   (*src)[ih][0].y * (*src)[ih][0].y)) * 180 / M_PI;
    }
    SearchDirection.resize(m2pggetND);
    for (int id = 0; id < m2pggetND; id++) {
      SearchDirection[id] = (float)atan2((*src)[m2pggetNR * id][0].y,
					 (*src)[m2pggetNR * id][0].x) * 180 / M_PI;
    }
    SearchRange.resize(1);
    for (int ir = 0; ir < 1; ir++) {
      SearchRange[ir] = (float)sqrt((*src)[0][0].x * (*src)[0][0].x +
                                  (*src)[0][0].y * (*src)[0][0].y +
                                  (*src)[0][0].z * (*src)[0][0].z);
    }

    M2PG::TF_LIST_TYPE* tf = m2pg->getTF();
    for (int ih = 0; ih < m2pggetNR; ih++) {
      for (int id = 0; id < m2pggetND; id++) {
	for (int ir = 0; ir < 1; ir++) {
	  for (int m = 0; m < nmic2; m++) {
	    for (int k = 0; k < slength; k++) {
	      A[ih][id][ir](k, m) = (*tf)[m2pggetNR * id + ih][ir][ch[m]][k];
	    }
	  }
	}
      }
    }

// if NR means radius
#else

    A.resize(extents[1][m2pg->getND()][m2pg->getNR()]); // This should include height
    for (int ih = 0; ih < 1; ih++) {
      for (int id = 0; id < m2pg->getND(); id++) {
	for (int ir = 0; ir < m2pg->getNR(); ir++) {
	  A[ih][id][ir].resize(slength, nmic2);
	}
      }
    }
    M2PG::SRC_LIST_TYPE* src = m2pg->getSrc();

    SearchHeight.resize(1);
    for (int ih = 0; ih < 1; ih++) {
      SearchHeight[ih] = (float)atan2((*src)[0][0].z,
				      sqrt((*src)[0][0].x * (*src)[0][0].x +
					   (*src)[0][0].y * (*src)[0][0].y)) * 180 / M_PI;
    }
    SearchDirection.resize(m2pg->getND());
    for (int id = 0; id < m2pg->getND(); id++) {
      SearchDirection[id] = (float)atan2((*src)[id][0].y,
					 (*src)[id][0].x) * 180 / M_PI;
    }
    SearchRange.resize(m2pg->getNR());
    for (int ir = 0; ir < m2pg->getNR(); ir++) {
      SearchRange[ir] = (float)sqrt((*src)[0][ir].x * (*src)[0][ir].x +
				    (*src)[0][ir].y * (*src)[0][ir].y +
				    (*src)[0][ir].z * (*src)[0][ir].z);
    }

    M2PG::TF_LIST_TYPE* tf = m2pg->getTF();
    for (int ih = 0; ih < 1; ih++) {
      for (int id = 0; id < m2pg->getND(); id++) {
	for (int ir = 0; ir < m2pg->getNR(); ir++) {
	  for (int m = 0; m < nmic2; m++) {
	    for (int k = 0; k < slength; k++) {
	      A[ih][id][ir](k, m) = (*tf)[id][ir][ch[m]][k];
	    }
	  }
	}
      }
    }

#endif
      cerr << "TF was loaded by libharkio1." << endl;
    
  }
  else {
    cerr << harkio::Error::getInstance().getMsg() << endl;
    cerr << "Try to read " << filename << " as header-less MUSIC transfer function format.\n";    
    ::ReadTransferFunction(filename, A, SearchHeight, SearchDirection, SearchRange, ch);
  }

#endif

#if 0

    cout << "Height [" << SearchHeight.size() << "] : ";
    for(int j = 0; j < SearchHeight.size(); j++){
      cout << SearchHeight[j] << " ";
    }
    cout << endl;

    cout << "Direction [" << SearchDirection.size() << "] : ";
    for(int j = 0; j < SearchDirection.size(); j++){
      cout << SearchDirection[j] << " ";
    }
    cout << endl;

    cout << "Range [" << SearchRange.size() << "] : ";
    for(int j = 0; j < SearchRange.size(); j++){
      cout << SearchRange[j] << " ";
    }
    cout << endl;

#endif

  Pavg_sum.resize(SearchHeight.size());
  for (int ih = 0; ih < SearchHeight.size(); ih++) {
    Pavg_sum[ih].resize(SearchDirection.size());
    for (int id = 0; id < SearchDirection.size(); id++) {
      Pavg_sum[ih][id].resize(SearchRange.size());
    }
  }

  Pavg.resize(SearchHeight.size());
  for (int ih = 0; ih < SearchHeight.size(); ih++) {
    Pavg[ih].resize(SearchDirection.size());
    for (int id = 0; id < SearchDirection.size(); id++) {
      Pavg[ih][id].resize(SearchRange.size());
    }
  }

  Pavg_bin.resize(A[0][0][0].size1());
  for (int bins = 0; bins < A[0][0][0].size1(); bins++) {
    Pavg_bin[bins].resize(SearchHeight.size());
    for (int ih = 0; ih < SearchHeight.size(); ih++) {
      Pavg_bin[bins][ih].resize(SearchDirection.size());
      for (int id = 0; id < SearchDirection.size(); id++) {
	Pavg_bin[bins][ih][id].resize(SearchRange.size());
      }
    }
  }

  cerr << SearchHeight.size() << " heights, "
       << SearchDirection.size() << " directions, "
       << SearchRange.size() << " ranges, "
       << A[0][0][0].size2() << " microphones, "
       << (A[0][0][0].size1() - 1) * 2 << " points" << endl;

}

void LocalizationMUSIC::CalcAveragePower(vector<fvector>& w, vector<fcmatrix>& E,
					  AMatrixArray3D& A,
					  vector<bool>& UsesFreq,
					  int num_source, 
					  vector<vector<vector<float> > >& Pavg,
					  vector<vector<vector<vector<float> > > >& Pavg_bin,
					  ublas::vector<float>& Weight,
					  bool enable_eigenvalue_weight)
{
  vector<bool> UsesX(Pavg.size());
  for(int i = 0; i < Pavg.size(); i++) UsesX[i] = true;
  vector<bool> UsesY(Pavg[0].size());
  for(int i = 0; i < Pavg[0].size(); i++) UsesY[i] = true;
  vector<bool> UsesZ(Pavg[0][0].size());
  for(int i = 0; i < Pavg[0][0].size(); i++) UsesZ[i] = true;
  CalcAveragePower(w, E, A, UsesFreq, num_source, Pavg, Pavg_bin, Weight, enable_eigenvalue_weight, UsesX, UsesY, UsesZ);
}

void LocalizationMUSIC::CalcAveragePower(vector<fvector>& w, vector<fcmatrix>& E,
					  AMatrixArray3D& A,
					  vector<bool>& UsesFreq,
					  int num_source, 
					  vector<vector<vector<float> > >& Pavg,
					  vector<vector<vector<vector<float> > > >& Pavg_bin,
					  ublas::vector<float>& Weight,
					  bool enable_eigenvalue_weight,
					  vector<bool>& UsesNh,
					  vector<bool>& UsesNd,
					  vector<bool>& UsesNr)
{
  int nh = Pavg.size();
  int nd = Pavg[0].size();
  int nr = Pavg[0][0].size();
  int num_mic = w[0].size();
    
  for (int ih = 0; ih < nh; ih++) {
    for (int id = 0; id < nd; id++) {
      for (int ir = 0; ir < nr; ir++) {
	Pavg[ih][id][ir] = 0.0f;
      }
    }
  }

  for (int k = 0; k < Pavg_bin.size(); k++) {
    for (int ih = 0; ih < Pavg_bin[0].size(); ih++) {
      for (int id = 0; id < Pavg_bin[0][0].size(); id++) {
	for (int ir = 0; ir < Pavg_bin[0][0][0].size(); ir++) {
	  Pavg_bin[k][ih][id][ir] = 0.0f;
	}
      }
    }
  }
  
  float G = 0.0f;
  float norm_a = 0.0f;
  for (int k = 0; k < UsesFreq.size(); k++) {
    if (!UsesFreq[k]) {
      continue;
    }
        
    for (int ih = 0; ih < nh; ih++) {
      if (!UsesNh[ih]) continue; 
      for (int id = 0; id < nd; id++) {
	if (!UsesNd[id]) continue; 
	for (int ir = 0; ir < nr; ir++) {
	  if (!UsesNr[ir]) continue; 
	  G = 0.0f;
	  for (int m = num_mic - num_source - 1; m >= 0; m--) {
	    G += norm(ublas::inner_prod(ublas::herm(ublas::row(A[ih][id][ir], k)),
					ublas::column(E[k], m)));
	  }
	  norm_a = ublas::inner_prod(ublas::herm(ublas::row(A[ih][id][ir], k)),
				     ublas::row(A[ih][id][ir], k)).real();
	  float EV_Weight = (enable_eigenvalue_weight ? sqrt(w[k][num_mic - 1]) : 1.0 );
	  Pavg[ih][id][ir] += (norm_a / G) * Weight[k] * EV_Weight;
	  Pavg_bin[k][ih][id][ir] = 10 * log10((norm_a / G * Weight[k]) + 1.0f);
	  cerr << "";
	}
      }
    }
  }
}

int LocalizationMUSIC::SearchLocation(vector<vector<vector<float> > >& Pavg_sum, 
				       vector<vector<vector<float> > >& Pavg,
				       int n_source, 
				       vector<int>& height, 
				       vector<int>& direction, 
				       vector<int>& range, 
				       vector<int>& SDirection, int mindeg, int maxdeg)
{
  vector<float> fSDirection(SDirection.size());
  for(int i = 0; i < SDirection.size(); i++){
    fSDirection[i] = (float)SDirection[i];
  }
  return SearchLocation(Pavg_sum, Pavg, n_source,
			height, direction, range, 
			fSDirection, mindeg, maxdeg);
}

int LocalizationMUSIC::SearchLocation(vector<vector<vector<float> > >& Pavg_sum, 
				       vector<vector<vector<float> > >& Pavg,
				       int n_source, 
				       vector<int>& height, 
				       vector<int>& direction, 
				       vector<int>& range, 
				       vector<float>& SDirection, int mindeg, int maxdeg)
{
  int n_result;
  int nh = Pavg.size();
  int nd = Pavg[0].size();
  int nr = Pavg[0][0].size();
  vector<int>   candidate_id;
  vector<float> candidate_val;

  candidate_id.reserve(nh * nd * nr);
  candidate_id.resize(0);
  candidate_val.reserve(nh * nd * nr);
  candidate_val.resize(0);
  SearchPeak(Pavg_sum, candidate_id, candidate_val, SDirection, mindeg, maxdeg);

  vector<int> index;
  for(int i = 0; i < candidate_id.size(); i++) 
    index.push_back( i );
  
  sort( index.begin(), index.end(), bind<bool>(less_for_vec(), candidate_val, _2, _1) );

  vector<int> res_id;
  for(int i = 0; i < index.size(); i++) 
    res_id.push_back( candidate_id[index[i]] );

  vector<float> res_val;
  for(int i = 0; i < index.size(); i++) 
    res_val.push_back( candidate_val[index[i]] );

  if (res_id.size() < n_source) {
    n_result = res_id.size();
  } else {
    n_result = n_source;
  }

  height.resize(0);
  direction.resize(0);
  range.resize(0);  
  for (int i = 0; i < n_result; i++){
    int indextmp = res_id[i];
    int ihtmp = floor(indextmp / (nd * nr));
    int ihrem = indextmp - ihtmp * (nd * nr);
    int idtmp = floor(ihrem / nr);
    int irtmp = ihrem - idtmp * nr;
    height.push_back(ihtmp);
    direction.push_back(idtmp);
    range.push_back(irtmp);
  }

  return n_result;
}

void LocalizationMUSIC::SearchPeak(vector<vector<vector<float> > >& power, 
				    vector<int>& peak_cand_id, 
				    vector<float>& peak_cand_val, 
				    vector<float>& SDirection, int mindeg, int maxdeg)
{
  vector<vector<vector<float> > > pt;
  pt.resize(3);
  for(int ptx = 0; ptx < pt.size(); ptx++){
    pt[ptx].resize(3);
    for(int pty = 0; pty < pt[ptx].size(); pty++){
      pt[ptx][pty].resize(3);
    }
  }

  while( maxdeg - mindeg > 360 ) maxdeg -= 360; 
  while( maxdeg - mindeg <= 0 ) maxdeg += 360; 

  int nh = power.size();
  int nd = power[0].size();
  int nr = power[0][0].size();

  bool h_flg = (nh < 2 ? false : true);
  bool d_flg = (nd < 2 ? false : true);
  bool r_flg = (nr < 2 ? false : true);
  
  if ( !h_flg && !d_flg && !r_flg ) {
    peak_cand_id.push_back(0);
    peak_cand_val.push_back(0);
  } else {

    for (int ih = 0; ih < nh; ih++) {

      for (int id = 0; id < nd; id++) {

	int deg = (int)(SDirection[id] + 0.5); 
	while( deg - mindeg > 360 ) deg -= 360; 
	while( deg - mindeg <= 0 )  deg += 360; 
	
	if ( !(deg > mindeg) && !(deg < maxdeg) ) continue;

	for (int ir = 0; ir < nr; ir++) {

	  for (int jh = -1; jh <= 1; jh++) {
	    for (int jd = -1; jd <= 1; jd++) {
	      for (int jr = -1; jr <= 1; jr++) {
		bool exp_flg = false;
		bool h_exp_flg = (((ih+jh < 0)||(ih+jh > nh-1)) ? true : false);
		bool r_exp_flg = (((ir+jr < 0)||(ir+jr > nr-1)) ? true : false);		
		if(id+jd < 0){pt[jh+1][jd+1][jr+1]    = (((h_exp_flg)||(r_exp_flg)) ? -1.0f : power[ih+jh][nd-1][ir+jr] ); exp_flg = true;}
		if(id+jd > nd-1){pt[jh+1][jd+1][jr+1] = (((h_exp_flg)||(r_exp_flg)) ? -1.0f : power[ih+jh][0][ir+jr] );    exp_flg = true;}
		if(h_exp_flg){pt[jh+1][jd+1][jr+1] = -1.0f; exp_flg = true;}
		if(r_exp_flg){pt[jh+1][jd+1][jr+1] = -1.0f; exp_flg = true;}
		if(exp_flg) continue;
		pt[jh+1][jd+1][jr+1] = power[ih+jh][id+jd][ir+jr];
	      }
	    }
	  }
	
	  bool pf = true;
	  for (int jh = 0; jh < 3; jh++) {
	    for (int jd = 0; jd < 3; jd++) {
	      for (int jr = 0; jr < 3; jr++) {
		if (jh == 1 && jd == 1 && jr == 1) continue;
		if(pt[jh][jd][jr] >= pt[1][1][1]) pf = false;
	      }
	    }
	  }
	  if (pf) {
	    peak_cand_id.push_back(ir + id * nr + ih * nd * nr);  
	    peak_cand_val.push_back( power[ih][id][ir] );  
	  }
	  
	}
      }
    }    
    
  }
}

#endif
