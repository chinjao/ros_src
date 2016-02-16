#override datadir=$(FLOWDESIGNER_DATA)
override datadir=${prefix}/lib/flowdesigner/toolbox/hark
#override libdir=$(FLOWDESIGNER_DATA)
override libdir=${prefix}/lib/flowdesigner/toolbox/hark

lib_LTLIBRARIES = libhark.la 

install-data-local:
	mkdir -p ${DESTDIR}$(datadir)
	(if [ -f ${DESTDIR}$(libdir)/lib$(PACKAGE) ]; then cd ${DESTDIR}$(libdir); rm -f $(PACKAGE).tlb; ln -s lib$(PACKAGE) $(PACKAGE).tlb; else cd ${DESTDIR}$(libdir); rm -f $(PACKAGE).tlb; ln -s lib$(PACKAGE).so $(PACKAGE).tlb; fi)
	(info2def.pl $(libhark_la_SOURCES) > ${DESTDIR}$(datadir)/hark.def)

libhark_la_SOURCES = \
	VERSION.cc \
	MatrixToMap.cc \
	AudioStreamFromMic.cc \
	AudioStreamFromWave.cc \
	ChannelSelector.cc \
	MultiFFT.cc \
	ConstantLocalization.cc \
	SaveSourceLocation.cc LoadSourceLocation.cc \
	LocalizationFunctionMUSIC.cpp \
	LocalizationFunction.cpp \
	LocalizeMUSIC.cc LocalizationMUSIC.cpp \
	CMChannelSelector.cc \
	CMIdentityMatrix.cc \
	CMLoad.cc \
	CMMakerFromFFTwithFlag.cc \
	CMMultiplyMatrix.cc \
	CMDivideEachElement.cc \
	CMInverseMatrix.cc \
	CMMakerFromFFT.cc \
	CMMultiplyEachElement.cc \
	CMSave.cc \
	SourceTracker.cc \
	CombineSource.cc \
	DisplayLocalization.cc \
	MelFilterBank.cc \
	MultiDownSampler.cc \
	MSLSExtraction.cc \
	MFCCExtraction.cc \
	Delta.cc \
	SpectralMeanNormalization.cc \
	MFMGeneration.cc \
	DeltaMask.cc \
	SaveFeatures.cc \
	SaveHTKFeatures.cc \
	SaveRawPCM.cc \
	SaveWavePCM.cc \
	Synthesize.cc \
	WhiteNoiseAdder.cc \
	SpeechRecognitionClient.cc 	SpeechRecognitionSMNClient.cc \
	HarkDataStreamSender.cc \
	SourceSelectorByDirection.cc \
	FeatureRemover.cc \
	SourceIntervalExtender.cc \
	PF_est_leak.c PF_est_noise.c PF_est_rev.c PF_est_sn.c \
	PF_est_state_noise.c PF_est_voice_p.c PF_est_zeta.c PF_execV2.c \
	PF_hypergeom_gain.c PF_Init.c \
	PF_init_state_noise.c \
	CPF_exec.cpp \
	CPF_init_state_noise.cpp \
	PowerCalcForMap.cc \
	PowerCalcForMatrix.cc \
	BGNEstimator.cc \
	PostFilter.cc \
	SourceSelectorByID.cc \
	DeltaPowerMask.cc \
	PreEmphasis.cc \
	MultiGain.cc \
	GHDSS_SeparateMat.cpp \
	CGHDSS_exec.cpp \
	Cexec.cpp \
	GHDSS.cc\
	HarkFileIO.cpp \
	DataLogger.cc \
	HRLE.cc \
	SpectralGainFilter.cc \
	CalcSpecAddPower.cc \
	CalcSpecSubGain.cc \
	EstimateLeak.cc \
	SegmentAudioStreamByID.cc


pkginclude_HEADERS= \
	../include/CGHDSS_exec.h \
	../include/HarkData.h \
	../include/PF_est_noise.h \
	../include/PF_hypergeom_gain.h \
	../include/endian.hpp \
	../include/CPF_exec.h \
	../include/HarkFileIO.h \
	../include/PF_est_rev.h \
	../include/PF_init_state_noise.h \
	../include/evd.hpp \
	../include/CPF_init_state_noise.h \
	../include/HarkParamsObjects.h \
	../include/PF_est_sn.h \
	../include/PF_values.h \
	../include/fft.hpp \
	../include/Cexec.h \
	../include/LocalizationMUSIC.hpp \
	../include/PF_est_state_noise.h \
	../include/Source.h \
	../include/htkheader.hpp \
	../include/GHDSS_MathMat2.h \
	../include/Map.h \
	../include/PF_est_voice_p.h \
	../include/SpectralMean.hpp \
	../include/melfilterbank.hpp \
	../include/GHDSS_MathMatDouble.h \
	../include/PF_Init.h \
	../include/PF_est_zeta.h \
	../include/Time.h \
	../include/mfcnet.h \
	../include/GHDSS_SeparateMat.h \
	../include/PF_est_leak.h \
	../include/PF_exec.h \
	../include/TimeStamp.h \
	../include/wave.h



libhark_la_CXXFLAGS = @GTK_CFLAGS@

libhark_la_LDFLAGS = -release $(LT_RELEASE) \
	`flowdesigner-config --libs config libflow audio_blocks` \
	`pkg-config libharkio --libs` \
	-L../librecorder -lrecorder @GTK_LIBS@ \
	-L../libseparation -lseparation @GTK_LIBS@ \
	-lboost_thread-mt -lasound


INCLUDES = `flowdesigner-config --cflags config libflow audio_blocks` \
	-I ../include \
	`pkg-config libharkio --cflags` \
	-I ../librecorder/boost-sandbox -I ../librecorder \
	-I ../libseparation

if ENABLE_HARKIO1
libhark_la_LDFLAGS += -lharkio
endif
if ENABLE_HARKIO2
libhark_la_LDFLAGS += -lharkio2 -lxml++-2.6 -lxml2 -lglibmm-2.4 -lgobject-2.0 -lsigc-2.0 -lgthread-2.0 -lrt -lglib-2.0 -pthread
endif
if ENABLE_ASRASP
libhark_la_LDFLAGS += -lrasp-netapi
endif
if ENABLE_WS
libhark_la_LDFLAGS += -lrasp-netapi
endif
if ENABLE_RASP24
libhark_la_LDFLAGS += -lrasp-netapi
endif
if ENABLE_HARKNETAPI
libhark_la_LDFLAGS += -lhark-netapi
endif
if ENABLE_TDBD8CSUSB
libhark_la_LDFLAGS += -lasio
endif
if ENABLE_TDBD16ADUSB
libhark_la_LDFLAGS += -lsinich
endif
if ENABLE_RASPLC
libhark_la_LDFLAGS += -lrasp-netapi
endif




