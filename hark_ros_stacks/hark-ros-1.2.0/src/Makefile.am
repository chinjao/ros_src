#override datadir=$(FLOWDESIGNER_DATA)
override datadir=${prefix}/lib/flowdesigner/toolbox/hark-ros
#override libdir=$(FLOWDESIGNER_DATA)
override libdir=${prefix}/lib/flowdesigner/toolbox/hark-ros

lib_LTLIBRARIES = libhark-ros.la 

install-data-local:
	mkdir -p $(DESTDIR)$(datadir)
	(if [ -f $(DESTDIR)$(libdir)/lib$(PACKAGE) ]; then cd $(DESTDIR)$(libdir); rm -f $(PACKAGE).tlb; ln -s lib$(PACKAGE) $(PACKAGE).tlb; else cd $(DESTDIR)$(libdir); rm -f $(PACKAGE).tlb; ln -s lib$(PACKAGE).so $(PACKAGE).tlb; fi)
	(info2def.pl $(libhark_ros_la_SOURCES) > $(DESTDIR)$(datadir)/hark-ros.def)

libhark_ros_la_SOURCES = \
	VERSION.cc \
	AudioStreamFromRos.cc \
	BeatCandidate.cc \
	BeatIOIPublisher.cc \
	IfRosStdMsgsPublisher.cc \
	HarkMsgsStreamFromRos.cc \
	StdMsgsStreamFromRos.cc \
	RosAddHarkIntClient.cc \
	RosAddHarkIntServer.cc \
	RosHarkIntDynReconf.cc \
	RosHarkIntPublisher.cc \
	RosHarkIntSubscriber.cc \
	RosHarkMsgsPublisher.cc \
	RosHarkMsgsSubscriber.cc \
	RosHarkParamsDynReconf.cc \
	RosHarkParamsSubscriber.cc \
	RosStdMsgsPublisher.cc \
	RosStdMsgsSubscriber.cc \
	RosNodeGenerator.cc \
	STPMPublisher.cc

libhark_ros_la_CXXFLAGS = -O3 -march=core2 -mtune=core2 -mmmx -msse -msse2 -msse3 -mfpmath=sse @GTK_CFLAGS@

libhark_ros_la_LDFLAGS = -release $(LT_RELEASE) \
	`flowdesigner-config --libs config libflow audio_blocks` \
	`pkg-config libharkio --libs`

INCLUDES = `flowdesigner-config --cflags config libflow audio_blocks` \
	-I ../include

libhark_ros_la_LIBADD = -lboost_thread-mt
