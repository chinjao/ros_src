# include $(shell rospack find mk)/cmake_stack.mk

MAKE =make
STACK=hark_common_msgs
SUBDIRS=hark_msgs hark_params
SUBSUBDIRS=build 
TARGET=$(DESTDIR)/opt/ros/cturtle/stacks/hark_ros_stacks

all :	
	set -e; for i in $(SUBDIRS); do cd $$i; $(MAKE); cd ../; done

install :

	set -e; for i in $(SUBDIRS); do cd $$i; for j in $(SUBSUBDIRS); do rm -rf $$j; done; cd ../; done
	mkdir -p ${TARGET}/$(STACK)
	find ./ -maxdepth 1 ! -name "." -and ! -name debian -exec cp -fpR {} ${TARGET}/$(STACK) \;
	rm -rf ${TARGET}/${STACK}/Makefile
	mv ${TARGET}/${STACK}/Makefile.stack ${TARGET}/${STACK}/Makefile

clean :	
	set -e; for i in $(SUBDIRS); do cd $$i; $(MAKE) clean; cd ../; done

