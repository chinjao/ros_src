#!/bin/bash
# hark-ros に対して debuildを行う
# 本来のdebuildは、環境変数を削除してしまうので
# このスクリプトにより、hark-rosに必要な環境変数を付加している
# configureやmakefileの中で、明示的に環境変数を設定しても、debuildのサブプロセスである限り無効である。
# ROS_PACKAGE_PATHは、環境により適宜設定を行うする必要がある。


if [ $# -eq 0 ]; then

	echo "not argument" 
	echo "groovy or fuerte or cturtle"


elif [ $1 = "groovy" ]; then

ROS_ROOT=/opt/ros/groovy/share/ros
PATH=${ROS_ROOT}/bin:${PATH}
PYTHONPATH=${ROS_ROOT}/core/roslib/src:${PYTHONPATH}
ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:/opt/ros/groovy/stacks
LD_LIBRARY_PATH=/opt/ros/groovy/lib

# --preserve-envvar PAHT \
debuild \
 -e ROS_ROOT=${ROS_ROOT}\
 -e PATH=${ROS_ROOT}/bin:${PATH} \
 -e PYTHONPATH=${ROS_ROOT}/core/roslib/src:${PYTHONPATH} \
 -e ROS_PACKAGE_PATH=/opt/ros/groovy/share:/opt/ros/groovy/stacks:${HOME}/ros/hark/hark_common_msgs/hark_msgs:${HOME}/ros/hark/hark_common_msgs/hark_params \
 -e LD_LIBRARY_PATH=/opt/ros/groovy/lib


elif [ $1 = "fuerte" ]; then

ROS_ROOT=/opt/ros/fuerte/share/ros
PATH=${ROS_ROOT}/bin:${PATH}
PYTHONPATH=${ROS_ROOT}/core/roslib/src:${PYTHONPATH}
ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:/opt/ros/fuerte/stacks
LD_LIBRARY_PATH=/opt/ros/fuerte/lib

# --preserve-envvar PAHT \
debuild \
 -e ROS_ROOT=${ROS_ROOT}\
 -e PATH=${ROS_ROOT}/bin:${PATH} \
 -e PYTHONPATH=${ROS_ROOT}/core/roslib/src:${PYTHONPATH} \
 -e ROS_PACKAGE_PATH=/opt/ros/fuerte/share:/opt/ros/fuerte/stacks:${HOME}/ros/hark/hark_common_msgs/hark_msgs:${HOME}/ros/hark/hark_common_msgs/hark_params \
 -e LD_LIBRARY_PATH=/opt/ros/fuerte/lib



else


	echo "hazure"

fi 









#ROS_ROOT=/opt/ros/cturtle/ros
#PATH=${ROS_ROOT}/bin:${PATH}
#PYTHONPATH=${ROS_ROOT}/core/roslib/src:${PYTHONPATH}
#ROS_PACKAGE_PATH=/opt/ros/cturtle/stacks
#
# --preserve-envvar PAHT \
#debuild \
# -e ROS_ROOT=${ROS_ROOT}\
# -e PATH=${ROS_ROOT}/bin:${PATH} \
# -e PYTHONPATH=${ROS_ROOT}/core/roslib/src:${PYTHONPATH} \
# -e ROS_PACKAGE_PATH=/opt/ros/cturtle/stacks:`pwd`/../../ROS/ros_stack_hri/hark_common_msgs/hark_msgs:`pwd`/../../ROS/#ros_stack_hri/hark_common_msgs/hark_params
 
 
 
 
