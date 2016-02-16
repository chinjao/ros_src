#!/bin/bash
# hark-ros に対して debuildを行う
# 本来のdebuildは、環境変数を削除してしまうので
# このスクリプトにより、hark-rosに必要な環境変数を付加している
# configureやmakefileの中で、明示的に環境変数を設定しても、debuildのサブプロセスである限り無効である。
# ROS_PACKAGE_PATHは、環境により適宜設定を行うする必要がある。

ROS_ROOT=/opt/ros/cturtle/ros
PATH=${ROS_ROOT}/bin:${PATH}
PYTHONPATH=${ROS_ROOT}/core/roslib/src:${PYTHONPATH}
ROS_PACKAGE_PATH=/opt/ros/cturtle/stacks

# --preserve-envvar PAHT \
debuild \
 -e ROS_ROOT=${ROS_ROOT}\
 -e PATH=${ROS_ROOT}/bin:${PATH} \
 -e PYTHONPATH=${ROS_ROOT}/core/roslib/src:${PYTHONPATH} \
 -e ROS_PACKAGE_PATH=/opt/ros/cturtle/stacks:`pwd`/hark_msgs:`pwd`/hark_params
