cmake_minimum_required(VERSION 2.8.3)
project(zmq_ros)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  visualization_msgs
  sensor_msgs
  message_generation
  message_filters
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependencies might have been
##     pulled in transitively but can be declared for certainty nonetheless:
##     * add a build_depend tag for "message_generation"
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

 add_message_files(
   FILES
   zmq_deb.msg
   zmq_time.msg
   mecab_amivoice.msg
   amivoice_noun_verb.msg
   zmq_deb2.msg
   zmq_time2.msg
   mecab_amivoice2.msg
   amivoice_noun_verb2.msg
#   Message2.msg
 )
## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs
# )
 generate_messages(
   DEPENDENCIES
   std_msgs 
   visualization_msgs
   sensor_msgs
 )
###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES zmq_ros
#  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)



## Declare a cpp library
# add_library(zmq_ros
#   src/${PROJECT_NAME}/zmq_ros.cpp
# )

## Declare a cpp executable
# add_executable(zmq_ros_node src/zmq_ros_node.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(zmq_ros_node zmq_ros_generate_messages_cpp)

## Specify libraries to link a library or executable target against
# target_link_libraries(zmq_ros_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS zmq_ros zmq_ros_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )
find_library(ZMQ_LIBRARIE
NAMES libzmq.so
HITNS /usr/local/lib/
PATH_SUFFIXES lib
)
## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_zmq_ros.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
add_executable(zmq_sub src/zmq_sub.cpp)
target_link_libraries(zmq_sub ${catkin_LIBRARIES} ${ZMQ_LIBRARIE} msgpack)

add_executable(zmq_pub src/zmq_pub.cpp)
target_link_libraries(zmq_pub ${catkin_LIBRARIES} ${ZMQ_LIBRARIE} msgpack)

add_executable(zmq_sub_name src/zmq_sub2.cpp)
target_link_libraries(zmq_sub_name ${catkin_LIBRARIES} ${ZMQ_LIBRARIE} msgpack)

add_executable(zmq_sub_full src/zmq_sub3.cpp)
target_link_libraries(zmq_sub_full ${catkin_LIBRARIES} ${ZMQ_LIBRARIE} msgpack)

add_executable(zmq_sub_name_cpy src/zmq_sub_cpy.cpp)
target_link_libraries(zmq_sub_name_cpy ${catkin_LIBRARIES} ${ZMQ_LIBRARIE} msgpack)

add_executable(zmq_ros_subscribe src/zmq_ros_subscribe.cpp)
target_link_libraries(zmq_ros_subscribe ${catkin_LIBRARIES} ${ZMQ_LIBRARIE} msgpack)

add_executable(zmq_sub_marker src/zmq_sub_marker.cpp)
target_link_libraries(zmq_sub_marker ${catkin_LIBRARIES} ${ZMQ_LIBRARIE} msgpack)

add_executable(zmq_sub_new src/zmq_sub4.cpp)
target_link_libraries(zmq_sub_new ${catkin_LIBRARIES} ${ZMQ_LIBRARIE} msgpack)

add_executable(speech_recog src/speech_recog.cpp)
target_link_libraries(speech_recog ${catkin_LIBRARIES})

add_executable(speech_recog_conv src/speech_recog_conv.cpp)
target_link_libraries(speech_recog_conv ${catkin_LIBRARIES} ${ZMQ_LIBRARIE} msgpack)

add_executable(speech_recog4 src/speech_recog4.cpp)
target_link_libraries(speech_recog4 ${catkin_LIBRARIES} ${ZMQ_LIBRARIE} msgpack)

add_executable(speech_recog_client src/speech_recog_client.cpp)
target_link_libraries(speech_recog_client ${catkin_LIBRARIES})

#add_executable(speech_recog_baxter src/speech_recog_baxter3.cpp)
#target_link_libraries(speech_recog_baxter ${catkin_LIBRARIES} ${ZMQ_LIBRARIE} msgpack)


add_executable(speech_recog_baxter2 src/speech_recog/main.cpp
				 src/speech_recog/baxter.cpp
				 src/speech_recog/robot.cpp
				 src/speech_recog/move.cpp
				 src/speech_recog/command.cpp
				 src/speech_recog/extract.cpp)
target_link_libraries(speech_recog_baxter2 ${catkin_LIBRARIES} ${ZMQ_LIBRARIE} msgpack)

add_executable(emovoice_sub src/emovoice_sub.cpp)
target_link_libraries(emovoice_sub ${catkin_LIBRARIES} ${ZMQ_LIBRARIE} msgpack)

add_executable(emovoice_browser src/emovoice_browser_mpr/main.cpp
				 src/emovoice_browser_mpr/baxter.cpp
				 src/emovoice_browser_mpr/robot.cpp
				 src/emovoice_browser_mpr/move.cpp
				 src/emovoice_browser_mpr/command.cpp
				 src/emovoice_browser_mpr/extract.cpp)
target_link_libraries(emovoice_browser ${catkin_LIBRARIES} ${ZMQ_LIBRARIE} msgpack)

add_executable(speech_recog_word src/speech_recog_10word/main.cpp
				 src/speech_recog_10word/baxter.cpp
				 src/speech_recog_10word/robot.cpp
				 src/speech_recog_10word/move.cpp
				 src/speech_recog_10word/command.cpp
				 src/speech_recog_10word/extract.cpp)
target_link_libraries(speech_recog_word ${catkin_LIBRARIES} ${ZMQ_LIBRARIE} msgpack)

add_executable(experiment_browser src/experiment_browser/main.cpp
				 src/experiment_browser/baxter.cpp
				 src/experiment_browser/robot.cpp
				 src/experiment_browser/move.cpp
				 src/experiment_browser/command.cpp
				 src/experiment_browser/extract.cpp)
target_link_libraries(experiment_browser ${catkin_LIBRARIES} ${ZMQ_LIBRARIE} msgpack)

add_executable(deep_browser src/deep_browser/main.cpp
				 src/deep_browser/baxter.cpp
				 src/deep_browser/robot.cpp
				 src/deep_browser/move.cpp
				 src/deep_browser/command.cpp
				 src/deep_browser/extract.cpp)
target_link_libraries(deep_browser ${catkin_LIBRARIES} ${ZMQ_LIBRARIE} msgpack)
