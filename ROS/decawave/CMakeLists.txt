cmake_minimum_required(VERSION 2.8.3)
project(decawave)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  serial
  std_msgs
  geometry_msgs
  roslib
  sensor_msgs
  message_generation
  message_filters
##  mavros
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
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

## Generate messages in the 'msg' folder
add_message_files(
  FILES
#  PosVec.msg
  trek1000_report.msg
  pdoa_report.msg
  pdoa_vicon.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

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
  INCLUDE_DIRS include
#  LIBRARIES cyphyhouse
  CATKIN_DEPENDS roscpp serial std_msgs geometry_msgs message_runtime message_filters
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
 include
  ${catkin_INCLUDE_DIRS}
  /usr/include/eigen3/
)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(decaSerial src/decaSerial.cpp)
add_executable(decaLocate src/decaLocate.cpp src/trilateration.cpp)
add_executable(decaOrient src/decaOrient.cpp)
add_executable(filter_UWBpoint_vicon src/UWBpoint_vicon_approxTimeFilter.cpp)
add_executable(pdoaSerial src/pdoa/pdoaSerial.cpp)
add_executable(pdoaVICON src/pdoa/pdoa_vicon_approxTimeFilter.cpp)
add_executable(pdoaLocator src/pdoa/pdoaLocator.cpp)
add_executable(pdoaLocateVICON src/pdoa/pdoaLocate_vicon_timeFilter.cpp)
add_executable(filter_UWBpoint_SX10 src/UWBpoint_SX10_approxTimeFilter.cpp)
add_executable(filter_UWBpose_SX10 src/UWBpose_SX10_approxTimeFilter.cpp)
add_executable(filter_UWBpose_vicon src/UWBpose_vicon_approxTimeFilter.cpp)
#add_executable(decaNode src/decaNode_PDoA.cpp src/decaNode_TDoA.cpp)
#add_executable(tdoa_node src/saveTDOA.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(decaSerial ${catkin_EXPORTED_TARGETS})
add_dependencies(decaLocate ${catkin_EXPORTED_TARGETS})
add_dependencies(decaOrient ${catkin_EXPORTED_TARGETS})
add_dependencies(filter_UWBpoint_vicon ${catkin_EXPORTED_TARGETS})
add_dependencies(pdoaSerial ${catkin_EXPORTED_TARGETS})
add_dependencies(pdoaVICON ${catkin_EXPORTED_TARGETS})
add_dependencies(pdoaLocator ${catkin_EXPORTED_TARGETS})
add_dependencies(pdoaLocateVICON ${catkin_EXPORTED_TARGETS})
add_dependencies(filter_UWBpoint_SX10 ${catkin_EXPORTED_TARGETS})
add_dependencies(filter_UWBpose_SX10 ${catkin_EXPORTED_TARGETS})
add_dependencies(filter_UWBpose_vicon ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(decaSerial ${catkin_LIBRARIES})
target_link_libraries(decaLocate ${catkin_LIBRARIES})
target_link_libraries(decaOrient ${catkin_LIBRARIES})
target_link_libraries(filter_UWBpoint_vicon ${catkin_LIBRARIES})
target_link_libraries(pdoaSerial ${catkin_LIBRARIES})
target_link_libraries(pdoaVICON ${catkin_LIBRARIES})
target_link_libraries(pdoaLocator ${catkin_LIBRARIES})
target_link_libraries(pdoaLocateVICON ${catkin_LIBRARIES})
target_link_libraries(filter_UWBpoint_SX10 ${catkin_LIBRARIES})
target_link_libraries(filter_UWBpose_SX10 ${catkin_LIBRARIES})
target_link_libraries(filter_UWBpose_vicon ${catkin_LIBRARIES})

#target_link_libraries(tdoa_node ${catkin_LIBRARIES})

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
# install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
#install(DIRECTORY #include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
#)

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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_cyphyhouse.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
