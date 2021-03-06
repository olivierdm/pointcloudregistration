cmake_minimum_required(VERSION 2.8.3)
project(pointcloudregistration)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
set(CMAKE_BUILD_TYPE Debug)
find_package(catkin REQUIRED COMPONENTS
roscpp
rosconsole
sensor_msgs
std_msgs
cmake_modules
pcl_ros
image_transport
message_filters
tf_conversions
cv_bridge)
## find_package(OpenGL REQUIRED)
## find_package(QGLViewer REQUIRED)
find_package(PCL 1.7 REQUIRED)
find_package(Eigen REQUIRED)
find_package(OpenCV REQUIRED) 
find_package(TBB REQUIRED)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS thread system)


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
#   std_msgs  # Or other packages containing msgs
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
 # LIBRARIES vision
  CATKIN_DEPENDS rosconsole sensor_msgs pcl_ros image_transport
#  DEPENDS OpenCV
)
# not compatible with pcl
set(CMAKE_CXX_FLAGS
  "${CMAKE_CXX_FLAGS} -march=native -Wall -std=c++0x"
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
 include_directories(SYSTEM include ${catkin_INCLUDE_DIRS}   ${PCL_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${TBB_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS} ${TBB_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

set(HEADER_DIR include/pointcloudregistration)
set(HEADER_FILES ${HEADER_DIR}/KeyFrame.h ${HEADER_DIR}/KeyFrameGraph.h  ${HEADER_DIR}/pcl_registration.h ${HEADER_DIR}/settings.h ${HEADER_DIR}/vision.h ${HEADER_DIR}/pcl_analyser.h ${HEADER_DIR}/datastructures.h ${HEADER_DIR}/linereg.h)

## Declare a cpp library
# add_library(vision
#   src/${PROJECT_NAME}/vision.cpp
# )

## Declare a cpp executable
 add_executable(registrar src/main_registration.cpp src/pcl_registration.cpp src/KeyFrame.cpp src/KeyFrameGraph.cpp src/settings.cpp   src/vision.cpp src/pcl_analyser.cpp src/linereg.cpp ${HEADER_FILES})
## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(vision_node vision_generate_messages_cpp)

## Specify libraries to link a library or executable target against
 target_link_libraries(registrar ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS} ${TBB_LIBRARIES} )
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
 install(TARGETS registrar
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

## Mark cpp header files for installation
 install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
 )

install(DIRECTORY launch/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
    PATTERN ".git" EXCLUDE
)

install(FILES
	src/stair_cascade.xml
	DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_vision.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
