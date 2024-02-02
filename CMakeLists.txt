cmake_minimum_required(VERSION 2.8.3)
project(radar_slam)


## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)
set(CMAKE_BUILD_TYPE "Release") # Release mode for speed

# For Ceres
# Set the C++ version (must be >= C++11) when compiling Ceres.
#
# Reflect a user-specified (via -D) CMAKE_CXX_STANDARD if present, otherwise
# default to C++11.
set(DEFAULT_CXX_STANDARD ${CMAKE_CXX_STANDARD})
if (NOT DEFAULT_CXX_STANDARD)
  set(DEFAULT_CXX_STANDARD 11)
endif()

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  cv_bridge
  pcl_ros
  pcl_conversions
  std_msgs
  sensor_msgs
  nav_msgs
  geometry_msgs
  tf
  visualization_msgs
  message_filters
  image_transport
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
catkin_python_setup()

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
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES vio_text
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)



## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/vio_text_node.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
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
# install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_vio_text.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

# Libosmium
#message(WARNING "CMAKE_SOURCE_DIR is ${CMAKE_SOURCE_DIR}\n")
#list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/radar_slam/cmake")
#message(WARNING "CMAKE_MODULE_PATH is ${CMAKE_MODULE_PATH}\n")
#find_package(Osmium REQUIRED)

#if(NOT OSMIUM_FOUND)
#    message(WARNING "Libosmium not found!\n")
#endif()

#include_directories(${OSMIUM_INCLUDE_DIRS})
#message(WARNING "OSMIUM_INCLUDE_DIRS is ${OSMIUM_INCLUDE_DIRS}\n")

## CFAR from RmSAT-CFAR
#find_package(Cfar REQUIRED)
#if(NOT CFAR_FOUND)
#    message(WARNING "Libcfar not found!\n")
#endif()
#include_directories(${CFAR_INCLUDE_DIR})
#message(WARNING "CFAR_INCLUDE_DIR is ${CFAR_INCLUDE_DIR}\n")


# G2O
LIST( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules )
SET( G2O_ROOT /usr/local/include/g2o )
FIND_PACKAGE( G2O )
#message(STATUS "G2O version: ${G2O_VERSION}" )

# CSparse
FIND_PACKAGE( CSparse )
INCLUDE_DIRECTORIES( ${G2O_INCLUDE_DIR} ${CSPARSE_INCLUDE_DIR} )
#message(STATUS "CSparse version: ${CSparse_VERSION}" )




# Eigen
find_package(Eigen3 REQUIRED)
#message(WARNING "EIGEN3_INCLUDE_DIR is ${EIGEN3_INCLUDE_DIR}\n")
IF( NOT EIGEN3_INCLUDE_DIR )
    MESSAGE( FATAL_ERROR "Please point the environment variable EIGEN3_INCLUDE_DIR to the include directory of your Eigen3 installation.")
ENDIF()
INCLUDE_DIRECTORIES ( "${EIGEN3_INCLUDE_DIR}" )
message("Eigen path: ${EIGEN3_INCLUDE_DIR}" )

## Library: CAIRO
#FIND_PACKAGE(Cairo)
#IF(CAIRO_FOUND)
#        MESSAGE(WARNING "Cairo found")
#        #IF($ENV{VERBOSE})
#                MESSAGE(STATUS "   CAIRO_LIBRARIES = ${CAIRO_LIBRARIES}")
#                MESSAGE(STATUS "   CAIRO_INCLUDE_DIR = ${CAIRO_INCLUDE_DIR}")
#        #ENDIF($ENV{VERBOSE})
#        INCLUDE_DIRECTORIES(${CAIRO_INCLUDE_DIR})
#ELSE(CAIRO_FOUND)
#        MESSAGE(WARNING "        Cairo not found, it is OPTIONAL: to draw to PDF files.")
#ENDIF(CAIRO_FOUND)

# Flirt
#INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/Thirdparty/openslam_flirtlib/src) 

# M2DP
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/Thirdparty/codegen/lib/generateGlobalFeature) 

# ANMS
SET(ANMS_SOURCE ${PROJECT_SOURCE_DIR}/Thirdparty/ANMS-Codes/CmakeProject/source)
INCLUDE_DIRECTORIES(${ANMS_SOURCE}) 

add_executable(radar_slam 
src/main.cpp
src/FullSystem/System.cpp
src/FullSystem/System.h
src/FullSystem/Viewer.cpp
src/FullSystem/Viewer.h
src/FullSystem/Tracking.cpp
src/FullSystem/Tracking.h
src/FullSystem/FeatureTracker.cpp
src/FullSystem/FeatureTracker.h
src/FullSystem/Feature.cpp
src/FullSystem/Feature.h
src/FullSystem/LocalMapping.cpp
src/FullSystem/LocalMapping.h
src/FullSystem/Matcher.cpp
src/FullSystem/Matcher.h
src/FullSystem/Converter.cpp
src/FullSystem/Converter.h
src/FullSystem/Frame.cpp
src/FullSystem/Frame.h
src/FullSystem/Algorithms.cpp
src/FullSystem/Algorithms.h
src/FullSystem/cfar.cpp
src/FullSystem/cfar.h
src/FullSystem/KDTree.cpp
src/FullSystem/KDTree.h
src/FullSystem/KeyFrame.cpp
src/FullSystem/KeyFrame.h
src/FullSystem/MapPoint.cpp
src/FullSystem/MapPoint.h
src/FullSystem/Map.cpp
src/FullSystem/Map.h
src/FullSystem/EdgeSE2XYOnlyPose.cpp
src/FullSystem/EdgeSE2XYOnlyPose.h
src/FullSystem/EdgeSelfDefined.cpp
src/FullSystem/EdgeSelfDefined.h
src/FullSystem/Optimizer.cpp
src/FullSystem/Optimizer.h
src/FullSystem/LoopClosing.cpp
src/FullSystem/LoopClosing.h
# Graph clique
src/FullSystem/mcqd.h
# ANMS
${ANMS_SOURCE}/nanoflann.hpp
${ANMS_SOURCE}/range-tree/lrtypes.h
${ANMS_SOURCE}/range-tree/ranget.h
${ANMS_SOURCE}/anms.h
# Utilities
src/util/mercator.cpp
src/util/mercator.h
src/util/Loader.cpp
src/util/Loader.h
src/util/polarToCartesian.cpp
src/util/polarToCartesian.h
)

# OpenMP
find_package(OpenMP)
if(OpenMP_CXX_FOUND)
    target_link_libraries(radar_slam PUBLIC OpenMP::OpenMP_CXX)
endif()

# OpenCV
find_package(OpenCV)
#message(STATUS "OpenCV libs: ${OpenCV_LIBRARIES}" )
message(STATUS "OpenCV version: ${OpenCV_VERSION}" )

# Link libraries
target_link_libraries(radar_slam ${OpenCV_LIBRARIES} ${catkin_LIBRARIES}  g2o_core g2o_types_slam2d g2o_types_slam3d g2o_solver_csparse g2o_solver_eigen g2o_stuff g2o_csparse_extension ${CSPARSE_LIBRARY} 
#${CAIRO_LIBRARIES}
${PROJECT_SOURCE_DIR}/Thirdparty/codegen/lib/build/generateGlobalFeature/libglobalFeature.so
#${OSMIUM_LIBRARIES}
# FLIRT lib, not used
#${PROJECT_SOURCE_DIR}/Thirdparty/openslam_flirtlib/build/src/geometry/libgeometry.so
#${PROJECT_SOURCE_DIR}/Thirdparty/openslam_flirtlib/build/src/feature/libfeature.so
#${PROJECT_SOURCE_DIR}/Thirdparty/openslam_flirtlib/build/src/sensorstream/libsensorstream.so
#${PROJECT_SOURCE_DIR}/Thirdparty/openslam_flirtlib/build/src/sensors/libsensors.so
#${PROJECT_SOURCE_DIR}/Thirdparty/openslam_flirtlib/build/src/utils/libutils.so

)


