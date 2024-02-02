# Install script for directory: /home/hong/catkin_ws/src/radar_localization/Thirdparty/openslam_flirtlib/src/feature

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "RelWithDebInfo")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/flirtlib/libfeature.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/flirtlib/libfeature.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/flirtlib/libfeature.so"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/flirtlib" TYPE SHARED_LIBRARY FILES "/home/hong/catkin_ws/src/radar_localization/Thirdparty/openslam_flirtlib/build/src/feature/libfeature.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/flirtlib/libfeature.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/flirtlib/libfeature.so")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/flirtlib/libfeature.so")
    endif()
  endif()
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/flirtlib/feature" TYPE FILE FILES
    "/home/hong/catkin_ws/src/radar_localization/Thirdparty/openslam_flirtlib/src/feature/Detector.h"
    "/home/hong/catkin_ws/src/radar_localization/Thirdparty/openslam_flirtlib/src/feature/Descriptor.h"
    "/home/hong/catkin_ws/src/radar_localization/Thirdparty/openslam_flirtlib/src/feature/InterestPoint.h"
    "/home/hong/catkin_ws/src/radar_localization/Thirdparty/openslam_flirtlib/src/feature/MultiScaleDetector.h"
    "/home/hong/catkin_ws/src/radar_localization/Thirdparty/openslam_flirtlib/src/feature/RangeDetector.h"
    "/home/hong/catkin_ws/src/radar_localization/Thirdparty/openslam_flirtlib/src/feature/NormalDetector.h"
    "/home/hong/catkin_ws/src/radar_localization/Thirdparty/openslam_flirtlib/src/feature/NormalEdgeDetector.h"
    "/home/hong/catkin_ws/src/radar_localization/Thirdparty/openslam_flirtlib/src/feature/NormalBlobDetector.h"
    "/home/hong/catkin_ws/src/radar_localization/Thirdparty/openslam_flirtlib/src/feature/CurvatureDetector.h"
    "/home/hong/catkin_ws/src/radar_localization/Thirdparty/openslam_flirtlib/src/feature/ShapeContext.h"
    "/home/hong/catkin_ws/src/radar_localization/Thirdparty/openslam_flirtlib/src/feature/BetaGrid.h"
    "/home/hong/catkin_ws/src/radar_localization/Thirdparty/openslam_flirtlib/src/feature/AbstractFeatureSetMatcher.h"
    "/home/hong/catkin_ws/src/radar_localization/Thirdparty/openslam_flirtlib/src/feature/RansacFeatureSetMatcher.h"
    "/home/hong/catkin_ws/src/radar_localization/Thirdparty/openslam_flirtlib/src/feature/RansacMultiFeatureSetMatcher.h"
    )
endif()

