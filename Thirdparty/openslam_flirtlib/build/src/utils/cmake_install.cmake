# Install script for directory: /home/hong/catkin_ws/src/radar_localization/Thirdparty/openslam_flirtlib/src/utils

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
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/flirtlib/libutils.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/flirtlib/libutils.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/flirtlib/libutils.so"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/flirtlib" TYPE SHARED_LIBRARY FILES "/home/hong/catkin_ws/src/radar_localization/Thirdparty/openslam_flirtlib/build/src/utils/libutils.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/flirtlib/libutils.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/flirtlib/libutils.so")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/flirtlib/libutils.so")
    endif()
  endif()
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/flirtlib/utils" TYPE FILE FILES
    "/home/hong/catkin_ws/src/radar_localization/Thirdparty/openslam_flirtlib/src/utils/Regression.h"
    "/home/hong/catkin_ws/src/radar_localization/Thirdparty/openslam_flirtlib/src/utils/Convolution.h"
    "/home/hong/catkin_ws/src/radar_localization/Thirdparty/openslam_flirtlib/src/utils/Convolution.hpp"
    "/home/hong/catkin_ws/src/radar_localization/Thirdparty/openslam_flirtlib/src/utils/HistogramDistances.h"
    "/home/hong/catkin_ws/src/radar_localization/Thirdparty/openslam_flirtlib/src/utils/HistogramDistances.hpp"
    "/home/hong/catkin_ws/src/radar_localization/Thirdparty/openslam_flirtlib/src/utils/SimplePeakFinder.h"
    "/home/hong/catkin_ws/src/radar_localization/Thirdparty/openslam_flirtlib/src/utils/SimpleMinMaxPeakFinder.h"
    "/home/hong/catkin_ws/src/radar_localization/Thirdparty/openslam_flirtlib/src/utils/PoseEstimation.h"
    )
endif()

