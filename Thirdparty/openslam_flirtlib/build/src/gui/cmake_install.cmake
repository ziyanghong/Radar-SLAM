# Install script for directory: /home/hong/catkin_ws/src/radar_localization/Thirdparty/openslam_flirtlib/src/gui

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
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/flirtlib/libgui.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/flirtlib/libgui.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/flirtlib/libgui.so"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/flirtlib" TYPE SHARED_LIBRARY FILES "/home/hong/catkin_ws/src/radar_localization/Thirdparty/openslam_flirtlib/build/src/gui/libgui.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/flirtlib/libgui.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/flirtlib/libgui.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/flirtlib/libgui.so"
         OLD_RPATH "/home/hong/catkin_ws/src/radar_localization/Thirdparty/openslam_flirtlib/build/src/sensorstream:/home/hong/catkin_ws/src/radar_localization/Thirdparty/openslam_flirtlib/build/src/feature:/home/hong/catkin_ws/src/radar_localization/Thirdparty/openslam_flirtlib/build/src/utils:/home/hong/catkin_ws/src/radar_localization/Thirdparty/openslam_flirtlib/build/src/sensors:/home/hong/catkin_ws/src/radar_localization/Thirdparty/openslam_flirtlib/build/src/geometry:/opt/ros/kinetic/lib/x86_64-linux-gnu:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/flirtlib/libgui.so")
    endif()
  endif()
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/flirtlib/gui" TYPE FILE FILES
    "/home/hong/catkin_ws/src/radar_localization/Thirdparty/openslam_flirtlib/src/gui/AbstractRenderer.h"
    "/home/hong/catkin_ws/src/radar_localization/Thirdparty/openslam_flirtlib/src/gui/Color.h"
    "/home/hong/catkin_ws/src/radar_localization/Thirdparty/openslam_flirtlib/src/gui/LaserReadingRenderer.h"
    "/home/hong/catkin_ws/src/radar_localization/Thirdparty/openslam_flirtlib/src/gui/InterestPointRenderer.h"
    "/home/hong/catkin_ws/src/radar_localization/Thirdparty/openslam_flirtlib/src/gui/PolarGridRenderer.h"
    "/home/hong/catkin_ws/src/radar_localization/Thirdparty/openslam_flirtlib/src/gui/SensorStreamWidget.h"
    "/home/hong/catkin_ws/src/radar_localization/Thirdparty/openslam_flirtlib/src/gui/RendererWidget.h"
    "/home/hong/catkin_ws/src/radar_localization/Thirdparty/openslam_flirtlib/src/gui/MultiScaleDetectorPlotWidget.h"
    "/home/hong/catkin_ws/src/radar_localization/Thirdparty/openslam_flirtlib/src/gui/PeakFinderPresenter.h"
    "/home/hong/catkin_ws/src/radar_localization/Thirdparty/openslam_flirtlib/src/gui/SimplePeakFinderPresenter.h"
    "/home/hong/catkin_ws/src/radar_localization/Thirdparty/openslam_flirtlib/src/gui/DetectorPresenter.h"
    "/home/hong/catkin_ws/src/radar_localization/Thirdparty/openslam_flirtlib/src/gui/MultiScaleDetectorPresenter.h"
    "/home/hong/catkin_ws/src/radar_localization/Thirdparty/openslam_flirtlib/src/gui/MultiScaleCurvatureDetectorPresenter.h"
    "/home/hong/catkin_ws/src/radar_localization/Thirdparty/openslam_flirtlib/src/gui/MultiScaleNormalDetectorPresenter.h"
    "/home/hong/catkin_ws/src/radar_localization/Thirdparty/openslam_flirtlib/src/gui/DetectorChooserPresenter.h"
    "/home/hong/catkin_ws/src/radar_localization/Thirdparty/openslam_flirtlib/src/gui/DescriptorChooserPresenter.h"
    "/home/hong/catkin_ws/src/radar_localization/Thirdparty/openslam_flirtlib/src/gui/ParameterWidget.h"
    "/home/hong/catkin_ws/src/radar_localization/Thirdparty/openslam_flirtlib/src/gui/TabbedParameterWidget.h"
    "/home/hong/catkin_ws/src/radar_localization/Thirdparty/openslam_flirtlib/src/gui/BoxParameterWidget.h"
    "/home/hong/catkin_ws/src/radar_localization/Thirdparty/openslam_flirtlib/src/gui/DescriptorPresenter.h"
    "/home/hong/catkin_ws/src/radar_localization/Thirdparty/openslam_flirtlib/src/gui/ShapeContextPresenter.h"
    "/home/hong/catkin_ws/src/radar_localization/Thirdparty/openslam_flirtlib/src/gui/BetaGridPresenter.h"
    "/home/hong/catkin_ws/src/radar_localization/Thirdparty/openslam_flirtlib/src/gui/DescriptorWidget.h"
    "/home/hong/catkin_ws/src/radar_localization/Thirdparty/openslam_flirtlib/src/gui/PolarGridGraphicsItem.h"
    "/home/hong/catkin_ws/src/radar_localization/Thirdparty/openslam_flirtlib/src/gui/CorrespondenceRenderer.h"
    "/home/hong/catkin_ws/src/radar_localization/Thirdparty/openslam_flirtlib/src/gui/FeatureSetMatcherPresenter.h"
    "/home/hong/catkin_ws/src/radar_localization/Thirdparty/openslam_flirtlib/src/gui/RansacPresenter.h"
    )
endif()

