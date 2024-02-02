# Install script for directory: /home/hong/catkin_ws/src/radar_localization/Thirdparty/codegen/lib/generateGlobalFeature

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
    set(CMAKE_INSTALL_CONFIG_NAME "Release")
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

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

# Set default install directory permissions.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/usr/bin/objdump")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/globalFeature/libglobalFeature.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/globalFeature/libglobalFeature.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/globalFeature/libglobalFeature.so"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/globalFeature" TYPE SHARED_LIBRARY FILES "/home/hong/catkin_ws/src/radar_localization/Thirdparty/codegen/lib/build/generateGlobalFeature/libglobalFeature.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/globalFeature/libglobalFeature.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/globalFeature/libglobalFeature.so")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/globalFeature/libglobalFeature.so")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/globalFeature" TYPE FILE FILES
    "/home/hong/catkin_ws/src/radar_localization/Thirdparty/codegen/lib/generateGlobalFeature/cart2pol.h"
    "/home/hong/catkin_ws/src/radar_localization/Thirdparty/codegen/lib/generateGlobalFeature/eml_setop.h"
    "/home/hong/catkin_ws/src/radar_localization/Thirdparty/codegen/lib/generateGlobalFeature/findpeaks.h"
    "/home/hong/catkin_ws/src/radar_localization/Thirdparty/codegen/lib/generateGlobalFeature/generateGlobalFeature.h"
    "/home/hong/catkin_ws/src/radar_localization/Thirdparty/codegen/lib/generateGlobalFeature/generateGlobalFeature_data.h"
    "/home/hong/catkin_ws/src/radar_localization/Thirdparty/codegen/lib/generateGlobalFeature/generateGlobalFeature_emxAPI.h"
    "/home/hong/catkin_ws/src/radar_localization/Thirdparty/codegen/lib/generateGlobalFeature/genreateGlobalFeature_emxutil.h"
    "/home/hong/catkin_ws/src/radar_localization/Thirdparty/codegen/lib/generateGlobalFeature/generateGlobalFeature_initialize.h"
    "/home/hong/catkin_ws/src/radar_localization/Thirdparty/codegen/lib/generateGlobalFeature/generateGlobalFeature_terminate.h"
    "/home/hong/catkin_ws/src/radar_localization/Thirdparty/codegen/lib/generateGlobalFeature/generateGlobalFeature_types.h"
    "/home/hong/catkin_ws/src/radar_localization/Thirdparty/codegen/lib/generateGlobalFeature/histcounts2.h"
    "/home/hong/catkin_ws/src/radar_localization/Thirdparty/codegen/lib/generateGlobalFeature/M2DP.h"
    "/home/hong/catkin_ws/src/radar_localization/Thirdparty/codegen/lib/generateGlobalFeature/mapElementsToBins.h"
    "/home/hong/catkin_ws/src/radar_localization/Thirdparty/codegen/lib/generateGlobalFeature/pca.h"
    "/home/hong/catkin_ws/src/radar_localization/Thirdparty/codegen/lib/generateGlobalFeature/repmat.h"
    "/home/hong/catkin_ws/src/radar_localization/Thirdparty/codegen/lib/generateGlobalFeature/rt_defines.h"
    "/home/hong/catkin_ws/src/radar_localization/Thirdparty/codegen/lib/generateGlobalFeature/rtGetInf.h"
    "/home/hong/catkin_ws/src/radar_localization/Thirdparty/codegen/lib/generateGlobalFeature/rtGetNaN.h"
    "/home/hong/catkin_ws/src/radar_localization/Thirdparty/codegen/lib/generateGlobalFeature/rt_nonfinite.h"
    "/home/hong/catkin_ws/src/radar_localization/Thirdparty/codegen/lib/generateGlobalFeature/scan2pointCloud.h"
    "/home/hong/catkin_ws/src/radar_localization/Thirdparty/codegen/lib/generateGlobalFeature/rtwtypes.h"
    "/home/hong/catkin_ws/src/radar_localization/Thirdparty/codegen/lib/generateGlobalFeature/sort.h"
    "/home/hong/catkin_ws/src/radar_localization/Thirdparty/codegen/lib/generateGlobalFeature/sortIdx.h"
    "/home/hong/catkin_ws/src/radar_localization/Thirdparty/codegen/lib/generateGlobalFeature/svd.h"
    "/home/hong/catkin_ws/src/radar_localization/Thirdparty/codegen/lib/generateGlobalFeature/svd1.h"
    "/home/hong/catkin_ws/src/radar_localization/Thirdparty/codegen/lib/generateGlobalFeature/xaxpy.h"
    "/home/hong/catkin_ws/src/radar_localization/Thirdparty/codegen/lib/generateGlobalFeature/xdotc.h"
    "/home/hong/catkin_ws/src/radar_localization/Thirdparty/codegen/lib/generateGlobalFeature/xnrm2.h"
    "/home/hong/catkin_ws/src/radar_localization/Thirdparty/codegen/lib/generateGlobalFeature/xrot.h"
    "/home/hong/catkin_ws/src/radar_localization/Thirdparty/codegen/lib/generateGlobalFeature/xrotg.h"
    "/home/hong/catkin_ws/src/radar_localization/Thirdparty/codegen/lib/generateGlobalFeature/xswap.h"
    "/home/hong/catkin_ws/src/radar_localization/Thirdparty/codegen/lib/generateGlobalFeature/xzsvdc.h"
    "/home/hong/catkin_ws/src/radar_localization/Thirdparty/codegen/lib/generateGlobalFeature/tmwtypes.h"
    )
endif()

