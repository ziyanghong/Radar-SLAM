# Install script for directory: /home/hong/catkin_ws/src/radar_localization/Thirdparty/openslam_flirtlib/src/applications

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
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/ransacLoopClosureTest" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/ransacLoopClosureTest")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/ransacLoopClosureTest"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/bin" TYPE EXECUTABLE FILES "/home/hong/catkin_ws/src/radar_localization/Thirdparty/openslam_flirtlib/build/src/applications/ransacLoopClosureTest")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/ransacLoopClosureTest" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/ransacLoopClosureTest")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/ransacLoopClosureTest"
         OLD_RPATH "/home/hong/catkin_ws/src/radar_localization/Thirdparty/openslam_flirtlib/build/src/feature:/home/hong/catkin_ws/src/radar_localization/Thirdparty/openslam_flirtlib/build/src/sensorstream:/home/hong/catkin_ws/src/radar_localization/Thirdparty/openslam_flirtlib/build/src/sensors:/home/hong/catkin_ws/src/radar_localization/Thirdparty/openslam_flirtlib/build/src/utils:/home/hong/catkin_ws/src/radar_localization/Thirdparty/openslam_flirtlib/build/src/geometry:/opt/ros/kinetic/lib/x86_64-linux-gnu:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/ransacLoopClosureTest")
    endif()
  endif()
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/ransacLoopClosureDraw" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/ransacLoopClosureDraw")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/ransacLoopClosureDraw"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/bin" TYPE EXECUTABLE FILES "/home/hong/catkin_ws/src/radar_localization/Thirdparty/openslam_flirtlib/build/src/applications/ransacLoopClosureDraw")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/ransacLoopClosureDraw" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/ransacLoopClosureDraw")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/ransacLoopClosureDraw"
         OLD_RPATH "/home/hong/catkin_ws/src/radar_localization/Thirdparty/openslam_flirtlib/build/src/feature:/home/hong/catkin_ws/src/radar_localization/Thirdparty/openslam_flirtlib/build/src/sensorstream:/home/hong/catkin_ws/src/radar_localization/Thirdparty/openslam_flirtlib/build/src/sensors:/home/hong/catkin_ws/src/radar_localization/Thirdparty/openslam_flirtlib/build/src/utils:/opt/ros/kinetic/lib/x86_64-linux-gnu:/home/hong/catkin_ws/src/radar_localization/Thirdparty/openslam_flirtlib/build/src/geometry:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/ransacLoopClosureDraw")
    endif()
  endif()
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/flirtDemo" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/flirtDemo")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/flirtDemo"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/bin" TYPE EXECUTABLE FILES "/home/hong/catkin_ws/src/radar_localization/Thirdparty/openslam_flirtlib/build/src/applications/flirtDemo")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/flirtDemo" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/flirtDemo")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/flirtDemo"
         OLD_RPATH "/home/hong/catkin_ws/src/radar_localization/Thirdparty/openslam_flirtlib/build/src/gui:/home/hong/catkin_ws/src/radar_localization/Thirdparty/openslam_flirtlib/build/src/sensorstream:/home/hong/catkin_ws/src/radar_localization/Thirdparty/openslam_flirtlib/build/src/sensors:/opt/ros/kinetic/lib/x86_64-linux-gnu:/home/hong/catkin_ws/src/radar_localization/Thirdparty/openslam_flirtlib/build/src/feature:/home/hong/catkin_ws/src/radar_localization/Thirdparty/openslam_flirtlib/build/src/geometry:/home/hong/catkin_ws/src/radar_localization/Thirdparty/openslam_flirtlib/build/src/utils:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/flirtDemo")
    endif()
  endif()
endif()

