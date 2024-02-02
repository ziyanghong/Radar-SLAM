#!/bin/bash
cd /home/hong/catkin_ws/devel/lib/radar_localization
./radar_localization "/home/hong/catkin_ws/src/radar_localization/Example/../testConfig/001_config.yaml" > /dev/null  &
./radar_localization "/home/hong/catkin_ws/src/radar_localization/Example/../testConfig/002_config.yaml" > /dev/null  &
./radar_localization "/home/hong/catkin_ws/src/radar_localization/Example/../testConfig/003_config.yaml" > /dev/null  &
./radar_localization "/home/hong/catkin_ws/src/radar_localization/Example/../testConfig/004_config.yaml" > /dev/null  &

