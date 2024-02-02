#!/bin/bash
cd /home/hong/catkin_ws/devel/lib/radar_localization
./radar_localization "/home/hong/catkin_ws/src/radar_localization/loop2_config.yaml" &
wait && echo "Finished 1 job with loop closure."

