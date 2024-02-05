# RadarSLAM

# 1. System enviroment:
Ubuntu 16.04

# 2. Dependancies:
Install the following dependancies
## ROS
version: kinetic
## g2o 
version: https://github.com/RainerKuemmerle/g2o/tree/20170730_git

Install g2o at /usr/local/include/g2o, you can change the installation path but you need to change the line at CMakeLists.txt: SET( G2O_ROOT /usr/local/include/g2o )
## Eigen
version 3.3.8
## OpenCV
version 3.3.1

# 3. How to build the sytem
1. Clone the repository 
2. Place it under the catkin workspace, e.g., catkin_ws/src
3. Use build.sh to build the Thirdparty libraries
   ```
   cd radar_slam
   chmod +x build.sh
   ./build.sh
   ```

4. Compile the code under catkin_ws
   ```
   cd ~/catkin_ws/
   catkin_make 
   source devel/setup.bash
   ```


# 4. Download the datasets at:
Download the Navtech radar sequences described in our paper from [here](https://drive.google.com/drive/folders/1Ebean2n64oBgZS4IxnG5CxxzG_u4FZ0R?usp=drive_link)

# 5. How to run
To run the SLAM system with the sequences described in our paper:
1. Addjust the yaml config file in /testConfigOxford, e.g., 001.yaml. Modify the sequence path according to where you store the data.
2. Under the project directory, simply run the system by the following command:
   rosrun radar_slam radar_slam testConfigOxford/001.yaml
3. Run rivz and load the RadarSLAM.rviz in /rviz_config for visualization


To run the SLAM system pwith your own polar radar scan:
1. Convert the raw polar scan to Cartesian images using the (modified) robotcar-dataset-sdk, download from [here](https://drive.google.com/file/d/1cMuuY_69dQQMX359yOCkGJ-FWXK_VBwC/view?usp=drive_link), check /robotcar-dataset-sdk/matlab/parseSensor.m for reference
2. Addjust the yaml config file in /testConfig, e.g., 001.yaml. Most importantly the numbers of cartImageWidth, cartImageHeight and cartImageScale
3. Then run the system by the following command:
   rosrun  radar_slam radar_slam PathToYourYamlFile.yaml
4. Run rivz and load the RadarSLAM.rviz in /rviz_config for visualization

# 6. Coordinate system
<img src="images/coordinate.png" width = 35% height = 35%/>

# 7. Citation
If you use Radar-SLAM in an academic work, please cite:
```bibtex
  @article{hong2022radarslam,
    title={RadarSLAM: A robust simultaneous localization and mapping system for all weather conditions},
    author={Hong, Ziyang and Petillot, Yvan and Wallace, Andrew and Wang, Sen},
    journal={The International Journal of Robotics Research},
    volume={41},
    number={5},
    pages={519--542},
    year={2022},
    publisher={SAGE Publications Sage UK: London, England}
  }

```
