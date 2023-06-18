# integrated-navigation-driver
ROS Humble
### Install
```bash
sudo apt install ros-melodic-nmea-navsat-driver libgps-dev
mkdir src && cd src
git clone https://github.com/Terrensou/integrated-navigation-driver.git
git clone https://github.com/nobleo/rviz_satellite.git
cd .. && catkin_make
```
## Run
```bash
roslaunch integrated_navigation_driver integrated_navigation_pub.launch
```
## Config
Please see [config/config.yaml](config/config.yaml). 

## TODO
1. add support for odometry path export and visualization.
2. test navsatfix ENU/LLA output for visualization