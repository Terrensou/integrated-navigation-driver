# integrated-navigation-reader
ROS Humble, read NMEA message(currently support: GPCHC, GPFPD, GPFPS, GPFPS_BIN, GPGGA, GTIMU, GTIMU_BIN, NVSTD) and Span logs(currently support: INSPVAXA, INSPVAXB) from serial and parse them to sensor_msgs::NavsatFIx, sensor_msgs::Imu, nav_msg::Odometry and nav_msgs::Path.
### Install
```bash
sudo apt install ros-melodic-nmea-navsat-driver libgps-dev
mkdir src && cd src
git clone https://github.com/Terrensou/integrated-navigation-reader.git
git clone https://github.com/nobleo/rviz_satellite.git
cd .. && catkin_make
```
## Run
```bash
roslaunch integrated_navigation_reader integrated_navigation_pub.launch
```
## Config
Please see [config/config.yaml](config/config.yaml). 

## TODO
1. add support for odometry path export and visualization.
2. test navsatfix ENU/LLA output for visualization
3. support rotate imu data.
4. split GPFPD statu for diff company
5. support prase big endian binary data



## known issue 
1. If Imu_generate_from: GPFPD-GTIMU with visualization, could cause 'Invalid argument passed to canTransform argument source_frame in tf2 frame_ids cannot be empty'

