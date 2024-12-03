# FinalProjectTurtle
Turtlebot final Project :)


## RealSense Setup
1. Installed on Ubuntu 20.04 LTS.
2. Remember to run
	```
		sudo cp config/99-realsense-libusb.rules /etc/udev/rules.d/
	
		sudo udevadm control --reload-rules&&udevadm trigger
	``` after install the dependencies.
2. Plug the camera when building librealsense2 SDK.
3. Run `realsense-viewer` to test if successfullt installed.

### Reference
1. Instruction: https://github.com/IntelRealSense/librealsense/blob/master/doc/installation.md
2. Debug video: https://www.bilibili.com/video/BV1iM4y1d7kL?spm_id_from=333.788.videopod.sections&vd_source=4da953d6172fefe3b5aabce7be3f5662


## ROS Setup
### Installation before dependencies
Follow the instruction here: https://wiki.ros.org/noetic/Installation/Ubuntu

### Installation of dependencies
1. Essential packages
```
sudo apt-get install ros-noetic-joy ros-noetic-teleop-twist-joy \
ros-noetic-teleop-twist-keyboard ros-noetic-laser-proc \
ros-noetic-rgbd-launch ros-noetic-rosserial-arduino \
ros-noetic-rosserial-python ros-noetic-rosserial-client \
ros-noetic-rosserial-msgs ros-noetic-amcl ros-noetic-map-server \
ros-noetic-move-base ros-noetic-urdf ros-noetic-xacro \
ros-noetic-compressed-image-transport ros-noetic-rqt* ros-noetic-rviz \
ros-noetic-gmapping ros-noetic-navigation ros-noetic-interactive-markers
```
2. Turtle bot packages
```
$ sudo apt install ros-noetic-dynamixel-sdk
$ sudo apt install ros-noetic-turtlebot3-msgs
$ sudo apt install ros-noetic-turtlebot3
```
3. RealSense Package for ROS1
	1. Follow the instrustion here: https://github.com/IntelRealSense/realsense-ros/tree/ros1-legacy?tab=readme-ov-file
	2. Fail to catkin_make: https://blog.csdn.net/tanmx219/article/details/122765853

## Test RealSense Using ROS:
* Run `roslaunch realsense2_camera rs_camera.launch` to open the camera. Then run `rosrun rviz rviz`
* See ros topic by install `ros-noetic-rqt-image-view` and run `rqt_image_view`
![](./Assets/ROSTopic.png)
* See point cloud by install `ros-noetic-rviz` and run `roslaunch realsense2_camera demo_pointcloud.launch `
![](./Assets/PointCloud.png)


## TODO For slam
Reference: https://github.com/IntelRealSense/realsense-ros/wiki/SLAM-with-D435i
