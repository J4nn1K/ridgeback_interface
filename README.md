## Orbbec Astra Installation
Install dependencies
```bash
sudo apt install libgflags-dev  ros-$ROS_DISTRO-image-geometry ros-$ROS_DISTRO-camera-info-manager ros-$ROS_DISTRO-image-transport ros-$ROS_DISTRO-image-publisher libgoogle-glog-dev libusb-1.0-0-dev libeigen3-dev
```
Install libuvc  
```bash
git clone https://github.com/libuvc/libuvc.git
cd libuvc
mkdir build && cd build
cmake .. && make -j4
sudo make install
sudo ldconfig
```
In ROS workspace:
```bash
 cd ~/ros_ws/src
 git clone https://github.com/orbbec/ros_astra_camera.git
```
Build:
```bash
cd ~/ros_ws
catkin_make
```
Install udev rules:
```bash
cd ~/ros_ws
source ./devel/setup.bash
roscd astra_camera
./scripts/create_udev_rules
sudo udevadm control --reload && sudo  udevadm trigger
```
Start the camera node:
```bash
source ./devel/setup.bash 
roslaunch astra_camera astra.launch
```