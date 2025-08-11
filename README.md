# SPF1
Solar Positioning Friend v1

## Purpose
Embedded Systems Development Lab project to create a robot that navigates to the sunniest spot for solar harvesting with a solar panel.

## Setup

### RPi
Tested on Ubuntu 22.04. For the GPIO on the RPi, install wiringpi with apt and enable permissions for I2C: 
```
sudo apt install wiringpi
sudo chmod 666 /dev/i2c-1
```

### ROS

#### Installation
Followed https://docs.ros.org/en/kilted/Installation/Ubuntu-Install-Debs.html to install ROS distro "Kilted Kaiju" using the following commands:
```
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb" # If using Ubuntu derivates use $UBUNTU_CODENAME
sudo dpkg -i /tmp/ros2-apt-source.deb
sudo apt update 
sudo apt upgrade
sudo apt install ros-dev-tools
sudo apt install ros-kilted-desktop
sudo apt-get install ros-kilted-ros-gz
sudo apt-get install ros-kilted-rclc
sudo apt install python3-colcon-common-extensions
```

#### Node setup
Create new packages with:
```
ros2 pkg create  --build-type ament_cmake --license Apache-2.0 pkg_name
```

Use the build.sh and run.sh script to build and run the node packages in this repo or build and run each package individually with:
```
colcon build --packages-select pkg_name # build
. install/setup.bash                    # install
ros2 run pkg_name node_name             # run
ros2 topic echo /topic name             # monitor
```

### MQTT
Install mosquitto with:
```
sudo apt install libmosquitto-dev mosquitto
```
In `/etc/mosquitto/mosquitto.conf`, add the following lines to bind the listener to the host IP on port 1883:
```
listener 1883 <HOST IP>
allow_anonymous true
```
Enable and start mosquitto with:
```
sudo systemctl enable mosquitto
sudo systemctl start mosquitto
```
Update the MQTT_HOST macro in the pr_node.cpp file to the host IP. Also update the WIFI_SSID, WIFI_PASS, and BROKER_URI on the ESP32.

### OpenCV

For OpenCL, install the following packages (not needed):
```
sudo apt install build-essential git cmake opencl-headers ocl-icd-opencl-dev
```

For OpenGL, install the following packages:
```
sudo apt install pkg-config mesa-utils libglu1-mesa-dev freeglut3-dev mesa-common-dev libglew-dev libglfw3-dev libglm-dev libao-dev libmpg123-dev
```

For Gstreamer, install the following packages:
```
sudo apt-get install libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev libgstreamer-plugins-bad1.0-dev gstreamer1.0-plugins-base gstreamer1.0-plugins-good gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly gstreamer1.0-libav gstreamer1.0-tools gstreamer1.0-x gstreamer1.0-alsa gstreamer1.0-gl gstreamer1.0-gtk3 gstreamer1.0-qt5 gstreamer1.0-pulseaudio
```

To build OpenCV run:
```
sudo apt install libharfbuzz-dev libtesseract-dev libgoogle-glog-dev libgflags-dev libgtk-3-dev pocl-opencl-icd ocl-icd-libopencl1
cd ~
git clone https://github.com/opencv/opencv.git
git clone https://github.com/opencv/opencv_contrib.git
cd ~/opencv
mkdir build
cd build
cmake -D CMAKE_BUILD_TYPE=RELEASE \
      -D CMAKE_INSTALL_PREFIX=/usr/local \
      -D OPENCV_EXTRA_MODULES_PATH=~/opencv_contrib/modules \
      -D WITH_GSTREAMER=ON \
      -D WITH_OPENGL=ON \
      -D WITH_OPENCL=ON \
      -D WITH_TBB=ON \
      -D CPU_BASELINE=NEON \
      -D BUILD_TESTS=OFF \
      -D BUILD_PERF_TESTS=OFF \
      -D BUILD_EXAMPLES=OFF ..
```

## Camera Calibration
1. Bring up a 9x6 chessboard with 20mm squares on [this](https://calib.io/pages/camera-calibration-pattern-generator) website or print it out and hold it in front of the camera
2. Run the capture_calibration_images.py script while moving the chessboard at different angles in front of the camera
3. Run the calibrate.py script to extract the camera parameters for the sun_finder node

## Documentation
Install the Doxygen package:
```
sudo apt install doxygen
```
To regenerate the docs and serve them, run:
```
bash docs.sh
```

## Quick Start

To build all the nodes, run:
```
bash build.sh
```
To run all the nodes, run:
```
bash run.sh
```
To generate the test executables, run
```
bash test.sh
```

# Resources:
- https://docs.ros.org/en/kilted/Tutorials/Beginner-Client-Libraries.html