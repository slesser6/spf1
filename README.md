# SPF1
Solar Positioning Friend v1

## Purpose
Embedded Systems Development Lab project to create a robot that navigates to the sunniest spot for solar harvesting with a solar panel

## Setup



### ROS

#### Installation
Followed https://docs.ros.org/en/kilted/Installation/Ubuntu-Install-Debs.html to install ROS distro "Kilted Kaiju"
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
sudo apt install wiringpi
sudo apt install python3-colcon-common-extensions
sudo apt install libmosquitto-dev mosquitto
```

#### Node setup
ros2 pkg create  --build-type ament_cmake --license Apache-2.0 pkg_name
colcon build --packages-select pkg_name
. install/setup.bash
ros2 run pkg_name node_name

#### MQTT setup
sudo systemctl enable mosquitto
sudo systemctl start mosquitto

Bind listener to host IP on port 1883

#### I2C setup

sudo chmod 666 /dev/i2c-1

### OpenCV

OpenCL
```
sudo apt install build-essential git cmake opencl-headers ocl-icd-opencl-dev
```

OpenGL
```
sudo apt install pkg-config mesa-utils libglu1-mesa-dev freeglut3-dev mesa-common-dev libglew-dev libglfw3-dev libglm-dev libao-dev libmpg123-dev
```

Gstreamer
```
sudo apt-get install libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev libgstreamer-plugins-bad1.0-dev gstreamer1.0-plugins-base gstreamer1.0-plugins-good gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly gstreamer1.0-libav gstreamer1.0-tools gstreamer1.0-x gstreamer1.0-alsa gstreamer1.0-gl gstreamer1.0-gtk3 gstreamer1.0-qt5 gstreamer1.0-pulseaudio
```

OpenCV
```
sudo apt install libharfbuzz-dev libtesseract-dev libgoogle-glog-dev libgflags-dev libgtk-3-dev
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

## Usages

1. To setup ROS, run `source /opt/ros/kilted/setup.bash`
2. To test the motors run `g++ src/test_motor_driver.cpp src/motor_driver.cpp -Iinclude -lwiringPi -o motor_test`
3. To test the OpenCV components run 
```
g++ test_opencl.cpp -o test_opencl `pkg-config --cflags --libs opencv4`
g++ test_opengl.cpp -o test_opengl `pkg-config --cflags --libs opencv4`
g++ test_gstreamer.cpp -o test_gstreamer `pkg-config --cflags --libs opencv4`
g++ test_dual_camera.c -o test_dual_camera `pkg-config --cflags --libs opencv4`
```

https://docs.ros.org/en/kilted/Tutorials/Beginner-Client-Libraries.html