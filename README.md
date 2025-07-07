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
```

#### Node setup


### OpenCV


## Usage

1. To setup ROS, run `source /opt/ros/kilted/setup.bash`
2. To test the motors run `g++ src/test_motor_driver.cpp src/motor_driver.cpp -Iinclude -lwiringPi -o motor_test` then `sudo ./motor_test`


https://docs.ros.org/en/kilted/Tutorials/Beginner-Client-Libraries.html