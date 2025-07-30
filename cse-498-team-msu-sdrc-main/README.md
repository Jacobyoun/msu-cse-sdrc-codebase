# CSE 498 team MSU SDRC
The project runs on a 6.x Jetpack firmware Jetson Orin Nano and 22.04 Ubuntu kernel.<br>
If a completely new Jetson Orin Nano is acquired, it is recommended that the firmware is not update to 6.x but kept at 5.1. If Ubuntu 22.04 work on that firmware, in-depth features of the camera could be used such as heat map and IMU. Jetpack 6.x  has yet supported them.
```
mkdir <your_directory>
cd <your_directory>
git clone git@gitlab.msu.edu:phivu/cse-498-team-msu-sdrc.git
cd cse-498-team-msu-sdrc
git checkout -b <your_branch>
git config user.name <your_username>
git config user.email <your_email>
```
# Installation Guide:
Make sure that the Ubuntu version is 22.04

Install jtop
```
sudo apt install python3-pip
sudo pip3 install -U jetson-stats
```

Reboot

Create 4GB Swap File

- Use jtop
```
jtop
```
-- Go to Mem Tab, add 4GB swap file

Install ROS2, we are installing ROS2 Humble: https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html

Setup locale
```
locale  # check for UTF-8

sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

locale  # verify settings
```
Setup sources

Ensure that the Ubuntu Universe repository is enabled.
```
sudo apt install software-properties-common
sudo add-apt-repository universe
```

Add the ROS 2 GPG key with apt
```
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/rosarchive-keyring.gpg
```

Then add the repository to the sources list
```
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```
```
sudo apt update
sudo apt upgrade
```
```
sudo apt install ros-humble-desktop
sudo apt install ros-dev-tools
```
Test Installation

One terminal:
```
source /opt/ros/humble/setup.bash
ros2 run demo_nodes_cpp talker
```
Second terminal:
```
source /opt/ros/humble/setup.bash
ros2 run demo_nodes_py listener
```
Add to ~/.bashrc
```
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

Install colcon
```
sudo apt install python3-colcon-common-extensions
```

# Setup ROS2 environment for communication between laptop and car

Check environment variables
```
printenv | grep -i ROS
```
Check that variables like ROS_DISTRO and ROS_VERSION are set.
```
ROS_VERSION=2
ROS_PYTHON_VERSION=3
ROS_DISTRO=humble
```
Change Domain ID and Localhost variable to enable communication
```
echo "export ROS_DOMAIN_ID=<your_domain_id>" >> ~/.bashrc
echo "export ROS_LOCALHOST_ONLY=0" >> ~/.bashrc
```

In order for them to send data over, the simplest way is to turn off the firewall
```
sudo apt update
sudo apt install -y ufw
sudo ufw status verbose
sudo ufw disable
```
# Install sensor and system libraries
```
sudo apt update
sudo apt upgrade
```
```
sudo pip3 install numpy
sudo pip3 install opencv-python
sudo pip3 install pyrealsense2
sudo pip3 install pynmeagps
sudo pip3 install matplotlib
sudo pip3 install rplidar-roboticia
sudo pip3 install hidapi
```
# Setup our ROS2 code on the laptop and Jetson

YOU MUST DO THE FOLLOWING ON BOTH THE JETSON AND THE LAPTOP

Create a directory to put the ROS source code
```
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
```

Every time you make a change to a file in the src folder created above you must colcon build and source:
```
colcon build
source install/setup.bash
```
You will not need this command right now but if you want to create a new package later on:
```
ros2 pkg create <your_package> --build-type ament_python --dependencies rclpy std_msgs sensor_msgs
```

Copy over the content of the ROS2 Code directory that you cloned earlier into the directory you just created:
```
cp -a ~/your_direcory/cse-498-team-msu-sdrc/ROS2\ Code/src/. ~/ros2_ws/src
```
Or directly copy paste the directories over using the file explorer

Also delete the Build, Install, and Log files if present:
```
cd ~/ros2_ws
rm -r Install
rm -r Build
rm -r Log
```

Build one more time. This will create the packages and the second command sources the workspace.
```
colcon build
source install/setup.bash
```

Now all nodes are ready to be used. Hook a sensor up and test out. For exmaple, the LiDAR. Run each node on different terminals

Using the laptop<br>
Publisher
```
sudo bash -c "source $HOME/ros2_humble/install/setup.bash; source $HOME/ros2_ws/install/setup.bash; ros2 run lidar_visualizer lidar_publisher"
```
Subscriber
```
sudo bash -c "source $HOME/ros2_humble/install/setup.bash; source $HOME/ros2_ws/install/setup.bash; ros2 run lidar_visualizer lidar_subscriber"
```
Using the Jetson<br>
Publisher
```
sudo bash -c "source /opt/ros/humble/setup.bash; source $HOME/ros2_ws/install/setup.bash; ros2 run lidar_visualizer lidar_publisher"
```
Subscriber
```
sudo bash -c "source /opt/ros/humble/setup.bash; source $HOME/ros2_ws/install/setup.bash; ros2 run lidar_visualizer lidar_subscriber"
```

# Setting up F1/10 on Jetson
```
cd $HOME
mkdir -p f1tenth_ws/src
cd f1tenth_ws
colcon build
cd src
git clone https://github.com/f1tenth/f1tenth_system.git
cd f1tenth_system
git submodule update --init --force --remote
```
Make VESC ready for humble
```
git fetch origin
cd ~/f1tenth_ws/src/f1tenth_system/vesc/
git show origin/ros2:vesc_ackermann/src/ackermann_to_vesc.cpp > vesc_ackermann/src/ackermann_to_vesc.cpp
git show origin/ros2:vesc_ackermann/src/vesc_to_odom.cpp > vesc_ackermann/src/vesc_to_odom.cpp
cd $HOME/f1tenth_ws
sudo rosdep init
rosdep update
rosdep install --from-paths src -i -y
sudo apt install ros-humble-asio-cmake-module
colcon build
```

VESC setup – This has already been done<br> Plug VESC into PC. Use VESC Tool to program the VESC:<br> https://vesc-project.com/vesc_tool<br> Use Autoconnect wizard to connect to the VESC<br> Motor setup<br> The motor is a Medium Inrunner ( ~750g)<br> Max Power Loss: 140.0 W<br> Openloop ERPM : 1400<br> Sensorless ERPM: 3500<br> Motor Poles: 4<br> Battery<br> Battery Type: BATTERY_tYPE_LIION_3_0__4_2<br> Battery Cells Series: 2<br> Battery Capacity: 3.000 Ah<br> After running detection, there are other settings you need to check:<br> Under General Motor Settings → Current<br> Motor Current Max: 30.00A<br> Motor Current Max Brake: -30.00A<br> Absolute Maximum Current: 60.00 A<br> Under FOC → Sensorless<br> Openloop ERPM: 1400.00<br> Under App Settings → General<br> Enable Servo Output : True<br> Note: After changing these values, write the Motor AND App settings to the VESC<br> UDEV Rules<br> Make sure to setup udev rules:<br> As root, open /etc/udev/rules.d/99-vesc.rules and copy in<br> the following rule for the VESC:<br> KERNEL=="ttyACM[0-9]*", ACTION=="add",<br> ATTRS{idVendor}=="0483", ATTRS{idProduct}=="5740",<br> MODE="0666", GROUP="dialout", SYMLINK+="sensors/vesc"<br> You should reboot the machine for the changes to take effect.<br> Wireless Network<br> Following the instructions on setting up the wireless network on the F1/10 site. This will be dependent on your wireless network setup.<br>

# Complete setup of the car driving:

Car Setup:

First plug in the router.

Next, plug in the Jetson and the VESC.

On the laptop, you need to SSH into the jetson. Make sure it is connected to the router (GL-MT3000-56d):
```
ssh younge32@192.168.8.200
```
When prompted for the password type:
```
Random1234
```
Start all of the car nodes (speed_max can bet set anywhere between 0 - 100,000 rpm. It will default to 6,000 if not set):
```
sudo bash -c "source /opt/ros/humble/setup.bash; source $HOME/ros2_ws/install/setup.bash; ros2 launch start_car start_car_launch.py speed_max:=10000.0"
```

Open another terminal tab that is not being used to SSH:
Start the laptop website:
```
cd ~/<your_directory>/cse-498-team-msu-sdrc/Local\ Website/
flask run 
```
Click on the link generated by flask. 
Then click on the "Start ROS Nodes" button in the website.

If the start script doesn't work or only certain sensors are needed to be tested, use the single commands:
Sensor nodes commands on the Jetson<br>
IMU:
```
sudo bash -c "source /opt/ros/humble/setup.bash; source $HOME/ros2_ws/install/setup.bash; ros2 run vesc_imu imu_publisher"
```
GNSS:
```
sudo bash -c "source /opt/ros/humble/setup.bash; source $HOME/ros2_ws/install/setup.bash; ros2 run gnss_data gnss_publisher"
```
LiDAR:
```
sudo bash -c "source /opt/ros/humble/setup.bash; source $HOME/ros2_ws/install/setup.bash; ros2 run lidar_visualizer lidar_publisher"
```
Camera:
```
sudo bash -c "source /opt/ros/humble/setup.bash; source $HOME/ros2_ws/install/setup.bash; ros2 run my_camera_sensor camera_publisher"
```
Vesc setup:<br> On car
```
sudo bash -c "source /opt/ros/humble/setup.bash; source $HOME/ros2_ws/install/setup.bash; ros2 run controller_receiver control_receive"
```
In another terminal
```
sudo bash -c "source /opt/ros/humble/setup.bash; source $HOME/ros2_ws/install/setup.bash; ros2 run vesc_driver vesc_driver_node --ros-args -p port:=/dev/vesc_device -p baud:=115200 -p speed_max:=3000.0 -p servo_max:=1.0 -p servo_min:=0.0 -p brake_min:=-20000.0 -p brake_max:=20000.0 -p enable_imu:=true -p imu_rate:=50.0"
```
On the laptop
```
sudo bash -c "source $HOME/ros2_humble/install/setup.bash; source $HOME/ros2_ws/install/setup.bash; ros2 run controller_publisher control_publish"
```
Loopback test node:<br> On the laptop
```
sudo bash -c "source $HOME/ros2_humble/install/setup.bash; python3 temp2/cse-498-team-msu-sdrc/ROS2\ Code/src/loopback_test/loopback_test_laptop.py"
```
Rosbridge setup (MAKE SURE THE PUBLISHERS ARE PUBLISHING FIRST)

In one terminal
```
sudo bash -c "cd $HOME/local_web/cse-498-team-msu-sdrc && \
source $HOME/ros2_humble/install/setup.bash && \
source $HOME/ros2_ws/install/setup.bash && \
ros2 run rosbridge_server rosbridge_websocket"
```
In a second terminal
```
flask run
```
# All sensor nodes commands
IMU:<br>
Publisher on laptop
```
sudo bash -c "source $HOME/ros2_humble/install/setup.bash; source $HOME/ros2_ws/install/setup.bash; ros2 run imu_visualizer imu_publisher"
```
Subscriber on laptop
```
sudo bash -c "source $HOME/ros2_humble/install/setup.bash; source $HOME/ros2_ws/install/setup.bash; ros2 run imu_visualizer imu_subscriber"
```
Publisher on Jetson
```
sudo bash -c "source /opt/ros/humble/setup.bash; source $HOME/ros2_ws/install/setup.bash; ros2 run imu_visualizer imu_publisher"
```
Subscriber on Jetson
```
sudo bash -c "source /opt/ros/humble/setup.bash; source $HOME/ros2_ws/install/setup.bash; ros2 run imu_visualizer imu_subscriber"
```

VESC IMU:<br>
Publisher on Jetson
```
sudo bash -c "source /opt/ros/humble/setup.bash; source $HOME/ros2_ws/install/setup.bash; ros2 run vesc_imu imu_publisher"
```

Echo of the VESC IMU
```
sudo bash -c "source /opt/ros/humble/setup.bash; source $HOME/ros2_ws/install/setup.bash; ros2 topic echo /imu/data_raw"
```
GNSS:

Publisher on laptop
```
sudo bash -c "source $HOME/ros2_humble/install/setup.bash; source $HOME/ros2_ws/install/setup.bash; ros2 run gnss_data gnss_publisher"
```
Subscriber on laptop
```
sudo bash -c "source $HOME/ros2_humble/install/setup.bash; source $HOME/ros2_ws/install/setup.bash; ros2 run gnss_data gnss_subscriber"
```
Publisher on Jetson
```
sudo bash -c "source /opt/ros/humble/setup.bash; source $HOME/ros2_ws/install/setup.bash; ros2 run gnss_data gnss_publisher"
```
Subscriber on Jetson
```
sudo bash -c "source /opt/ros/humble/setup.bash; source $HOME/ros2_ws/install/setup.bash; ros2 run gnss_data gnss_subscriber"
```


Lidar:

Publisher on laptop
```
sudo bash -c "source $HOME/ros2_humble/install/setup.bash; source $HOME/ros2_ws/install/setup.bash; ros2 run lidar_visualizer lidar_publisher"
```
Subscriber on laptop
```
sudo bash -c "source $HOME/ros2_humble/install/setup.bash; source $HOME/ros2_ws/install/setup.bash; ros2 run lidar_visualizer lidar_subscriber"
```
Publisher on Jetson
```
sudo bash -c "source /opt/ros/humble/setup.bash; source $HOME/ros2_ws/install/setup.bash; ros2 run lidar_visualizer lidar_publisher"
```
Subscriber on Jetson
```
sudo bash -c "source /opt/ros/humble/setup.bash; source $HOME/ros2_ws/install/setup.bash; ros2 run lidar_visualizer lidar_subscriber"
```
Camera:

Publisher on laptop
```
sudo bash -c "source $HOME/ros2_humble/install/setup.bash; source $HOME/ros2_ws/install/setup.bash; ros2 run my_camera_sensor camera_publisher"
```
Publisher on Jetson
```
sudo bash -c "source /opt/ros/humble/setup.bash; source $HOME/ros2_ws/install/setup.bash; ros2 run my_camera_sensor camera_publisher"
```

Depth Camera (in this order):

Terminal 1:
```
ros2 launch realsense2_camera rs_launch.py enable_depth:=true enable_infra1:=true enable_infra2:=true enable_rgb:=false
```
Terminal 2:
```
ros2 run my_depth_sensor depth_publisher
```

Control commands:

On car:
```
sudo bash -c "source /opt/ros/humble/setup.bash; source $HOME/ros2_ws/install/setup.bash; ros2 run controller_receiver control_receive"
```
```
ros2 run controller_receiver control_receive
```
In another Terminal
```
sudo bash -c "source /opt/ros/humble/setup.bash; source $HOME/ros2_ws/install/setup.bash; ros2 run vesc_driver vesc_driver_node --ros-args -p port:=/dev/vesc_device -p baud:=115200 -p speed_max:=3000.0 -p servo_max:=1.0 -p servo_min:=0.0 -p brake_min:=-20000.0 -p brake_max:=20000.0 -p enable_imu:=true -p imu_rate:=50.0"
```
```
ros2 run vesc_driver vesc_driver_node --ros-args -p port:=/dev/vesc_device -p baud:=115200 -p speed_max:=3000.0 -p servo_max:=1.0 -p servo_min:=0.0 -p brake_min:=-20000.0 -p brake_max:=20000.0 -p enable_imu:=true -p imu_rate:=50.0
```
On the laptop:
```
sudo bash -c "source $HOME/ros2_humble/install/setup.bash; source $HOME/ros2_ws/install/setup.bash; ros2 run controller_publisher control_publish"
```
```
ros2 run controller_publisher control_publish
```


Loopback test node:<br>
On the laptop:
```
sudo bash -c "source $HOME/ros2_humble/install/setup.bash; python3 temp2/cse-498-team-msu-sdrc/ROS2\ Code/src/loopback_test/loopback_test_laptop.py"
```
On the car (assuming working dir of home):
```
sudo bash -c "source /opt/ros/humble/setup.bash; python3 ros2_ws/src/loopback_test/loopback_test_car.py"
```

Rosbridge setup (MAKE SURE THE PUBLISHER IS PUBLISHING FIRST)

In one terminal
```
sudo bash -c "cd $HOME/local_web/cse-498-team-msu-sdrc && \
source $HOME/ros2_humble/install/setup.bash && \
source $HOME/ros2_ws/install/setup.bash && \
ros2 run rosbridge_server rosbridge_websocket"
```
In a second terminal
```
flask run
```

Start all of the laptop nodes:
```
sudo bash -c "source $HOME/ros2_humble/install/setup.bash; source $HOME/ros2_ws/install/setup.bash; ros2 launch start_laptop start_laptop_launch.py"
```

Start all of the car nodes (speed_max can bet set anywhere between 0 - 100,000 rpm. It will default to 6,000 if not set):
```
sudo bash -c "source /opt/ros/humble/setup.bash; source $HOME/ros2_ws/install/setup.bash; ros2 launch start_car start_car_launch.py speed_max:=10000.0"
```


# How to use the Local Website
When running for the first time, create a directory named ros2_bags at the home/username/Desktop/ros2_bags to store the recordings
```
cd
mkdir ros2_bags
```

1. Click the start ROS Nodes button.<br>
2. Choose the sensors you want from the drop down windows.<br>
3. If you want to record, click on the red record circle. (left most button)<br>
4. The record button will take about 9 seconds to start recording. Hit record again when you finish recording.<br>
5. If you want to replay then click the "Select a Rosbag" button, a window will show up.<br>
6. Navigate to the ros2_bags directory, double click on the bag that you want to replay, and click upload. The name of the rosbag is the time and date it is recorded.(Not the file named "ros_bag" inside of it)<br>
7. Click the mode switching button (second left most button) to change to the replaying mode. After 2 seconds, the bag will be automatically replayed.<br>
8. Click the pause/continue button (third left most button) to pause/continue in your recording session to your liking.(Due to we are using ros service for this function, it will take a second to work)<br>
9. To resume to the record/display mode, click the mode switching button.
10. Click the reset chart button (right most button) if you want to reset the states of the sensors being shown on the drop down windows.
11. While using the online map function, once after you to connect to internet and load the map of area you need, you are good to disconnect from internet. The function will also ask you to insert your current location data to help locate your location to center of the map window if you do not know the latitude and longitude of your current place, you can also insert 0,0 and drag the map to the location you want.

During record/replay process, try to avoid forcibly terminating the terminal. If it happens, you can 1. close the terminal you were running the website 2. run command 
```
ps aux | grep '[r]os2 bag play'
```
In other terminal to check if ther is any ros bag node not close correctly. 3. if there any ros bag process still running in back ground, you can use 
```
sudo pkill -9 -f "ros2 bag"
```
To stop all the ros bag nodes. After you are down, do step 2 on more time to make sure all process is closed<br>

