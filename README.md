
<p align="center">
<img src="Documentation/Figures/data_flow_diagram.png" alt="Front readme image" width=100%>
</p>

## Installation

#### Prerequisites
1. docker, if you don't have you can install it following the docker install instructions [link](https://docs.docker.com/get-docker/).


#### Clone repo
```bash
git clone https://github.com/MoTahoun/docker_tii_assessment.git
```

#### Build docker
The following command will build the [Dockerfile](https://github.com/MoTahoun/docker_tii_assessment/blob/main/Dockerfile) that encapsulate all the requirements to run the ros package
```bash
cd docker_tii_assessment
docker build -t lidar_filtering .
```

## Usage
### Run docker
In the terminal and directory
```bash
./docker_init.sh
```

### Build the ROS Package
```bash
cd tii_ws
catkin_make
source ~/.bashrc
```

### Run the filtering algorithm
Launch the package using a launch file
```bash
roslaunch os_lidar_filtering os_lidar_processor.launch
```

### Play the bag
Please put the bag file inside the package: "/workspace/tii_ws/src/os_lidar_filtering/data/[BAG_NAME].bag"
Open other terminal and play the bag
```bash
docker exec -it lidar_filtering bash
rosbag play -l /workspace/tii_ws/src/os_lidar_filtering/data/[BAG_NAME].bag
```

### Run dynamic configure to fine tune the parameters
Open other terminal and play the bag
```bash
docker exec -it lidar_filtering bash
rosrun rqt_reconfigure rqt_reconfigure
```


<!---
#### Packages Installation
1. Install ROS Noetic following the steps in the following [link](https://wiki.ros.org/noetic/Installation/Ubuntu).
2. Install ROS Foxy following the steps in the following [link](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html).
3. Install ros2 bridge package via debian (Tested only on ROS Foxy),
```bash
sudo apt-get install ros-foxy-ros1-bridge
```
#### Environment setup
1. Open the .bashrc file by any editor, ```gedit ~/.bashrc ```.
2. Look for the ``` source /opt/ros/noetic/setup.bash``` and add alias to it, ```alias noetic = "source /opt/ros/noetic/setup.bash; echo Noetic \(ROS1\) is active"```
3. Similarly, do the same for ROS Foxy, Look for the ``` source /opt/ros/foxy/setup.bash``` and add alias to it, ```alias foxy = "source /opt/ros/foxy/setup.bash; echo Foxy \(ROS2\) is active"```

#### Test the envirmontal setup
1. Open a new terminal and source the .bashrc file via ```source ~/.bashrc```, then test the alias for ros1 by typing ```noetic```, the following message should show up, ```Noetic (ROS1) is active```.
2. Similarly, open a new terminal to test ROS2, ```source ~/.bashrc```, then test the alias for ros2 by typing ```foxy```, the following message should show up, ```Foxy (ROS2) is active```.

### PC2 (Ubuntu 22.04)
1. Install ROS Humble following the steps in the following [link](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html).

#### Test the bridge between ROS1 and ROS2 on the same PC (PC1 ubuntu 20.04)
##### Example 1: run the bridge and the example talker and listener
Open 4 terminals (4 Shells)
First we start a ROS 1 roscore:
```bash
# Terminal A (ROS 1 only):
source ~/.bashrc
noetic
roscore
```

***

-->
