**Pathplanning ROS node for ground robot**
================================================================================================================================
# Installing prerequisites
### 1. Install ROS Kinetic

```bash
$ sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
```
```bash
$ sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
```
```bash
$ sudo apt-get update
```
```bash
$ sudo apt-get install ros-kinetic-desktop-full
```
```bash
$ sudo rosdep init
$ rosdep update
```
```bash
$ echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
$ source ~/.bashrc
```
### 2. Initialize catkin workspace

```bash
$ mkdir -p {custom_path}/catkin_ws/src
$ cd {custom_path}/catkin_ws/src
$ catkin_init_workspace
$ cd ..
$ catkin_make
$ echo "source {custom_path}/catkin_ws/devel/setup.bash" >> ~/.bashrc
$ source ~/.bashrc
```
# Clone repository
### 1. Optional: Install git
```bash
$ sudo apt install git
```
### 2. Clone repository
```bash
$ cd {custom_path}/catkin_ws/src
$ git clone -b v0.1b https://github.com/avbokovoy/pathplanning_tb.git
```
# Install dependencies
```bash
$ cd {custom_path}/catkin_ws
$ rosdep install --from-paths src --ignore-src -r -y
```

# Make package
```bash
$ cd {custom_path}/catkin_ws
$ catkin_make --pkg pathplanning_tb
```
# Run package 
```bash
$ roslaunch pathplanning_tb test.launch
```
