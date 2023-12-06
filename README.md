# GPS Turtle Nav - Noetic
## Project Overview
<!-- ![Alt text]( "Overview") -->
### Intoduction
This project focuses on enabling a robot to navigate a map using only GPS coordinates for both its localization and waypoint guidance. By relying solely on GPS technology, the robot can accurately determine its position and follow predefined waypoints within the map.
## Usage (ROS1 Noetic)
### Prerequisites
- Ubuntu 20.04
- ROS noetic
```shell
sudo apt-get install ros-noetic-hector-gazebo-plugins
pip3 install pyproj
echo "export TURTLEBOT3_MODEL=waffle" >> ~/.bashrc
```
clone this repo \
for example:
```shell
cd ~
git clone https://github.com/eladpar/turtlebot3_gps_waypoint.git turtlebot3_gps_waypoint_ws
```
please compile in the root directory of the project :
```shell
catkin_make
```
for example:
```shell
cd ~/turtlebot3_gps_waypoint_ws # this is the root of the project
catkin_make
```
### Launching
please use 3 terminals:\
**assuming youre in the root of the project**\
Terminal 1:
```shell
source devel/setup.bash
roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch
```
Terminal 2:
```shell
source devel/setup.bash
python3 GPSwaypointNav.py
```
Terminal 3 (launching the seperate gui):
```shell
source /opt/ros/noetic/setup.bash
python3 gui/main.py
```
## DIY
### inserting gps module into a model urdf
here we have a joint added to base link and another gps link add on the other side of the joint,\
after adding the gps link we add a pluging from the library hector-gazebo-plugins we installed.
```xml
  <joint name="gps_joint" type="fixed">
    <parent link="base_link"/>
    <child link="base_gps"/>
    <origin xyz="-0.032 0 0.172" rpy="0 0 0"/>
  </joint>

  <link name="base_gps">
    <visual>
      <origin xyz="0 0 0.0" rpy="0 0 0"/>
      <geometry>
        <box size="0.01 0.01 0.01" />
      </geometry>
        <material name="Cyan">
          <color rgba="0 1.0 1.0 1.0"/>
        </material>
    </visual>

  </link>

    <gazebo>
    <plugin name="gazebo_ros_gps" filename="libhector_gazebo_ros_gps.so">
      <alwaysOn>1</alwaysOn>
      <updateRate>10.0</updateRate>
      <bodyName>base_gps</bodyName>
      <topicName>robot_location</topicName>
      <velocityTopicName>fix_velocity</velocityTopicName>

      <referenceLatitude>32.072734</referenceLatitude>
      <referenceLongitude>34.787465</referenceLongitude>
    </plugin>
   </gazebo>
``` 
more documentation can be found at GazeboRosGps: http://wiki.ros.org/hector_gazebo_pluginss

### controlling and moving the robot
The main command used to move and control the robot is /cmd_vel that is subscribed by\
diffdrivecontroller and gets twist msgs (angular and linear speeds)
using the heading you can derive from 2 gps points,\
you can make the robots yaw orientation get to the same angle and then drive to that direction.

