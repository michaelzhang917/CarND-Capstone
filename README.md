# Final Project of Udacity Self Driving Car Course 
**System Integration for a Real Self-Driving Car** 

[//]: # (Image References)
[image1]: ./imgs/final-project-ros-graph-v2.png
[image2]: ./imgs/tl-detector-ros-graph.png
[image3]: ./imgs/waypoint-updater-ros-graph.png
[image4]: ./imgs/dbw-node-ros-graph.png

## Introduction 

The goal of this project is to integrate the perception, planning and control software subsystems for a provided Udacity car (called "Carla") so that the car can drive around a given test track using a waypoint navigation. Carla is an autonomous Lincoln MKZ, at Udacity's test site in Palo Alto, California. Since our team members come from different places in the world, we use a simulator during our development which is provided by Udacity. 

The provided car has these hardware specifications:

* 31.4 GiB Memory
* Intel Core i7-6700K CPU @ 4 GHz x 8
* TITAN X Graphics
* 64-bit OS

The following is a system architecture diagram showing the ROS nodes and topics used in the project. 

![architecture][image1]

There are three core functionalities of the system, including preception, planning and control.

1. The perception subsystem contains obstacle and traffic light detection. The detection provides a traffic light color detection so that the car knows when to stop or drive if the car approaches an intersection with a traffic light.

2. In the planning subsystem, we implement a waypoint updater for updating the next waypoint depending on the desired behavior. 

3. The throttle, break, and steering of the car are actuated by the control subsystem. 

## The Team

Our team  _What's In a Name? (WIN)_ is composed of the following course participants:

<table>
  <tr>
    <th>Name</th>
    <th>Role / Responsibility</th>
  </tr>
  <tr>
    <td>Helio Perroni Filho</td>
    <td>Team Leader</td>
  </tr>
  <tr>
    <td>Ian Zhang</td>
    <td>Drive-By-Wire (DBW)</td>
  </tr>
  <tr>
    <td>Lucas Souza</td>
    <td>Traffic Sign Detector</td>
  </tr>
  <tr>
    <td>Shay Koren</td>
    <td>Waypoint Updater</td>
  </tr>
  <tr>
    <td>Yong Zhang</td>
    <td>Documentation and Integration Tests</td>
  </tr>
</table>

## Manual
### Installation

* Be sure that your workstation is running Ubuntu 16.04 Xenial Xerus or Ubuntu 14.04 Trusty Tahir. [Ubuntu downloads can be found here](https://www.ubuntu.com/download/desktop).
* If using a Virtual Machine to install Ubuntu, use the following configuration as minimum:
  * 2 CPU
  * 2 GB system memory
  * 25 GB of free hard drive space

  The Udacity provided virtual machine has ROS and Dataspeed DBW already installed, so you can skip the next two steps if you are using this.

* Follow these instructions to install ROS
  * [ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu) if you have Ubuntu 16.04.
  * [ROS Indigo](http://wiki.ros.org/indigo/Installation/Ubuntu) if you have Ubuntu 14.04.
* [Dataspeed DBW](https://bitbucket.org/DataspeedInc/dbw_mkz_ros)
  * Use this option to install the SDK on a workstation that already has ROS installed: [One Line SDK Install (binary)](https://bitbucket.org/DataspeedInc/dbw_mkz_ros/src/81e63fcc335d7b64139d7482017d6a97b405e250/ROS_SETUP.md?fileviewer=file-view-default)
* Download the [Udacity Simulator](https://github.com/udacity/self-driving-car-sim/releases/tag/v0.1).

### Usage

1. Clone the project repository
```bash
git clone https://github.com/udacity/carnd_capstone.git
```

2. Install python dependencies
```bash
cd carnd_capstone
pip install -r requirements.txt
```
3. Make and run styx
```bash
cd ros
catkin_make
source devel/setup.sh
roslaunch launch/styx.launch
```
4. Run the simulator
```bash
unzip lights_no_cars.zip
cd lights_no_cars
chmod +x ros_test.x86_64
./ros_test.x86_64
```

### Real world testing
1. Download [training bag](https://drive.google.com/file/d/0B2_h37bMVw3iYkdJTlRSUlJIamM/view?usp=sharing) that was recorded on the Udacity self-driving car (a bag demonstraing the correct predictions in autonomous mode can be found [here](https://drive.google.com/open?id=0B2_h37bMVw3iT0ZEdlF4N01QbHc))
2. Unzip the file
```bash
unzip traffic_light_bag_files.zip
```
3. Play the bag file
```bash
rosbag play -l traffic_light_bag_files/loop_with_traffic_light.bag
```
4. Launch your project in site mode
```bash
cd CarND-Capstone/ros
roslaunch launch/site.launch
```

## Code Structure and Implementation

Below is a brief overview of the repo structure, along with descriptions of the ROS nodes. We implenment our system integration in (path_to_project_repo)/ros/src/ directory. Within this directory, you will find the following ROS packages.

### Traffic Light Detection Package

This package is implenmented in (path_to_project_repo)/ros/src/tl_detector/ .

The package contains the traffic light detection node: tl_detector.py. This node takes in data from the /image_color, /current_pose, and /base_waypoints topics and publishes the locations to stop for red traffic lights to the /traffic_waypoint topic.

The /current_pose topic provides the vehicle's current position, and /base_waypoints provides a complete list of waypoints the car will be following.

We build both a traffic light detection node and a traffic light classification node. Traffic light detection is implenmented in tl_detector.py, while traffic light classification is built in ../tl_detector/light_classification_model/tl_classfier.py. A tensorflow based classification model is trained for the light classification and stored in ../tl_detector/light_classification/model_data/ .

![traffic_light][image2]

### Waypoint updater package

This package is implenmented in (path_to_project_repo)/ros/src/waypoint_updater/ .

The package contains the waypoint updater node: waypoint_updater.py. The purpose of this node is to update the target velocity property of each waypoint based on traffic light and obstacle detection data. This node will subscribe to the /base_waypoints, /current_pose, /obstacle_waypoint, and /traffic_waypoint topics, and publish a list of waypoints ahead of the car with target velocities to the /final_waypoints topic.

![waypoint_updater][image3]

### Drive-by-Wire (DBW) Package 

Carla is equipped with a drive-by-wire (dbw) system, meaning the throttle, brake, and steering have electronic control. This package contains the files that are responsible for control of the vehicle: the node dbw_node.py and the file twist_controller.py, along with a PID controller that we use in ourour implementation. The dbw_node subscribes to the /current_velocity topic along with the /twist_cmd topic to receive target linear and angular velocities. Additionally, this node will subscribe to /vehicle/dbw_enabled, which indicates if the car is under dbw or driver control. This node will publish throttle, brake, and steering commands to the /vehicle/throttle_cmd, /vehicle/brake_cmd, and /vehicle/steering_cmd topics.

![DBW][image4]

In addition, the following styx and styx_msgs packages are used to provide a link between the simulator and ROS, and to provide custom ROS message types:

(path_to_project_repo)/ros/src/styx/
A package that contains a server for communicating with the simulator, and a bridge to translate and publish simulator messages to ROS topics.
(path_to_project_repo)/ros/src/styx_msgs/
A package which includes definitions of the custom ROS message types used in the project.
(path_to_project_repo)/ros/src/waypoint_loader/
A package which loads the static waypoint data and publishes to /base_waypoints.
(path_to_project_repo)/ros/src/waypoint_follower/
A package containing code from Autoware which subscribes to /final_waypoints and publishes target vehicle linear and angular velocities in the form of twist commands to the /twist_cmd topic.
