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
    <th> Udacity Email</th>
  </tr>
  <tr>
    <td>Helio Perroni Filho</td>
    <td>Team Leader</td>
    <td>xperroni@gmail.com</td>
  </tr>
  <tr>
    <td>Ian Zhang</td>
    <td>Drive-By-Wire (DBW)</td>
    <td>ianboyanzhang@gmail.com</td>
  </tr>
  <tr>
    <td>Lucas Souza</td>
    <td>Traffic Sign Detector</td>
    <td>lucasosouza@gmail.com</td>
  </tr>
  <tr>
    <td>Shay Koren</td>
    <td>Waypoint Updater</td>
    <td>shaykoren18@gmail.com</td>
  </tr>
  <tr>
    <td>Yong Zhang</td>
    <td>Documentation and Integration Tests</td>
    <td>michaelzhang917@gmail.com</td>
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

1. Location: (path_to_project_repo)/ros/src/tl_detector/
2. Description: This package contains the traffic light detection node: tl_detector.py. This node takes in data from the /image_color, /current_pose, and /base_waypoints topics and publishes the locations to stop for red traffic lights to the /traffic_waypoint topic.

The /current_pose topic provides the vehicle's current position, and /base_waypoints provides a complete list of waypoints the car will be following.

You will build both a traffic light detection node and a traffic light classification node. Traffic light detection should take place within tl_detector.py, whereas traffic light classification should take place within ../tl_detector/light_classification_model/tl_classfier.py.

![traffic_light][image2]
