Kuon
=============

A collection of [ROS](http://ros.org) packages for the Kuon family of rugged mobile Robotic platforms.

![Kuon Rugged Mobile Robotic Platform](http://www.roadnarrows.com/r-and-d/Kuon/img/Kuon_Reflect.png)

Learn more about Kuon on the [RoadNarrows R&D - Kuon](http://roadnarrows.com/r-and-d/Kuon/) page.

Visit the [RoadNarrows Store](http://www.roadnarrows-store.com/products/move/mobile-bases/outdoor-bases/kuon.html)

#Quick Start:
The kuon ROS packages were developed under:
 * _Ubuntu 12.04 64-bit_
 * _ROS Groovy Galapagos_ 

To get started with your mobile robotic platform, or to take virtual Kuon for a test spin:
* Install Ubuntu 12.04 (Long Term Support LTS) 64-Bit:
  * [ubuntu.com](http://www.ubuntu.com/download/desktop)
* Install ROS on your system as described here: 
  * [http://www.ros.org/wiki/groovy/Installation/Ubuntu](http://www.ros.org/wiki/groovy/Installation/Ubuntu)
* Be sure to install utilities for the new _catkin_ build system, and to source the ROS configuration file:
  * sudo apt-get install python-wstool
  * sudo source /opt/ros/groovy/setup.bash
* It is helpful to understand the basics about ROS before moving on. Follow some of the tutorials here:
  * [http://www.ros.org/wiki/ROS/Tutorials](http://www.ros.org/wiki/ROS/Tutorials)
* Install required RoadNarrows library dependencies:
  * TODO(dhp) - add instructions for roadnarrows apt repo)
* Create a catkin workspace somewhere on your system (e.g. in your home directory) and add the Kuon ROS packages to the workspace:
  * mkdir ~/catkin_ws/src
  * cd ~/catkin_ws/src
  * catkin_init_workspace
  * wstool init
  * wstool set kuon --git http://github.com/roadnarrows-robotics/kuon
* Build the kuon ROS packages:
  * cd ~/catkin_ws
  * catkin_make
* Source devel/setup.bash to add this workspace to your ROS search paths:
  * source devel/setup.bash
* Try some of the examples:
  * sim
  * minimal launch
  * ??? Chess ??? Dance ???
* Contribute!
  * When developing your own software for Kuon, we recommend forking the official github repository. If you develop some new application or functionality, we would greatly appreciate notifying RoadNarrows by issuing a "pull request"



