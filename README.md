# Service Robot
My final project for the Udacity Robotics Software Engineer Nanodegree 2019.

## Getting Started

I have used  [ROS](http://ros.org) Kinetic as this is used on the nanodegree. There are plenty of tutorials online and the [ROS Wiki](http://wiki.ros.org) is particularly useful. You will need a ROS workspace, either one you have currently or [create a new workspace](http://wiki.ros.org/catkin/Tutorials/create_a_workspace).
You will also need [RVIZ](http://wiki.ros.org/rviz/UserGuide) and [Gazebo](http://gazebosim.org/).

The project has several launch shell script that perform the required project requirements. Each shell script launches a number of xterm terminals for each of several different Ros packages used. These scripts should be run from the main package folder as they use the main package folder as the root to correctly locate the map and yaml files.

### Prerequisites
- An installed ROS system with rviz, gazebo.
- xterm installed `sudo apt-get install xterm`
- A ROS workspace

### Installing
You should [create a new Ros package](http://wiki.ros.org/ROS/Tutorials/CreatingPackage).

Copy/clone this project into your workspace/src directory.

Since there are a few package dependencies I have provided an install_pkgs.sh file that will clone the required Ros packages from github into the workspace/src directory and perform a catkin_make.

```
cd <your workspace>/src
sh scripts/install_pkgs.sh
```

## Usage

#### Requirement 1
```
sh scripts/test_slam.sh
```

#### Requirement 2
```
sh scripts/test_navigation.sh
```

#### Requirement 3
```
sh scripts/pick_objects.sh
```

#### Requirement 4
```
sh scripts/add_marker.sh
```

#### Requirement 5
```
sh scripts/home_service.sh
```

## Description of Packages Used
This project uses the following packages :-

The `ros-perception/slam_gmapping` package performs laser based SLAM, Simultaneous Location And Mapping. A 2D occupancy grid map is created using the laser data and turtlebot robot (`turtlebot` package) pose data as it moves through an environment, which the user can do using the `keyboard_teleop` package.
From a given map created by the SLAM package, the AMCL package is used to localise the robot within the environment using a particle filter. The `navigation` package can be given a goal and will use the odometry(from AMCL) and other sensor information to navigate to a given goal.
As usual the `RVIZ` package is used to visualise everything.

For this project's requirements, I created a marker_manager node that decouples and simplifies the action of managing markers that subscribes to requests to show or hide a marker at a given Point using a custom Ros message. The marker_manager is used by both the add_marker and home_service nodes to simplify them and remove the code overhead of them having to show and hide markers themselves.
