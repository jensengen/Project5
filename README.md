# Home Service Robot Project
This repository contains the code for the UDACITY project Home Service Robot.

## Packages
The Project relies on official ROS packages as well as custom packages developed for this project.

### Official ROS Packages
`gmapping`: This package performs SLAM and build a map of the environment with a robot equipped with laser range finder sensors or RGB-D cameras.
http://wiki.ros.org/gmapping

`turtlebot_teleop`: This package provides teleoperation to manually control a robot using keyboard commands.
http://wiki.ros.org/turtlebot_teleop

`turtlebot_rviz_launchers`: This package contains launchers for visualizing TurtleBot in rviz.
http://wiki.ros.org/turtlebot_rviz_launchers

`turtlebot_gazebo`: This package contains Gazebo launchers for a TurtleBot simulation.
http://wiki.ros.org/turtlebot_gazebo

### Custom Packages
`pick_objects`: This package communicates with the ROS navigation stack and autonomously send successive goals for the robot to reach. The first goal is a pickup location
 and the second goal is a drop-off location. 
 
`add_markers`: This package displays virtual object with markers in rviz. The package has two flavors which can be selected by a parameter called robot_tracking (configured in launch file)

With parameter `robot_tracking=0` the package performs the following actions:
* Publish the marker at the pickup zone
* Pause 5 seconds
* Hide the marker
* Pause 5 seconds
* Publish the marker at the drop off zone

With parameter `robot_tracking=1` the package performs the following actions:
* Initially show the marker at the pickup zone
* Hide the marker once your robot reaches the pickup zone
* Wait 5 seconds to simulate a pickup
* Show the marker at the drop off zone once your robot reaches it

## Python
The UDACITY workspace is setup to use Python 3 while the ROS-Kinetic distribution requires Python 2. To correctly launch and run this project, make sure to use Python 2.

For the UDACITY workspace, a work-around is described here:
https://github.com/PenguinLemma/udacity-robond-p5/blob/master/doc/UDACITY_WS_SETUP.md

## Scripts
The folder /src/scripts/ contains a number of scripts to launch and configure all packages.. A short description of the scripts is below. 

`test_slam.sh`
Launches the packages for manual SLAM testing. Robot is operated by teleop

`test_navigation.sh`
Launches the packages for navigation testing. Robot goal is set though RVIZ.

`pick_objects.sh`
Launches the packages for testing of pick_objects node. pick_objects publishes navigation goals for the robot

`add_marker.sh`
Launches the packages for displaying virtual objects in RVIZ. 

`home_service.sh`
Launches all packages for the project. 

All scripts should be run from catkin_ws like this:

`./src/scripts/script_name.sh` 

Before running a script, make sure to build the workspace and source the setup script
source devel/setup.bash

As mentioned above make sure to run scripts from a Python 2 enabled terminal window.
