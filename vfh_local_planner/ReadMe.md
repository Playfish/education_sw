# vfh_local_planner
==================

VFH+ algorithm for obstacle avoidance implementation in ROS.
Reference:
```
Ulrich, I.; Borenstein, J., "VFH+: reliable obstacle avoidance for fast mobile robots," 
Robotics and Automation, 1998.
```

## Preparation

 * Based on Turtlebot, in simulation, put ```param/vfh_local_planner_params.yaml``` into ```turtlebot_navigation/param```.

 * Modify ```move_base.launch.xml``` in ```turtlebot_navigation```:
```
    <rosparam file="$(find turtlebot_navigation)/param/dwa_local_planner_params.yaml" command="load" />
```
into 
```
    <rosparam file="$(find turtlebot_navigation)/param/vfh_local_planner_params.yaml" command="load"/>
```

## Usage

In simulation, run under command:
```
roslaunch turtlebot_gazebo turtlebot_world.launch
roslaunch turtlebot_gazebo amcl_demo.launch
roslaunch turtlebot_rviz_launchers view_navigation.launch
```
Choose ```2D Nav Go```, and then checkout path of vfh_local_planner.

## Parameters

All parameters in vfh_local_planner is ```vfh_local_planner_params.yaml```.
