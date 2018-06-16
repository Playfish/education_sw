# Graduation Project for AR

Graduation project for AR track which will let turtlebot tracking AR and follow the first.

## Dependency

| System  | Version |
|---------|---------|
| Ubuntu  | 16.04   |
| ROS     | Kinetic |

Run under command for dependencies based on Turtlebot:
```
sudo  apt-get  install  ros-kinetic-ar-track-alvar
```

## Compilation
Extract ```src.tar.gz``` into ```your catkin workspace```, such as ```catkin_ws```:
```
cd ~/catkin_ws/src
tar -zxvf src.tar.gz
cd ~/catkin_ws
catkin_make
```

## Simulaton
Put Gazebo models into ```~/.gazebo/models/``` from ```models``` directory.

Open terminal for launch gazebo:
```
roslaunch  turtlebot_gazebo turtlebot_ar_world.launch
```

Open terminal for ar tracker:
```
roslaunch  turtlebot_gazebo ar_tracker.launch
```

Open terminal for turtlebot ar follower:
```
roslaunch turtlebot_ar_follower follower.launch 
```

For view:
```
roslaunch  turtlebot_ar_follower view_ar.launch
```

## Real Robot
Open terminal for launch turtlebot:
```
roslaunch turtlebot_bringup minimal.launch
```

Open terminal for launch sensor:
```
roslaunch turtlebot_bringup 3dsensor.launch
```

Open terminal for ar tracker:
```
roslaunch  turtlebot_gazebo ar_tracker.launch  # Put an AR track in front of turtlebot.
```

Open terminal for turtlebot ar follower:
```
roslaunch turtlebot_ar_follower follower.launch 
```
