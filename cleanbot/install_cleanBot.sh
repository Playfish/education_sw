#!/bin/sh
sudo python scripts/change_dash_to_bash.py

tar -xvf gazebo_models.tar -C ~/.gazebo

workspace=~/catkin_ws_test
echo "Create workspace."
mkdir -p "$workspace/src"
echo "Begin install cleanBot."
tar -xvf src.tar.gz -C $workspace/src

cd $workspace
rosdep install --rosdistro indigo --from-paths src -y

echo "Complie..."
catkin_make

echo "source $workspace/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc

echo -e  "\033[;;5m All work is finished. Now you can run run_gazebo_demo.sh \033[0m"
