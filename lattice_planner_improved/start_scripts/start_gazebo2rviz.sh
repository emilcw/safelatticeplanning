cd
#source .bashrc
source /opt/ros/melodic/setup.bash
source catkin_ws/devel/setup.bash
export MESH_WORKSPACE_PATH=~/catkin_ws
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/catkin_ws/src/simulator-specifications/models/
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/gazebo_models/
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/catkin_ws/src/drone_gazebo/models

roslaunch gazebo2rviz gazebo2rviz.launch