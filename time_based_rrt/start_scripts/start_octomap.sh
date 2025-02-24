cd
source .bashrc
RESOLUTION=0.2
MAX_RANGE=1000
roslaunch collision_avoidance_m100_gazebo octomap_mapping.launch resolution:=${RESOLUTION} max_range:=${MAX_RANGE} namespace:=dji0