cd
source .bashrc
RESOLUTION=0.2
MAX_RANGE=1000
map="granso.bt" #Needs to be fixed later.

roslaunch collision_avoidance_m100_gazebo octomap_mapping_load_map.launch resolution:=${RESOLUTION} max_range:=${MAX_RANGE} map:=${map}