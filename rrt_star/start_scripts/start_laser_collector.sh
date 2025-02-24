cd
source .bashrc

# Taken from dev_env.sh
MAX_SCANS=4000

roslaunch collision_avoidance_m100_gazebo laser_assembly.launch max_scans:=${MAX_SCANS} namespace:=dji0