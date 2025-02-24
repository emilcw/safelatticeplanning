cd
source .bashrc
location_tolerance=$1
dji_safety_radius=$2
adaptive_planning=$3
survival_planning=$4
emergency_trajectories=$5

roslaunch lattice_planner lattice_planner.launch namespace:=dji0 location_tolerance:=$location_tolerance  dji_safety_radius:=$dji_safety_radius adaptive_planning:=$adaptive_planning survival_planning:=$survival_planning emergency_trajectories:=$emergency_trajectories