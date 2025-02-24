cd
source .bashrc
location_tolerance=$1

roslaunch lattice_planner lattice_planner.launch namespace:=dji0 location_tolerance:=$location_tolerance