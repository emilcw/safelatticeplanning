cd
source .bashrc

location_tolerance=$1
dji_safety_radius=$2

roslaunch rrtplanner rrtplanner.launch namespace:=dji0 location_tolerance:=$location_tolerance dji_safety_radius:=$dji_safety_radius