cd
source .bashrc

location_tolerance=$1

roslaunch rrtplanner rrtplanner.launch namespace:=dji0 location_tolerance:=$location_tolerance