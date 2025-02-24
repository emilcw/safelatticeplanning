cd
source .bashrc

test=$1
scenario=$2
scenario_parameter=$3
location_tolerance=$4
dji_safety_radius=$5

roslaunch setup_scenario scenario.launch namespace:=dji0 test:=$test scenario:=$scenario scenario_parameter:=$scenario_parameter location_tolerance:=$location_tolerance dji_safety_radius:=$dji_safety_radius