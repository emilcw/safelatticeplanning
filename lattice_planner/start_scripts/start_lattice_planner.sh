cd
source .bashrc
location_tolerance=$1
dji_safety_radius=$2
use_only_geometric_search=$3
use_stand_still=$4
use_predictions=$5
use_geometric_secondary_search=$6

roslaunch lattice_planner lattice_planner.launch namespace:=dji0 location_tolerance:=$location_tolerance  dji_safety_radius:=$dji_safety_radius use_only_geometric_search:=$use_only_geometric_search use_stand_still:=$use_stand_still use_predictions:=$use_predictions use_geometric_secondary_search:=$use_geometric_secondary_search