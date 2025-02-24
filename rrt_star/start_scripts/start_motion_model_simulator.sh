cd
source .bashrc
x=$1
y=$2
z=$3
roslaunch motion_models motion_sim_matrice_m100.launch namespace:=dji0 x:=$x y:=$y z:=$z