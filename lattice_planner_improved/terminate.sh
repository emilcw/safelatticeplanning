cd
source .bashrc
rosnode kill -a
killall -9 gzserver
killall -9 gzclient
killall -9 rosmaster
exit
