source /opt/ros/melodic/setup.bash
source catkin_ws/devel/setup.bash
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/gazebo_models/
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/catkin_ws/src/simulator-specifications/models

function set-title() {
  if [[ -z "$ORIG" ]]; then
    ORIG=$PS1
  fi
  TITLE="\[\e]2;$*\a\]"
  PS1=${ORIG}${TITLE}
}

if [ ! -z "$SET_TITLE" ]; then
    set-title $SET_TITLE;
    export SET_TITLE=;
fi
