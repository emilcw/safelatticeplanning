cd
source .bashrc
x=$1
y=$2
z=$3
name=$4
rostopic pub /dji0/nav_goal geometry_msgs/PoseStamped "{
header: {
    stamp: now,
    frame_id: 'world'
},
pose: {
    position: {x: $x, y: $y, z: $z},
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
  }
}"
