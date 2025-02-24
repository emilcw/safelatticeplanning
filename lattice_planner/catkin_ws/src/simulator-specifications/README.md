# DJI m100 gazebo simulator for sense and avoid for collision avoidance applications.

This repository builds upon the implementation presented in the following [technical report](https://www.ida.liu.se/~matti23/mattisite/research/WASP-2018-Towards-RT-Motion-Planning-and-Collision-Avoidance.pdf). Please cite the technical report as
```
@techreport{andersson2018,
     title = {{Towards Real-Time Integrated Motion Planning and Collision Avoidance for the Matrice 100 Quadcopter}},
     author = {Olov Andersson and Oskar Ljungqvist and Mattias Tiger},
     year = {2018},
     institution = {Division of Artificial Intelligence and Integrated Computer Systems and Division of Automatic Control, Link{\"o}ping University
},
     month = {10},
     url = "https://www.ida.liu.se/~matti23/mattisite/research/WASP-2018-Towards-RT-Motion-Planning-and-Collision-Avoidance.pdf",
}

```

=========

A ROS package for simulating the DJI Matrice 100 Drone in Gazebo for collision avoidance applications.


Dependencies: hku_m100_description 

### A sample scene with a virtual LIDAR mounted on a DJI M100 quadcopter
![](img/virtual_sensors_gazebo_scene.png)
### A real-world 3D environement (*Gränsö*)
![](img/granso_gazebo.png)
### A virtual DJI M100 quadcopter flying around in *Gränsö*
![](img/granso_rviz_gazebo.png)

=========

### 1) Start empty scenario in gazebo

`roslaunch collision_avoidance_m100_gazebo start.launch`

### 2) Spawn m100 platform

`roslaunch collision_avoidance_m100_gazebo spawn_m100.launch name:=drone001`

### 3) Start interface for external control of m100 platform

`roslaunch collision_avoidance_m100_gazebo start_m100_interface.launch name:=drone001`

### 4) Start path generator for drone001
#### * 3D
`roslaunch collision_avoidance_m100_gazebo path_generator.launch name:=drone001 motion:="path" arg:='{path: [[0,0,1], [10,10,10], [2,5,5]], vel: 10, is2D: false}'`
#### * 2D with fixed height (z value)
`roslaunch collision_avoidance_m100_gazebo path_generator.launch name:=drone001 motion:="path" arg:='{path: [[0,0], [10,10], [2,5]], vel: 10, is2D: true, z: 5}'`
#### * Single run
`roslaunch collision_avoidance_m100_gazebo path_generator.launch name:=drone001 motion:="path" arg:='{path: [[0,0], [10,10], [2,5]], vel: 10}'`
#### * Reapeted run
`roslaunch collision_avoidance_m100_gazebo path_generator.launch name:=drone001 motion:="path" arg:='{path: [[0,0], [10,10], [2,5]], vel: 10, repeat: true}'`
#### * Patrol run (back and forth)
`roslaunch collision_avoidance_m100_gazebo path_generator.launch name:=drone001 motion:="path" arg:='{path: [[0,0], [10,10], [2,5]], vel: 10, patrol: true}'`
#### * Polygon run (last point connect to the first point)
`roslaunch collision_avoidance_m100_gazebo path_generator.launch name:=drone001 motion:="path" arg:='{path: [[0,0], [10,10], [2,5]], vel: 10, polygon: true}'`
