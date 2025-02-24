# Safe Dynamic Motion Planning or Dynamic Lattice Planning

![Alt text](pictures/image.png)

## Information
This project investigates the Lattice Planner first developed by [[1]](https://ieeexplore.ieee.org/document/8618964) and then enhanced in [[2]](https://ieeexplore.ieee.org/document/9385931). This project consists of three contributions

- A new benchmark to compare dynamic motion planners
- A comparison of the proposed Lattice Planner in the benchmark against temporal RRT*
- Proposing the Lattice Planner as a novel method to perform dynamic and temporal motion planning.

## System Requirements
TODO

## Installation and execution
Currently the system can be started in two ways, where Option 1 is tedious and Option 2 is nice.
- Clone the repository
- Set up [Rootless docker mode](https://docs.docker.com/engine/security/rootless/).

#### Option 1
- Use the **dev_env.sh** script to start the docker environment as described below and 
```
./dev_env.sh start lattice_planner

# Create 11 terminals with the command below
./dev_env.sh bash lattice_planner 

# In each terminal 
cd start_scripts

# Then run following commands once in the terminals to start all the ROS nodes

./catkin_build.sh
./start_lattice_planner
./start_m100_simulator_interface.sh [world] [mode] (inget ger cafe_static)
./spawn_dji100.sh [x] [y] [z] [name] 
(Genom att ändra namn kan du spawna in flera drönare på flera olika positioner genom att bara köra om scripet, default (0,0,1) och "dji0")
./start_motion_model_simulator.sh
./start_mpc.sh
./start_converter_djisdk_mav_node.sh
./start_octomap_ground_truth.sh 
(Optional, only displays ground truth and gets coverage)
./start_octomap.sh
./start_laser_collector.sh
./start_gazebo2rviz.sh
./start_rviz.sh
```

#### Option 2
- Modify the sdmp_paramters.yaml scrips as desired

```

#If you have modified the code, run this to compile
./dev_env.sh make lattice_planner

# To start the simulation loop
python3 sdmp.py
```


## Showcase
TODO

## Credits
This work has been developed by Emil Wiman with the help and support of the members at the [Division of Artificial Intelligence and Integrated Computer Systems](https://liu.se/en/organisation/liu/ida/aiics) at [Linköping University](https://liu.se/en).

## Contact
For questions and remarks please contact the developers.

