# Safe Dynamic Motion Planning or Dynamic Lattice Planning 

<img src="pictures/problem_enhanced2.png" alt="Alt Text" width="700" height="500">

Image: Crosswalk3 scenario where DJI100 navigates through a crowd of dynamic obstacles. 

## Information
This project extends the Receding-Horizon Lattice Planner (RHLP) [[1]](https://ieeexplore.ieee.org/document/8618964)[[2]](https://ieeexplore.ieee.org/document/9385931) with new improvements to handle the edge cases in the dynamic environment. 

This new planner, Safe Lattice Planner (SLP) is evaluated in the provided benchmark against the baselines RRT*, Temporal RRT* and RHLP. This repository contains the benchmark, the baselines and SLP. This has been built in a Docker environment.

For more information, please see our [paper](liu.se).

## System Requirements
System requirements:
- Docker version 27.5.1, build 9f9e405
- OS: Ubuntu 22.04.5 LTS
- Kernel: 6.8.0-52-generic
- Driver Version: 535.183.01   CUDA Version: 12.2 (Dual NVIDIA GeForce RTX 2080 Ti)
- nvidia-container-toolkit: 1.14.3-1

## Installation and execution
TODO
Please see the [Wiki](liu.se).

## Showcase
TODO

## Credits
If you find this work useful, please cite our paper.

*Safe Lattice Planning for Motion Planning with Dynamic Obstacles*
```
[INSERT CITATION]
```

This work has been developed by Emil Wiman and Mattias Tiger with the help and support of the members at the [Division of Artificial Intelligence and Integrated Computer Systems](https://liu.se/en/organisation/liu/ida/aiics) at [Linköping University](https://liu.se/en).

-----------------------------------------------------------------------------------------------

This work builds upon the work from two previous papers, they are listed here for completeness.

*Enhancing Lattice-Based Motion Planning With Introspective Learning and Reasoning*
```
@ARTICLE{9385931,
  author={Tiger, Mattias and Bergström, David and Norrstig, Andreas and Heintz, Fredrik},
  journal={IEEE Robotics and Automation Letters}, 
  title={Enhancing Lattice-Based Motion Planning With Introspective Learning and Reasoning}, 
  year={2021},
  volume={6},
  number={3},
  pages={4385-4392},
  keywords={Planning;Safety;Collision avoidance;Trajectory;Dynamics;Uncertainty;Lattices;Motion and path planning;collision avoidance},
  doi={10.1109/LRA.2021.3068550}}
```

*Receding-Horizon Lattice-Based Motion Planning with Dynamic Obstacle Avoidance*
```
@INPROCEEDINGS{8618964,
  author={Andersson, Olov and Ljungqvist, Oskar and Tiger, Mattias and Axehill, Daniel and Heintz, Fredrik},
  booktitle={2018 IEEE Conference on Decision and Control (CDC)}, 
  title={Receding-Horizon Lattice-Based Motion Planning with Dynamic Obstacle Avoidance}, 
  year={2018},
  volume={},
  number={},
  pages={4467-4474},
  keywords={Planning;Vehicle dynamics;Dynamics;Lattices;Trajectory;Navigation;Real-time systems},
  doi={10.1109/CDC.2018.8618964}}
```




## Contact
For questions and remarks please contact the developers by supplying an issue.

