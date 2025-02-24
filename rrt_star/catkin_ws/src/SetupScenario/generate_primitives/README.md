Author: Oskar Ljungqvist
Date: 2017-11-23
Description: TDDO!

This folder contains code for ger generate an MPC controller using ACADO toolkit.

The file nmpc_solver_setup.cpp is what defines the MPC controller and it is this you should change in order the change the controller. Then build/install it using the explanation below.

Prerequests: 

ACADO toolkit needs to be properly installed. Follow the instructions here: http://acado.github.io/install_linux.html

In your ~/.bashrc add the line:
source [INSTALLATION_PATH_ACADO]/build/acado_env.sh

Build/Install: 

From this folder:

mkdir build
cd build
cmake ..
make

cd ..
./nmpc_solver_setup

Then the controller is generated.



