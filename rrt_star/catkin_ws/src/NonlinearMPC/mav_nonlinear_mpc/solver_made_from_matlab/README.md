// Original authors of this code is the ETH zurich group.

// Oskar note to self and others:

ACADO Toolkit needs to be properly installed via the matlab interface: http://acado.github.io/matlab_overview.html

This folder contains a automatically generated MPC controller.

The matlab script nmpc_solver_setup.m needs to be runned from matlab (2014b has been tested) on an Ubutu 14.04 (Ubutu 16.04 does not seem to work). 

nmpc_solver_setup.m generates the relevant c-code and it uses qpoases as QP-Solver.
