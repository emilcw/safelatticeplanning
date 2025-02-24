#pragma once
#ifndef __GRID_POINT_H__
#define __GRID_POINT_H__

#include "Trajectory.h"
#include <string.h>
class GridPoint{

private:
    int ID;
    int group;
    State initialState;
    State finalState;
    Trajectory trajectory;
    string name;
    double cost;

public:
    
    GridPoint(State initialState, State finalState, string name, int ID, int group)
    {
        this->initialState = initialState;
        this->finalState   = finalState;
        this->name = name;
        this->ID = ID;
        this->group = group;
        this->cost = 0;
    }

    void setInitialState(double vx, double vy, double vz, double x, double y, double z);
    void setFinalState(double vx, double vy, double vz, double x, double y, double z);

    State getInitialState(){return this->initialState;}
    State getFinalState(){return this->finalState;}

    int getID(){return this->ID;}
    void printSolution(const char* fileName);

    Trajectory* getRef2Traj(){return &this->trajectory;}

    void setCost(double opt_cost);
};

#endif //__GRID_POINT_H__
