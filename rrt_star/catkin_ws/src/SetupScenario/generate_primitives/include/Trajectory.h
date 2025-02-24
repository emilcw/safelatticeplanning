#pragma once
#ifndef __TRAJECTORY_H__
#define __TRAJECTORY_H__

#include <list>
#include <math.h>
#include <iostream>
#include "State.h"
using namespace std;


class Trajectory
{
public:
  int ID = 0;
	list<State> stateList;

  double getLength();
  
  double getSquaredLength();


	void print(const char* fileName);

    Trajectory() = default;

	~Trajectory(){}

	State getEndState() {
		if (!stateList.empty())
			return stateList.back();
		else
			return State();
	}
};

#endif //__TRAJECTORY_H__
