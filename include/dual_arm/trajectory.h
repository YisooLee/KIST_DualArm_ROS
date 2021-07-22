#pragma once
#ifndef __TRAJECTORY_H
#define __TRAJECTORY_H

#include <iostream>
#include <eigen3/Eigen/Dense>
#include "custommath.h"
using namespace std;
using namespace Eigen;

class CTrajectory
{

public:
	CTrajectory();
	virtual ~CTrajectory();
		
	int _vector_size;
	double _time_start, _time, _time_end;	
	VectorCXd _init_pos, _init_vel, _goal_pos, _goal_vel;	
	void set_size(int dof);
	void reset_initial(double time0, VectorCXd init_pos, VectorCXd init_vel);
	void update_time(double time);
	void update_goal(VectorCXd goal_pos, VectorCXd goal_vel, double goal_time);
	int check_trajectory_complete();
	VectorCXd position_cubicSpline();
	VectorCXd velocity_cubicSpline();

private:
	void Initialize();
	void check_vector_size(VectorCXd X);
	bool _bool_trajectory_complete;
	double _motion_threshold;
};

#endif