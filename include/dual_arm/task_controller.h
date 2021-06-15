#pragma once
#ifndef __TASKCONTROLLER_H
#define __TASKCONTROLLER_H

#include <iostream>
#include <eigen3/Eigen/Dense>
#include "custommath.h"
#include "robotmodel.h"

using namespace std;
using namespace Eigen;

class CTaskController
{
    public:
	CTaskController(int jdof, double dt, const double touch_probe_position_rad[]);
	virtual ~CTaskController();

    void get_joint_position_offset(double joint_position_offset[]);
    void read(double time, double joint_position[], double joint_velocity[], double joint_force[]);
    void write(double torque_desired[]);
    void compute();
    
    int _jdof;

    private:
    void initialize();
    void model_update();

    double _dt;
    double _time;
    VectorCXd _q, _qdot, _torque; //state
    VectorCXd _q_des, _qdot_des, _torque_des; //reference
    VectorCXd _q_offset, _q_touch_probe;
    
    CModel Model; //robot model

    MatrixCXd _J_hands; // 12x15
	MatrixCXd _J_T_hands; // 15x12
	MatrixCXd _Jdot_hands;
	MatrixCXd _pre_J_hands;
	MatrixCXd _pre_Jdot_hands;
	VectorCXd _Jdot_qdot;

	MatrixCXd _J_pos_hands; // 6x15
	MatrixCXd _J_pos_T_hands; // 15x6
	MatrixCXd _J_ori_hands; // 6x15
	MatrixCXd _J_ori_T_hands; // 15x6
};

#endif