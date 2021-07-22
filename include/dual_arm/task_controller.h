#pragma once
#ifndef __TASKCONTROLLER_H
#define __TASKCONTROLLER_H

#include <iostream>
#include <eigen3/Eigen/Dense>
#include "custommath.h"
#include "robotmodel.h"
#include "trajectory.h"
#include "quadraticprogram.h"

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
    VectorCXd _q, _qdot, _torque; //state
    VectorCXd _qdot_lp; //qdot lowpass filter
    VectorCXd _q_des, _qdot_des, _torque_des, _qddot_ref; //reference

    VectorCXd _x_left_hand; // 6x1
	VectorCXd _x_right_hand; // 6x1
    VectorCXd _x_des_left_hand;
    VectorCXd _x_des_right_hand;

    private:
    void initialize();
    void model_update();

    double _dt;
    double _time;    
    VectorCXd _pre_q, _pre_qdot; //qdot lowpass filter
    VectorCXd _q_offset, _q_touch_probe, _q_home, _q_zero;
    
    CModel Model; //robot model

    int _control_mode;

    // variables for motion plan /////////////////////////////////////////////////////////////////////
    
    int _cnt_plan;        
    double _start_time, _end_time, _motion_time, _init_time;
    bool _bool_init, _bool_joint_motion, _bool_ee_motion;
    VectorCXi _bool_plan;
    VectorCXd _time_plan;    
    void motion_plan();

    //joint space motion
    VectorCXd _q_goal, _qdot_goal;
    CTrajectory JointTrajectory; //size = joint dof
    void reset_target(double motion_time);    
    void reset_target(double motion_time, VectorCXd target_joint_position);
    
    //operational space motion (two hand)
	VectorCXd _x_goal_left_hand;
	VectorCXd _xdot_goal_left_hand;
    VectorCXd _x_goal_right_hand;
	VectorCXd _xdot_goal_right_hand;
    Vector3d _pos_goal_left_hand;
	Vector3d _rpy_goal_left_hand;
	Vector3d _pos_goal_right_hand;
	Vector3d _rpy_goal_right_hand;
    
    CTrajectory RightHandTrajectory; //size = 6
	CTrajectory LeftHandTrajectory; //size = 6  

    void reset_target(double motion_time, Vector3d target_pos_lh, Vector3d target_ori_lh, Vector3d target_pos_rh, Vector3d target_ori_rh);

    ////////////////////////////////////////////////////////////////////////////////////////////////////

    // variables for controller  ///////////////////////////////////////////////////////////////////////
    
    //joint space control    
    VectorCXd _q_err, _qdot_err;
    MatrixCXd _A;
    double _kpj, _kvj;
    void jointControl(bool bool_offdiag);        

    //operational space control
    double _kp, _kv;    
	VectorCXd _xdot_left_hand; // 6x1
	VectorCXd _xdot_right_hand; // 6x1
    Vector3d _tmp_vec_ori;

	VectorCXd _xdot_des_left_hand;		
	VectorCXd _xdot_des_right_hand;
	Matrix3d  _R_des_left_hand;
	Matrix3d  _R_des_right_hand;

    Vector3d _x_err_left_hand;
	Vector3d _x_err_right_hand;
	Vector3d _xdot_err_left_hand;
	Vector3d _xdot_err_right_hand;
	Vector3d _R_err_left_hand;
	Vector3d _R_err_right_hand;
	Vector3d _Rdot_err_left_hand;
	Vector3d _Rdot_err_right_hand;

    MatrixCXd _A_inv;
    MatrixCXd _J_hands; // 12x15
	MatrixCXd _J_T_hands; // 15x12
    MatrixCXd _J_hands_inv; // 15x12
    MatrixCXd _J_T_hands_inv;// 12x15
	MatrixCXd _Lambda_hands; //inertia matri 12x12
    MatrixCXd _J_T_hands_inv_A; // 12x15
    MatrixCXd _J_T_hands_Lambda_hands; //15x12
    MatrixCXd _J_hands_A_inv; //12x15
    MatrixCXd _J_T_hands_Lambda_hands_J_hands_A_inv; //15x15
    MatrixCXd _Null_hands; //null space projection matrix 15x15	
	MatrixCXd _Id_15, _Id_12;
	VectorCXd _xddot_star; //12    
    VectorCXd _A_qddot_ref; //15
    
	MatrixCXd _Jdot_hands;
	MatrixCXd _pre_J_hands;
	MatrixCXd _pre_Jdot_hands;
	VectorCXd _Jdot_qdot;

	MatrixCXd _J_pos_hands; // 6x15
	MatrixCXd _J_pos_T_hands; // 15x6
	MatrixCXd _J_ori_hands; // 6x15
	MatrixCXd _J_ori_T_hands; // 15x6

    VectorCXd _torque_maintask;
    VectorCXd _torque_subtask;

    void OperationalSpaceControl();

    //Reduced HQP
	CQuadraticProgram rHQP_P1; //first priority task - dual hand control
	CQuadraticProgram rHQP_P2; //second priority task - joint damping
    int _max_iter;
    double _rHQP_threshold;
	MatrixCXd _rH1, _rH2, _rA1, _rA2;
	VectorCXd _rg1, _rg2, _rlbA1, _rlbA2, _rubA1, _rubA2, _rlb1, _rlb2, _rub1, _rub2;
    MatrixCXd _J_Ainv;

    void ReducedHQPTaskSpaceControl();

    ////////////////////////////////////////////////////////////////////////////////////////////////////

    // addons  ////////////////////////////////////////////////////////////////////////////////////////
    void calcJointFrictionCompensationTorque();
    void setJointFrictionCoeff();
    double _static_threshold;
    VectorCXd _torque_friction_comp;
    VectorCXd _static_friction;
    VectorCXd _viscous_friction;
    VectorCXd _max_friction_torque;
    VectorCXi _motion_direction_estimated;
    VectorCXd _torque_motion_estimated;
    VectorCXd _qddot_est;
    VectorCXd _qdot_est;

    void safetyTorqueHat();
	

	////////////////////////////////////////////////////////////////////////////////////////////////////

	
};

#endif