#pragma once
#ifndef __GETHOME_H
#define __GETHOME_H

#include <iostream>
#include <fstream>
#include <eigen3/Eigen/Dense>
#include "custommath.h"

using namespace std;
using namespace Eigen;


class CHoming
{
    public:
	CHoming(int jdof, const int jointtype[]);
	virtual ~CHoming();

    void homing_velocity_control();
    void read(double time, double joint_position[], int home_sensor[]); //time, joint angle, sensor value
    void write(double desired_velocity[], double offset_position[]); //desired joint velocity
    void reset_count();

    private:
    bool _init_time_bool;
    bool _operation_bool;
    bool _print_start_bool;
    int _joint_num;
    int _count;
    int _tot_motion_t;
    double _start_t;
    double _init_t;
    double _wait_t;
    double _motion_t;    
    double _t;
    double _time_count;

    VectorCXd _motion_range_rad_max;
    VectorCXd _motion_range_m_max;
    VectorCXd _motion_range_rad_min;
    VectorCXd _motion_range_m_min;
    VectorCXd _joint_type;
    VectorCXd _q;
    VectorCXi _home_sensor;
    VectorCXi _home_sensor_pre;
    VectorCXd _qdot_des;
    VectorCXd _q_offset;
    VectorCXd _homing_end;
    VectorCXd _t_motion_1, _t_motion_2, _t_motion_3; //motion time
    VectorCXd _motion_range_max;
    VectorCXd _motion_range_min;
    VectorCXd _q_start_touch;
    VectorCXd _q_end_touch;
    VectorCXb _bool_start_touch;
    VectorCXb _bool_end_touch;

    void joint_velocity_homing_motion();
    void check_home_sensor();
    void calculate_homeoffset();
};

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

class CMoveHome
{
    public:
	CMoveHome(int jdof, const double touch_probe_position_rad[]);
	virtual ~CMoveHome();

    void read(double time, double joint_position[], double home_offset_position[], int home_sensor[]);
    void move_touchsensor_position();
    void move_home_position();
    void write(double desired_velocity[]);

    private:
    bool _init_time_bool;
    bool _operation_bool;
    bool _print_start_bool;
    double _start_t;
    double _init_t;
    double _wait_t;
    double _motion_t;    
    double _t;
    int _joint_num;

    VectorCXd _q;
    VectorCXd _q_init;
    VectorCXd _q_offset;
    VectorCXi _home_sensor;
    VectorCXd _qdot_des;
    VectorCXd _q_touch_probe_rad;
};

#endif