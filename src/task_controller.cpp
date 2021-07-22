#include "task_controller.h"

CTaskController::CTaskController(int jdof, double dt, const double touch_probe_position_rad[])
{    
    _jdof = jdof;
    _dt = dt;    

    initialize();

    _q_touch_probe(0) = touch_probe_position_rad[14];
    for(int i=0; i<_jdof-1; i++)
    {
        _q_touch_probe(i+1) = touch_probe_position_rad[i];
    }    
}

CTaskController::~CTaskController()
{
}

void CTaskController::get_joint_position_offset(double joint_position_offset[])
{
    _q_offset(0) = joint_position_offset[14];

    for(int i=0; i<_jdof-1; i++)
    {
        _q_offset(i+1) = joint_position_offset[i];
    }
}

void CTaskController::read(double time, double joint_position[], double joint_velocity[], double joint_force[])
{
    _time = time;

    // Note: order of joint of model and elmo direver is different
    _q(0) = joint_position[14] - (_q_offset(0) - _q_touch_probe(0));
    _qdot(0) = joint_velocity[14];    
    _torque(0) = joint_force[14];

    for(int i=0; i<_jdof-1; i++)
    {
        _q(i+1) = joint_position[i] -(_q_offset(i+1) - _q_touch_probe(i+1));
        _qdot(i+1) = joint_velocity[i];
        _torque(i+1) = joint_force[i];
    }

    for(int i=0; i<_jdof; i++) //joint velocity lowpass filter
    {
        _qdot_lp(i) = CustomMath::VelLowpassFilter(_dt, 2.0*PI* 10.0, _pre_q(i), _q(i), _pre_qdot(i)); //low-pass filter                
        //_qdot_lp(i) = CustomMath::LowPassFilter(_dt, 2.0*PI* 20.0, _qdot(i), _pre_qdot(i)); //low-pass filter
        _pre_q(i) = _q(i);
	    _pre_qdot(i) = _qdot_lp(i);

        if(_time < 2.0)///use filtered data after convergece
        {
            _qdot_lp(i) = _qdot(i);
        }		
    }

    if (_bool_init == true)
	{
		_init_time = _time;
		_bool_init = false;
	}
}

void CTaskController::model_update()
{
    Model.update_kinematics(_q, _qdot_lp);
	Model.update_dynamics();
	Model.calculate_EE_Jacobians();
	Model.calculate_EE_positions_orientations();
	Model.calculate_EE_velocity();

    //update end-effector state (position and orientation)
    _tmp_vec_ori = CustomMath::GetBodyRotationAngle(Model._R_left_hand);
    for(int i=0; i<3; i++)
    {
        _x_left_hand(i) = Model._x_left_hand(i);
        _x_left_hand(i+3) = _tmp_vec_ori(i);
    }
    _tmp_vec_ori = CustomMath::GetBodyRotationAngle(Model._R_right_hand);
    for(int i=0; i<3; i++)
    {        
        _x_right_hand(i) = Model._x_right_hand(i);
        _x_right_hand(i+3) = _tmp_vec_ori(i);
    }
    _xdot_left_hand = Model._xdot_left_hand;
	_xdot_right_hand = Model._xdot_right_hand;

	//set Jacobian
	_J_hands.block<6, 15>(0, 0) = Model._J_left_hand;
	_J_hands.block<6, 15>(6, 0) = Model._J_right_hand;
	_J_T_hands = _J_hands.transpose();

	_J_ori_hands.block<3, 15>(0, 0) = Model._J_left_hand_ori;
	_J_ori_hands.block<3, 15>(3, 0) = Model._J_right_hand_ori;
	_J_ori_T_hands = _J_ori_hands.transpose();
	_J_pos_hands.block<3, 15>(0, 0) = Model._J_left_hand_pos;
	_J_pos_hands.block<3, 15>(3, 0) = Model._J_right_hand_pos;
	_J_pos_T_hands = _J_pos_hands.transpose();

	//calc Jacobian dot (with lowpass filter)	
	for (int i = 0; i < 12; i++)
	{
		for (int j = 0; j < 15; j++)
		{
			_Jdot_hands(i,j) = CustomMath::VelLowpassFilter(_dt, 2.0 * PI * 20.0, _pre_J_hands(i,j), _J_hands(i,j), _pre_Jdot_hands(i,j)); //low-pass filter

			_pre_J_hands(i, j) = _J_hands(i, j);
			_pre_Jdot_hands(i, j) = _Jdot_hands(i, j);
		}		
	}
	_Jdot_qdot.noalias() = _Jdot_hands * _qdot_lp;
}

void CTaskController::write(double torque_desired[])
{
    _torque_friction_comp.setZero();
    calcJointFrictionCompensationTorque(); //calc joint friction compensation torque

    for(int i=0; i<_jdof-1; i++)
    {
        torque_desired[i] = _torque_des(i+1) + _torque_friction_comp(i+1);
    }
    //torque_desired[14] = _torque_des(0) + _torque_friction_comp(0);
    torque_desired[14] = 0.0; //for temporary (not using prismatic joint)
}

void CTaskController::reset_target(double motion_time)
{
	_control_mode = 0; //gravity compensation
	_motion_time = motion_time;
	_bool_joint_motion = false;
	_bool_ee_motion = false;
}

void CTaskController::reset_target(double motion_time, VectorCXd target_joint_position)
{
	_control_mode = 1; //joint space control
	_motion_time = motion_time;
	_bool_joint_motion = false;
	_bool_ee_motion = false;

	_q_goal = target_joint_position.head(15);
	_qdot_goal.setZero();
}

void CTaskController::reset_target(double motion_time, Vector3d target_pos_lh, Vector3d target_ori_lh, Vector3d target_pos_rh, Vector3d target_ori_rh)
{
	_control_mode = 2;
	_motion_time = motion_time;
	_bool_joint_motion = false;
	_bool_ee_motion = false;

    for(int i=0; i<3; i++)
    {
        _x_goal_left_hand(i) = target_pos_lh(i);
        _x_goal_left_hand(i+3) = target_ori_lh(i);
        _x_goal_right_hand(i) = target_pos_rh(i);
        _x_goal_right_hand(i+3) = target_ori_rh(i);
    }
	_xdot_goal_left_hand.setZero();	
	_xdot_goal_right_hand.setZero();
}


void CTaskController::motion_plan()
{
    _time_plan(1) = 1.0; //move home position
    _time_plan(2) = 0.5; //wait
    _time_plan(3) = 1.0; //operational space control
    _time_plan(4) = 100000.0; //operational space control - keep position
    _time_plan(5) = 1000000.0; //gravity compensation

    if (_bool_plan(_cnt_plan) == 1)
	{
		_cnt_plan = _cnt_plan + 1;
		if (_cnt_plan == 1)
		{
			reset_target(_time_plan(_cnt_plan), _q_home);
            cout << "Joint Control: Homing, t= " <<_time_plan(_cnt_plan) <<" s" << endl;
		}
		else if (_cnt_plan == 2)
		{
			reset_target(_time_plan(_cnt_plan), _q);
            cout << "Joint Control: Wait, t= " <<_time_plan(_cnt_plan) <<" s" << endl;
		}
        else if (_cnt_plan == 3)
        {
            for(int i=0;i<3;i++)
            {
                _pos_goal_left_hand(i) = _x_left_hand(i);
                _rpy_goal_left_hand(i) = _x_left_hand(i+3);
                _pos_goal_right_hand(i) = _x_right_hand(i);
                _rpy_goal_right_hand(i) = _x_right_hand(i+3);
            }
            _pos_goal_left_hand(2) = _x_left_hand(2)+0.05;
            _pos_goal_right_hand(2) = _x_right_hand(2)+0.05;

            //_rpy_goal_left_hand(0) = _x_left_hand(3) - 10.0*DEG2RAD;
            //_rpy_goal_right_hand(0) = _x_right_hand(3) + 10.0*DEG2RAD;
            
            
			reset_target(_time_plan(_cnt_plan), _pos_goal_left_hand, _rpy_goal_left_hand, _pos_goal_right_hand, _rpy_goal_right_hand);
            cout << "Operational Space Control: Move position, t= " <<_time_plan(_cnt_plan) <<" s" << endl;
        }
        else if (_cnt_plan == 4)
        {
            for(int i=0;i<3;i++)
            {
                _pos_goal_left_hand(i) = _x_left_hand(i);
                _rpy_goal_left_hand(i) = _x_left_hand(i+3);
                _pos_goal_right_hand(i) = _x_right_hand(i);
                _rpy_goal_right_hand(i) = _x_right_hand(i+3);
            }
			reset_target(_time_plan(_cnt_plan), _pos_goal_left_hand, _rpy_goal_left_hand, _pos_goal_right_hand, _rpy_goal_right_hand);
            cout << "Operational Space Control: Keep position, t= " <<_time_plan(_cnt_plan) <<" s" << endl;
        }
        else if (_cnt_plan == 5)
        {
            reset_target(_time_plan(_cnt_plan));
            cout << "Gravity Comp, t= " <<_time_plan(_cnt_plan) <<" s" << endl;
        }
	}
}

void CTaskController::compute()
{
    model_update();
    motion_plan();

    _torque_des.setZero();

    _torque_des = Model._bg;

    if (_control_mode == 0) //gravity compensation
    {
        if (_time - _init_time < 0.1 && _bool_joint_motion == false)
		{
			_start_time = _init_time;
			_end_time = _start_time + _motion_time;
			JointTrajectory.reset_initial(_start_time, _q, _qdot_lp); //dummy
			JointTrajectory.update_goal(_q_goal, _qdot_goal, _end_time); //dummy
			_bool_joint_motion = true;
		}
		JointTrajectory.update_time(_time); //dummy

		_torque_des = Model._bg;

		if (JointTrajectory.check_trajectory_complete() == 1)
		{
			_bool_plan(_cnt_plan) = 1;
			_bool_init = true;
		}
         _torque_des = Model._bg;
    }
    else if (_control_mode == 1) //joint space control
	{
		if (_time - _init_time < 0.1 && _bool_joint_motion == false)
		{
			_start_time = _init_time;
			_end_time = _start_time + _motion_time;
			JointTrajectory.reset_initial(_start_time, _q, _qdot_lp);
			JointTrajectory.update_goal(_q_goal, _qdot_goal, _end_time);
			_bool_joint_motion = true;
		}
		JointTrajectory.update_time(_time);
		_q_des = JointTrajectory.position_cubicSpline();
		_qdot_des = JointTrajectory.velocity_cubicSpline();

		bool option = true;
        jointControl(option);

		if (JointTrajectory.check_trajectory_complete() == 1)
		{
			_bool_plan(_cnt_plan) = 1;
			_bool_init = true;
		}
	}
    else if (_control_mode == 2) //operational space control
	{
        if (_time - _init_time< 0.1 && _bool_ee_motion == false)
		{
			_start_time = _init_time;
			_end_time = _start_time + _motion_time;
			LeftHandTrajectory.reset_initial(_start_time, _x_left_hand, _xdot_left_hand);
			LeftHandTrajectory.update_goal(_x_goal_left_hand, _xdot_goal_left_hand, _end_time);
			RightHandTrajectory.reset_initial(_start_time, _x_right_hand, _xdot_right_hand);
			RightHandTrajectory.update_goal(_x_goal_right_hand, _xdot_goal_right_hand, _end_time);
			_bool_ee_motion = true;
		}
		LeftHandTrajectory.update_time(_time);
		_x_des_left_hand = LeftHandTrajectory.position_cubicSpline();
		_xdot_des_left_hand = LeftHandTrajectory.velocity_cubicSpline();

		RightHandTrajectory.update_time(_time);
		_x_des_right_hand = RightHandTrajectory.position_cubicSpline();
		_xdot_des_right_hand = RightHandTrajectory.velocity_cubicSpline();

        //OperationalSpaceControl();

        ReducedHQPTaskSpaceControl();

		if (LeftHandTrajectory.check_trajectory_complete() == 1 || RightHandTrajectory.check_trajectory_complete() == 1)
		{
			_bool_plan(_cnt_plan) = 1;
			_bool_init = true;
		}
    }
}

void CTaskController::jointControl(bool bool_offdiag)
{
    _kpj = 50.0;
    _kvj = 5.0;

	_torque_des.setZero();
    if(bool_offdiag == true)
    {
        _A = Model._A;
    }
    else
    {
        _A.setZero();
        for(int i=0; i< _jdof; i++)
        {
            _A(i,i) =  Model._A(i,i);
        }
    }    
    _q_err.noalias() = _q_des - _q;
    _qdot_err.noalias() = _qdot_des - _qdot_lp;
    _qddot_ref.noalias() = _kpj*_q_err + _kvj*_qdot_err;
    _torque_maintask.noalias() = _A*_qddot_ref;

    safetyTorqueHat();
	_torque_des.noalias() = _torque_maintask + Model._bg;
}

void CTaskController::OperationalSpaceControl()
{
	_torque_des.setZero();	
	_kp = 100.0;//100.0;
	_kv = 5.0;//5.0;

	_x_err_left_hand.noalias() = _x_des_left_hand.head(3) - _x_left_hand.head(3);
	_R_des_left_hand.noalias() = CustomMath::GetBodyRotationMatrix(_x_des_left_hand(3), _x_des_left_hand(4), _x_des_left_hand(5));	    
	_R_err_left_hand.noalias() = -CustomMath::getPhi(Model._R_left_hand, _R_des_left_hand);    
	_x_err_right_hand.noalias() = _x_des_right_hand.head(3) - _x_right_hand.head(3);    
	_R_des_right_hand.noalias() = CustomMath::GetBodyRotationMatrix(_x_des_right_hand(3), _x_des_right_hand(4), _x_des_right_hand(5));    
	_R_err_right_hand.noalias() = -CustomMath::getPhi(Model._R_right_hand, _R_des_right_hand);

	_xdot_err_left_hand.noalias() = _xdot_des_left_hand.head(3) - _xdot_left_hand.head(3);
	_Rdot_err_left_hand.noalias() = -_xdot_left_hand.tail(3); //only daming for orientation
	_xdot_err_right_hand.noalias() = _xdot_des_right_hand.head(3) - _xdot_right_hand.head(3);
	_Rdot_err_right_hand.noalias() = -_xdot_right_hand.tail(3); //only daming for orientation	

    _xddot_star.setZero();
    for(int i=0; i<3; i++)
    {
        _xddot_star(i) = _kp * _x_err_left_hand(i) + _kv * _xdot_err_left_hand(i);//left hand position control
        _xddot_star(i+3) = _kp * _R_err_left_hand(i) + _kv * _Rdot_err_left_hand(i);//left hand orientation control
        _xddot_star(i+6) = _kp * _x_err_right_hand(i) + _kv * _xdot_err_right_hand(i);//right hand position control
        _xddot_star(i+9) = _kp * _R_err_right_hand(i) + _kv * _Rdot_err_right_hand(i);//right hand orientation control
    }	
    //_xddot_star(0) = 0.0;
    //_xddot_star(6) = 0.0;
		
	// 1st: hands pos and ori, 2nd: joint dampings + prismatic fix    
	_Lambda_hands.setZero();
    _A = Model._A;
    _A_inv.noalias() = _A.inverse();
    _J_T_hands_inv.noalias() = CustomMath::pseudoInverseQR(_J_T_hands); 
    _J_hands_inv.noalias() = CustomMath::pseudoInverseQR(_J_hands);    
    _J_T_hands_inv_A.noalias() = _J_T_hands_inv * _A;
	_Lambda_hands.noalias() = _J_T_hands_inv_A * _J_hands_inv;	
    _J_T_hands_Lambda_hands.noalias() = _J_T_hands * _Lambda_hands;
    _J_hands_A_inv.noalias() = _J_hands * _A_inv;    
    _J_T_hands_Lambda_hands_J_hands_A_inv.noalias() = _J_T_hands_Lambda_hands*_J_hands_A_inv;
	_Null_hands.noalias() = _Id_15 - _J_T_hands_Lambda_hands_J_hands_A_inv;	    
    
    _torque_maintask.noalias() = _J_T_hands_Lambda_hands * _xddot_star;
    _qddot_ref.noalias() = -0.2*_kvj * _qdot_lp;
    _qddot_ref(0) = 0.2*(_kpj*(_q_goal(0)-_q(0)) - _kvj*_qdot_lp(0)); //prismatic joint is controlled to keep position
    //_qddot_ref(2) = 0.2*(_kpj*(_q_home(2)-_q(2)) - _kvj*_qdot_lp(2)); //for shoulder roll joint posture
    //_qddot_ref(9) = 0.2*(_kpj*(_q_home(9)-_q(9)) - _kvj*_qdot_lp(9)); //for shoulder roll joint posture
    _A_qddot_ref.noalias() =_A*_qddot_ref;
    _torque_subtask.noalias() = _Null_hands * _A_qddot_ref;    
    //_torque_subtask.setZero();    

    safetyTorqueHat();    
	_torque_des.noalias() = _torque_maintask + _torque_subtask + Model._bg;    
}

void CTaskController::ReducedHQPTaskSpaceControl()
{
	_torque_des.setZero();

	_kp = 100.0;
	_kv = 5.0;

    _x_err_left_hand.noalias() = _x_des_left_hand.head(3) - _x_left_hand.head(3);
	_R_des_left_hand.noalias() = CustomMath::GetBodyRotationMatrix(_x_des_left_hand(3), _x_des_left_hand(4), _x_des_left_hand(5));	    
	_R_err_left_hand.noalias() = -CustomMath::getPhi(Model._R_left_hand, _R_des_left_hand);    
	_x_err_right_hand.noalias() = _x_des_right_hand.head(3) - _x_right_hand.head(3);    
	_R_des_right_hand.noalias() = CustomMath::GetBodyRotationMatrix(_x_des_right_hand(3), _x_des_right_hand(4), _x_des_right_hand(5));    
	_R_err_right_hand.noalias() = -CustomMath::getPhi(Model._R_right_hand, _R_des_right_hand);

	_xdot_err_left_hand.noalias() = _xdot_des_left_hand.head(3) - _xdot_left_hand.head(3);
	_Rdot_err_left_hand.noalias() = -_xdot_left_hand.tail(3); //only daming for orientation
	_xdot_err_right_hand.noalias() = _xdot_des_right_hand.head(3) - _xdot_right_hand.head(3);
	_Rdot_err_right_hand.noalias() = -_xdot_right_hand.tail(3); //only daming for orientation
    
    _xddot_star.setZero();
    for(int i=0; i<3; i++)
    {
        _xddot_star(i) = _kp * _x_err_left_hand(i) + _kv * _xdot_err_left_hand(i);//left hand position control
        _xddot_star(i+3) = _kp * _R_err_left_hand(i) + _kv * _Rdot_err_left_hand(i);//left hand orientation control
        _xddot_star(i+6) = _kp * _x_err_right_hand(i) + _kv * _xdot_err_right_hand(i);//right hand position control
        _xddot_star(i+9) = _kp * _R_err_right_hand(i) + _kv * _Rdot_err_right_hand(i);//right hand orientation control
    }

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Solve rHQP   /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	
	//first priority task QP ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//set cost function x^T*H*x + g	
	_rH1.setZero();
	for (int i = 0; i < 15; i++)
	{
		_rH1(i, i) = 0.0001;
	}
	for (int i = 15; i < 27; i++)
	{
		_rH1(i, i) = 1.0;
	}	
	_rg1.setZero();
	rHQP_P1.UpdateMinProblem(_rH1, _rg1);

	//set A*x <= b	
	_rA1.setZero();
	_rlbA1.setZero();
	_rubA1.setZero();

    _A = Model._A;
    _A_inv.noalias() = _A.inverse();	
	_J_Ainv.noalias() = _J_hands * _A_inv;

	_rA1.block<12, 15>(0, 0) = _J_Ainv;
	_rA1.block<12, 12>(0, 15) = -_Id_12;

	for (int i = 0; i < 12; i++)
	{
		_rlbA1(i) = - _Jdot_qdot(i) + _xddot_star(i) - _rHQP_threshold;
		_rubA1(i) = - _Jdot_qdot(i) + _xddot_star(i) + _rHQP_threshold;
	}
	rHQP_P1.UpdateSubjectToAx(_rA1, _rlbA1, _rubA1);

	//set lb <= x <= ub	
	_rlb1.setZero();
	_rub1.setZero();
	//joint torque limit
	for (int i = 0; i < 15; i++)
	{
		//torque limit
		_rlb1(i) = Model._min_joint_torque(i) - Model._bg(i);
		_rub1(i) = Model._max_joint_torque(i) - Model._bg(i);
    }
	//task limit	
	for (int i = 0; i < 12; i++)
	{
		_rlb1(i + 15) = -1000.0;
		_rub1(i + 15) = 1000.0;
	}
	rHQP_P1.UpdateSubjectToX(_rlb1, _rub1);

	//Solve
	rHQP_P1.EnableEqualityCondition(0.0001);
	rHQP_P1.SolveQPoases(_max_iter);

	//second priority task QP ////////////////////////////////////////////////////////////////////////////////////////////////////////////////	
	//set cost function x^T*H*x + g	
	_rH2.setZero();
	for (int i = 0; i < 15; i++)
	{
		_rH2(i, i) = 0.00001;
	}
	for (int i = 15; i < 30; i++)
	{
		_rH2(i, i) = 1.0;
	}
	_rH2.block<15, 15>(15, 15) = _A;
	_rg2.setZero();
	rHQP_P2.UpdateMinProblem(_rH2, _rg2);

	//set A*x <= b	
	_rA2.setZero();
	_rlbA2.setZero();
	_rubA2.setZero();
	_rA2.block<15, 15>(0, 0) = _A_inv;
	_rA2.block<15, 15>(0, 15) = -_Id_15;
	_rA2.block<12, 15>(15, 0) = _J_Ainv;	

	_qddot_ref.setZero();
	_qddot_ref.noalias() = -0.2*_kvj * _qdot_lp;

	for (int i = 0; i < 15; i++)
	{
		_rlbA2(i) = _qddot_ref(i) - _rHQP_threshold;
		_rubA2(i) = _qddot_ref(i) + _rHQP_threshold;
	}
	for (int i = 0; i < 12; i++)
	{
		_rlbA2(i + 15) = _Jdot_qdot(i) + _xddot_star(i) + rHQP_P1._Xopt(i + 15) - _rHQP_threshold;
		_rubA2(i + 15) = _Jdot_qdot(i) + _xddot_star(i) + rHQP_P1._Xopt(i + 15) + _rHQP_threshold;
	}
	rHQP_P2.UpdateSubjectToAx(_rA2, _rlbA2, _rubA2);

	//set lb <= x <= ub
	_rlb2.setZero();
	_rub2.setZero();
	
	//joint torque limit
	for (int i = 0; i < 15; i++)
	{
		_rlb2(i) = Model._min_joint_torque(i) - Model._bg(i);
		_rub2(i) = Model._max_joint_torque(i) - Model._bg(i);		
	}
	//task limit
	for (int i = 0; i < 15; i++)
	{
		_rlb2(i + 15) = -1000.0;
		_rub2(i + 15) = 1000.0;
	}
	rHQP_P2.UpdateSubjectToX(_rlb2, _rub2);

	//Solve
	rHQP_P2.EnableEqualityCondition(0.0001);
	rHQP_P2.SolveQPoases(_max_iter);
	_torque_des.noalias() = rHQP_P2._Xopt.segment(0, 15) + Model._bg;
	//cout << _torque_des.transpose() << endl;
}


void CTaskController::calcJointFrictionCompensationTorque()
{
    //for motion direction estimation
    _A = Model._A;
    _A_inv.noalias() = _A.inverse();
    _torque_motion_estimated = _torque_des -  Model._bg;
    _qddot_est.noalias() = _A_inv*_torque_motion_estimated;
    _qdot_est.noalias() = _qdot_lp + 0.0*_qddot_est/_dt;//Aqddot + bg = tau => qddot = Ainv(tau - bg) => qdot_est = qdot+qddot/dt
    for(int i=0; i<_jdof; i++)
    {
        //estimate motion direction
        if(_qdot_est(i) > _static_threshold)
        {
            _motion_direction_estimated(i) = 1; 
        }
        else if(_qdot_est(i) < _static_threshold)
        {
            _motion_direction_estimated(i) = -1; 
        }
        else
        {
            _motion_direction_estimated(i) = 0; 
        }

        if(abs(_qdot_lp(i)) < _static_threshold) //for static joint
        {
            _torque_friction_comp(i) = _motion_direction_estimated(i)*_static_friction(i);
        }
        else //for moving joint
        {
            _torque_friction_comp(i) = _motion_direction_estimated(i)*(0.5*_static_friction(i) + _viscous_friction(i)*abs(_qdot_lp(i)));
        }

        //set maximum
        if(_torque_friction_comp(i) >_max_friction_torque(i))
        {
            _torque_friction_comp(i) =_max_friction_torque(i);
        }
        else if(_torque_friction_comp(i) < -_max_friction_torque(i))
        {
            _torque_friction_comp(i) = -_max_friction_torque(i);
        }
    }
}


void CTaskController::safetyTorqueHat()
{
    for(int i=0; i<_jdof; i++)
    {
        if(_torque_maintask(i) > 10.0)
        {
            _torque_maintask(i) = 10.0;
        }
        else if(_torque_maintask(i) < -10.0)
        {
            _torque_maintask(i) = -10.0;
        }

        if(_torque_subtask(i) > 5.0)
        {
            _torque_subtask(i) = 5.0;
        }
        else if(_torque_subtask(i) < -5.0)
        {
            _torque_subtask(i) = -5.0;
        }
    }
}

void CTaskController::initialize()
{
    _time = 0.0;

    _q.setZero(_jdof);
    _qdot.setZero(_jdof);
    _torque.setZero(_jdof);

    _qdot_lp.setZero(_jdof);
    _pre_q.setZero(_jdof);
    _pre_qdot.setZero(_jdof);

    _q_offset.setZero(_jdof);
    _q_touch_probe.setZero(_jdof);    
    _q_zero.setZero(_jdof);
    _q_home.setZero(_jdof);
    _q_home(0) = 0.0;
	_q_home(1) = 30.0 * DEG2RAD; //LShP
	_q_home(8) = -30.0 * DEG2RAD; //RShP
	_q_home(2) = 20.0 * DEG2RAD; //LShR
	_q_home(9) = -20.0 * DEG2RAD; //RShR
	_q_home(4) = 80.0 * DEG2RAD; //LElP
	_q_home(11) = -80.0 * DEG2RAD; //RElP
	_q_home(6) = -45.0 * DEG2RAD; //LWrP
	_q_home(13) = 45.0 * DEG2RAD; //RWrP  

    _Id_15.setZero(_jdof,_jdof);
    _Id_15.setIdentity(_jdof,_jdof);
    _Id_12.setZero(12,12);
    _Id_12.setIdentity(12,12);

    _control_mode = 0; //first motion, gravity compensation  
    
    //variables for motion plan
    _start_time = 0.0;
    _end_time = 0.0;
    _motion_time = 5.0;
    _init_time = 0.0;
    _cnt_plan = 0;
    _bool_init = true;
    _bool_joint_motion = false;
    _bool_ee_motion = false;    
    _bool_plan.setZero(10);
    _time_plan.setZero(10);
    _time_plan.setConstant(5.0);
    reset_target(_time_plan(_cnt_plan)); //first motion, gravity compensation

    //joint space
    _q_goal.setZero(_jdof);
    _qdot_goal.setZero(_jdof);
    JointTrajectory.set_size(_jdof);
    
    //operational space
    _x_goal_left_hand.setZero(6);
	_xdot_goal_left_hand.setZero(6);	
	_x_goal_right_hand.setZero(6);
	_xdot_goal_right_hand.setZero(6);	
    _pos_goal_left_hand.setZero();
    _rpy_goal_left_hand.setZero();
    _pos_goal_right_hand.setZero();
    _rpy_goal_right_hand.setZero();

    LeftHandTrajectory.set_size(6);
	RightHandTrajectory.set_size(6);

    //variables for control
    //joint space
    _kpj = 50.0;
    _kvj = 5.0;
    _A.setZero(_jdof,_jdof);
    _q_des.setZero(_jdof);
    _qdot_des.setZero(_jdof);
    _torque_des.setZero(_jdof);
    _q_err.setZero(_jdof);
    _qdot_err.setZero(_jdof);
    _qddot_ref.setZero(_jdof);

    //operational space    
    _kp = 50.0;
    _kv = 5.0;
    _x_left_hand.setZero(6); // 6x1
    _x_right_hand.setZero(6); // 6x1
    _xdot_left_hand.setZero(6); // 6x1
    _xdot_right_hand.setZero(6); // 6x1
    _tmp_vec_ori.setZero();

    _x_des_left_hand.setZero(6);
    _xdot_des_left_hand.setZero(6);
    _x_des_right_hand.setZero(6);
    _xdot_des_right_hand.setZero(6);
    _R_des_left_hand.setZero();
    _R_des_right_hand.setZero();

    _x_err_left_hand.setZero();
    _x_err_right_hand.setZero();
    _xdot_err_left_hand.setZero();
    _xdot_err_right_hand.setZero();
    _R_err_left_hand.setZero();
    _R_err_right_hand.setZero();
    _Rdot_err_left_hand.setZero();
    _Rdot_err_right_hand.setZero();

    _Lambda_hands.setZero(12,12);
    _J_T_hands_inv_A.setZero(12,_jdof);
    _Null_hands.setZero(_jdof,_jdof);    
    _xddot_star.setZero(12);
    _A_qddot_ref.setZero(_jdof);

    _A_inv.setZero(_jdof,_jdof);   
    _J_hands.setZero(12, _jdof);
    _J_T_hands.setZero(_jdof, 12);
    _J_hands_inv.setZero(_jdof, 12);
    _J_T_hands_inv.setZero(12, _jdof);
    _J_T_hands_Lambda_hands.setZero(_jdof, 12);
    _J_hands_A_inv.setZero(12, _jdof);	
    _J_T_hands_Lambda_hands_J_hands_A_inv.setZero(_jdof,_jdof);

	_Jdot_hands.setZero(12, _jdof);	
    _pre_J_hands.setZero(12, _jdof);
	_pre_Jdot_hands.setZero(12, _jdof);
    _Jdot_qdot.setZero(12);

    _J_pos_hands.setZero(6, _jdof);
	_J_pos_T_hands.setZero(_jdof, 6);
	_J_ori_hands.setZero(6, _jdof);
	_J_ori_T_hands.setZero(_jdof, 6);

    _torque_maintask.setZero(_jdof);
    _torque_subtask.setZero(_jdof);

    //reduced HQP
    _max_iter = 100;    
    _rHQP_threshold = 0.0001;
	rHQP_P1.InitializeProblemSize(27, 12); //variable size = (joint dof)+(task dof), constraint size =(task dof) 
	_rH1.setZero(rHQP_P1._num_var, rHQP_P1._num_var);
	_rg1.setZero(rHQP_P1._num_var);
	_rA1.setZero(rHQP_P1._num_cons, rHQP_P1._num_var);
	_rlbA1.setZero(rHQP_P1._num_cons);
	_rubA1.setZero(rHQP_P1._num_cons);
	_rlb1.setZero(rHQP_P1._num_var);
	_rub1.setZero(rHQP_P1._num_var);
	rHQP_P2.InitializeProblemSize(30, 27); //variable size = (joint dof)+(task dof), constraint size =(1st prioirty task dof)  + (2nd prioirty task dof) 
	_rH2.setZero(rHQP_P2._num_var, rHQP_P2._num_var);
	_rg2.setZero(rHQP_P2._num_var);
	_rA2.setZero(rHQP_P2._num_cons, rHQP_P2._num_var);
	_rlbA2.setZero(rHQP_P2._num_cons);
	_rubA2.setZero(rHQP_P2._num_cons);
	_rlb2.setZero(rHQP_P2._num_var);
	_rub2.setZero(rHQP_P2._num_var);
    _J_Ainv.setZero(12, 15);

    //friction compensation
    _torque_friction_comp.setZero(_jdof);
    _static_friction.setZero(_jdof);
    _viscous_friction.setZero(_jdof);
    _max_friction_torque.setZero(_jdof);
    _motion_direction_estimated.setZero(_jdof);
    _torque_motion_estimated.setZero(_jdof);
    _qddot_est.setZero(_jdof);
    _qdot_est.setZero(_jdof);
    _static_threshold = 0.0;    
    setJointFrictionCoeff();//get joint frictions
}

void CTaskController::setJointFrictionCoeff()
{
    _static_threshold = 0.01;

    _static_friction.setConstant(0.5);
    _static_friction(1) = 2.0;
    _static_friction(2) = 2.0;
    _static_friction(3) = 1.5;
    _static_friction(4) = 1.5;
    _static_friction(8) = 2.0;
    _static_friction(9) = 2.0;
    _static_friction(10) = 1.5;
    _static_friction(11) = 1.5;

    _viscous_friction.setConstant(3.0);
    _viscous_friction(1) = 8.0;
    _viscous_friction(2) = 8.0;
    _viscous_friction(3) = 6.0;
    _viscous_friction(4) = 6.0;
    _viscous_friction(8) = 8.0;
    _viscous_friction(9) = 8.0;
    _viscous_friction(10) = 6.0;
    _viscous_friction(11) = 6.0;    

    _max_friction_torque.setConstant(2.0);
    _max_friction_torque(1) = 3.5;
    _max_friction_torque(2) = 3.5;
    _max_friction_torque(3) = 2.5;
    _max_friction_torque(4) = 2.5;
    _max_friction_torque(8) = 3.5;
    _max_friction_torque(9) = 3.5;
    _max_friction_torque(10) = 2.5;
    _max_friction_torque(11) = 2.5;

}
