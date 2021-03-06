#ifndef QUADRATICPROGRAM_H
#define QUADRATICPROGRAM_H

#include <iostream>
#include <eigen3/Eigen/Dense>
#include "custommath.h"
#include "qpOASES.hpp"

using namespace Eigen;
using namespace std;
using namespace qpOASES;

class CQuadraticProgram
{
public:
	CQuadraticProgram();
	virtual ~CQuadraticProgram();

public:
	void InitializeProblemSize(const int& num_var, const int& num_cons);
	void UpdateMinProblem(const MatrixCXd& H, const VectorCXd& g);
	void UpdateSubjectToAx(const MatrixCXd& A, const VectorCXd& lbA, const VectorCXd& ubA);
	void UpdateSubjectToX(const VectorCXd& lb, const VectorCXd& ub);
	void DeleteSubjectToAx();
	void DeleteSubjectToX();
	void PrintMinProb();
	void PrintSubjectToAx();
	void PrintSubjectTox();
	void EnablePrintOptionDebug();
	void DisablePrintOptionDebug();
	void EnableEqualityCondition(const double Tolerance);
	void DisableEqualityCondition();
	void SetHotstartComputationTimeLimit(const real_t& compute_time); //it maybe unstable
	void SolveQPoases(const int& num_max_iter);
	int _num_state;
	VectorCXd _Xopt;
	bool _bInitialized;
	int _num_var;
	int _num_cons;

private:
	SQProblem _SQPprob;
	Options _options;	
	MatrixCXd _H;
	VectorCXd _g;
	bool _bool_constraint_Ax;
	MatrixCXd _A;
	VectorCXd _lbA;
	VectorCXd _ubA;
	bool _bool_constraint_x;
	VectorCXd _lb;
	VectorCXd _ub;
	real_t _comptime;

	real_t _H_realt[10000] = { 0 };
	real_t _g_realt[100] = { 0 };
	real_t _A_realt[10000] = { 0 };
	real_t _lbA_realt[100] = { 0 };
	real_t _ubA_realt[100] = { 0 };
	real_t _lb_realt[100] = { 0 };
	real_t _ub_realt[100] = { 0 };
	real_t _Xopt_realt[100] = { 0 };


	void Initialize();
};

#endif // QUADRATICPROGRAM_H