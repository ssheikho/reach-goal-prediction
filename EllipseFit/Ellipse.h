#ifndef ELLIPSE_H
#define ELLIPSE_H


#include "FitFunctions.h"
#include "LinearAlgebra.h"
#include "EllipseImplicitFit.h"

#include "EllipseConicConstraints.h"

#include <Eigen/Dense>
#include <cmath>

#include "RigidTrans2D.h"


#include <Eigen/Geometry> 
#include <cmath>
#include "ceres/ceres.h"
#include "ceres/rotation.h"

#include "ceres/solver.h"

#include "gflags/gflags.h"
#include "glog/logging.h"

#include <iostream>


using namespace Eigen;


using ceres::AutoDiffCostFunction;
using ceres::DynamicAutoDiffCostFunction;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solve;
using ceres::Solver;
using ceres::CauchyLoss;

DECLARE_double(fitErrNtolerance);

DECLARE_string(trust_region_strategy);
 
DECLARE_int32(num_iterations);


DECLARE_string(dogleg);
DECLARE_bool(inner_iterations);
DECLARE_string(blocks_for_inner_iterations);

DECLARE_double(eta);

DECLARE_double(max_solver_time);

DECLARE_bool(nonmonotonic_steps);


//template<typename T>
class Ellipse {
public:
	Ellipse(double a, double b, double alpha, double cX, double cY);

	Ellipse();
	// 2D (xy) inPts
	Ellipse(MatrixXd inPtsAlongCols);
	
	Ellipse(EllipseImplicitFit eif, MatrixXd inPtsAlongCols);
	Ellipse(MatrixXd inPtsAlongCols, double alphaFrac);
	
	Ellipse(EllipseImplicitFit eif);

	double* solveConicVect(MatrixXd inPtsAlongCols);

	~Ellipse();
	
	void setAB(double a, double b);

	double getA();

	double getB();

	RigidTrans2D getRT2D();

	MatrixXd getPointAtThetaH(double theta);

	double getPointXAtThetaH(double theta);

	double getPointYAtThetaH(double theta);

	double getPointWAtThetaH(double theta);


	MatrixXd getPointAtThetasH(MatrixXd thetas);

protected:
	double _a, _b;
	RigidTrans2D _rt2D;

};
#endif
