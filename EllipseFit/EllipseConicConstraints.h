#ifndef ELLIPSE_CONIC_CONSTRAINTS_H
#define ELLIPSE_CONIC_CONSTRAINTS_H

#include "FitFunctions.h"
#include "Plane.h"
#include "Ellipse.h"
#include "UBCUtil.h"

#include <Eigen/Core>

#include "ceres/ceres.h"
#include "gflags/gflags.h"
#include "glog/logging.h"

using ceres::AutoDiffCostFunction;
using ceres::CauchyLoss;
using ceres::CostFunction;
using ceres::LossFunction;
using ceres::Problem;
using ceres::Solve;
using ceres::Solver;

struct EllipseConicConstraints {
	EllipseConicConstraints(double x, double y) :
		_x(x), _y(y) {}

	template <typename U>
	
	//F(x,y)=ax2+bxy+cy2+dx+ey+f=0,
	//solve for conivVect = [a, b, c, d, e, f]
	bool operator()(const U* const conicConstraints, U* residuals) const {
		
		/* Each residual depends on the 6 conic constraints */
    residuals[0] = U (_x * _x) * conicConstraints[0] 
			+  U (_x * _y) * conicConstraints[1] 
			+  U (_y * _y) * conicConstraints[2] 
			+  U (_x) * conicConstraints[3] 
			+  U (_y) * conicConstraints[4] 
			+  conicConstraints[5];

		// 4ac - b^2 = 1.
	    residuals[1] = U (4.0) * conicConstraints[0] 
			* conicConstraints[2]	- conicConstraints[1] 
			* conicConstraints[1] - U (1.0);
   
		return true;
	}
	/* creating a CostFunction for every observation. */
	static ceres::CostFunction* Create(double x, double y) {
			return (
				new ceres::AutoDiffCostFunction
					<EllipseConicConstraints, 2, 6> (
				new EllipseConicConstraints(x, y)) 
		);
	}

	double _x, _y;

};

struct EllipseConicConstraintsSubjTo {
	EllipseConicConstraintsSubjTo(double x, double y) :
		_x(x), _y(y) {}

	template <typename U>
	
	//F(x,y)=ax2+bxy+cy2+dx+ey+f=0,
	//solve for conivVect = [a, b, c, d, e, f]
	bool operator()(const U* const conicConstraints, U* residuals) const {
		// 4ac - b^2 = 1.
    	residuals[0] = U (4.0) * conicConstraints[0] 
			* conicConstraints[2]	- conicConstraints[1] 
			* conicConstraints[1] - U (1.0);
   
   
   
/*	
	bool operator()(const U* const a, const U* const b
	, const U* const c, U* residuals) const {
	

		// 4ac - b^2 = 1.
 	  residuals[0] = U (4.0) * a[0]*c[0]	- b[0]* b[0] - U (1.0);
  */	
		return true;
	}
	/* creating a CostFunction for every observation. */
	static ceres::CostFunction* Create(double x, double y) {
			return (
				new ceres::AutoDiffCostFunction
					<EllipseConicConstraintsSubjTo, 1,6> (
				new EllipseConicConstraintsSubjTo(x, y)) 
		);
	}

	double _x, _y;

};



/*
struct EllipseConicConstraints {
	EllipseConicConstraints(double x, double y) :
		_x(x), _y(y) {}

	template <typename U>
	
	//F(x,y)=ax2+bxy+cy2+dx+ey+f=0,
	//solve for conivVect = [a, b, c, d, e, f]
	bool operator()(const U* const conicConstraints, U* residuals) const {
		
		/Each residual depends on the 6 conic constraints
		residuals[0] = U (_x * _x) * conicConstraints[0] 
			+  U (_x * _y) * conicConstraints[1] 
			+  U (_y * _y) * conicConstraints[2] 
			+  U (_x) * conicConstraints[3] 
			+  U (_y) * conicConstraints[4] 
			+  conicConstraints[5];

		// 4ac - b^2 = 1.
    		residuals[1] = U (4.0) * conicConstraints[0] 
			* conicConstraints[2]	- conicConstraints[1] 
			* conicConstraints[1] - U (1.0);
   
		return true;
	}
	//creating a CostFunction for every observation. 
	static ceres::CostFunction* Create(double x, double y) {
			return (
				new ceres::AutoDiffCostFunction
					<EllipseConicConstraints, 2, 6> (
				new EllipseConicConstraints(x, y)) 
		);
	}

	double _x, _y;

};
*/


#endif 
