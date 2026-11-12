#ifndef ELLIPSE_ALPHA_MIN_H
#define ELLIPSE_ALPHA_MIN_H

#include "Ellipse.h"
#include "UBCUtil.h"

#include "ceres/ceres.h"

//template <typename T>
struct EllipseAlphaMin {
	EllipseAlphaMin(
		double cX, double cY, double a, double b
		, double theta, double x, double y) :
		_cX(cX), _cY(cY), _a(a), _b(b), _theta(theta), _x(x), _y(y) {}

	template <typename U>
	//bool operator()(const U* const candidatePt, U* residuals) const {
	bool operator()(const U* const candidateAlpha, U* residuals) const {
		//_e.getA();//.getTheta();

		U x =  U(_cX) +  U(_a) * cos(candidateAlpha[0]) * cos(U(_theta)) -
			 U(_b) * sin(candidateAlpha[0]) * sin(U(_theta));
		U y =  U(_cY) +  U(_a) * cos(U(_theta)) * sin( candidateAlpha[0]) +
			 U(_b) * cos( candidateAlpha[0]) * sin(U(_theta));

	/* The error is the difference between 
		the predicted and observed position. */

    x = x - U(_x);
    y = y - U(_y);
   
		residuals[0] = sqrt(x * x + y * y);
		return true;
	}

	// Factory to hide the construction of the CostFunction object from
	// the client code.
	static ceres::CostFunction* Create(
		const double cX, const double cY
		, const double a, const double b
		, const double theta
		, const double x, const double y) {
			return (
				new ceres::AutoDiffCostFunction<EllipseAlphaMin, 1, 1>(
				new EllipseAlphaMin(cX, cY, a, b, theta, x, y)));
	}

	double _cX, _cY, _theta, _a, _b, _x, _y;
};

#endif 
