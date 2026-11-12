#ifndef ELLIPSE_TARGET_PREDICTION_CONSTRAINTS_H
#define ELLIPSE_TARGET_PREDICTION_CONSTRAINTS_H

#include <math.h> 
#include <Eigen/Core>
#include <Eigen/Dense>

using namespace Eigen;
// if true -> the motion has moved past the cluster and
// thus the cluster canNOT be a potential target
bool ptDistFromCluster2D (MatrixXd inPt, MatrixXd clusterCentroid) {

	Vector2d inPt2D = Vector2d::Zero(2,1);

	inPt2D(0,0) = inPt(0,0);
	inPt2D(1,0) = inPt(1,0);
	double 	inPt2dDist
		= inPt2D.norm();
		
	Vector2d cluster2d = Vector2d::Zero(2,1);
	cluster2d(0,0) 
		= clusterCentroid(0,0);
	cluster2d(1,0) 
		= clusterCentroid(1,0);
	double cluster2dDist = cluster2d.norm();

	// cout << "lastInPtRThSnippet2dDist: " << lastInPtRThSnippet2dDist << endl;
	// cout << "cluster2dDist: " << cluster2dDist << endl;


	if (cluster2dDist > inPt2dDist)  
		return true;
		
	return false;	

}


bool isHandOnLeftHalfYplane (MatrixXd inPt, MatrixXd clustersCentroidAlongCol) {

	// constant integers for directions
	const int RIGHT = 1, LEFT = -1, ZERO = 0;

	// Y component of the centroid
	double yCentroid = clustersCentroidAlongCol(1,0);
	double yPt = inPt(1,0);
	
	
//	cout << "yCentroid = " << yCentroid << endl;
	//hand on left side
	return yPt < yCentroid ;
	
/*	
	//hand on right side
	//eliminate clusters 1 & 7
	if (yPt > yCentroid)
	
	
	clusterN == 0 || clusterN == 6 
	//hand on left side
	//eliminate clusters 3 & 9
	if (yPt < yCentroid)
	*/

}

MatrixXd linearVel (MatrixXd inPts) {

	MatrixXd linVel = MatrixXd::Zero(3,inPts.cols()-1);
	for (size_t i = 0; i < inPts.cols()-1; i++) {
		linVel(0,i) = inPts(0,i+1) - inPts(0,i);
		linVel(1,i) = inPts(1,i+1) - inPts(1,i);
		linVel(2,i) = inPts(2,i+1) - inPts(2,i);
	}
	return linVel;
}

#endif
