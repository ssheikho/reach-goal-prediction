#include "ParseMathematica.h"
#include "ParseCSV.h"
#include "UBCUtil.h"
#include "FitFunctions.h"
#include "Ellipse3D.h"
#include "Plane.h"
#include "LinearAlgebra.cpp"
#include "RigidTrans2D.h"

#include "BaysFitFunctions.h"
#include "EllipseConicConstraints.h"

#include "DifferentialIKErrorTerm.h"
//#include "DikProblem.h"
//#include "DikSolver.h"

#include "ClusterFit.h"

#include "EllipseTargetPredictionConstraints.h"

#include <math.h> 
#include <Eigen/Core>
#include <Eigen/Dense>'
#pragma GCC diagnostic ignored "-Wunused-parameter"
//#include <Eigen/SVD>

//#include <cmath>
#include <iostream>
class DikProblem;

DikProblem *dIkProbInChest;

using namespace Eigen;
using namespace std;

enum clusterCentroids {C1,C2,C3,C4,C5,C6,};
//ensure theta is within [-pi,pi], 
//	&& theta(t) is a monotonic function 
MatrixXd fixThetas(MatrixXd a) {
    MatrixXd retVal(a.rows(), a.cols());

    // Normalize the first angle
    double curTheta = a(0, 0);
    while (curTheta < 0) curTheta += 2.0 * M_PI;
    while (curTheta >= 2.0 * M_PI) curTheta -= 2.0 * M_PI;
    retVal(0, 0) = curTheta;

    // Adjust subsequent angles for continuity
    for (size_t i = 1; i < a.cols(); i++) {
        curTheta = a(0, i);
        while (curTheta < 0) curTheta += 2.0 * M_PI;
        while (curTheta >= 2.0 * M_PI) curTheta -= 2.0 * M_PI;

        double testA = curTheta + 2.0 * M_PI;
        double testB = curTheta - 2.0 * M_PI;

        double distNull = std::abs(curTheta - retVal(0, i - 1));
        double distA = std::abs(testA - retVal(0, i - 1));
        double distB = std::abs(testB - retVal(0, i - 1));

        // Choose the angle that minimizes the distance
        if (distNull <= distA && distNull <= distB) {
            retVal(0, i) = curTheta;
        } else if (distA <= distNull && distA <= distB) {
            retVal(0, i) = testA;
        } else {
            retVal(0, i) = testB;
        }
    }

    return retVal;
}


int main(int argc, char **argv) {
		
		
		
		
	string fileName = argv[1];
	//cout << "(* " << fileName << "*)" << endl;
/*************************************/
/* H1.A. Ellipse Fitting Algorith */
/**********************************************/


	/*** Reading input CSV Motion Files ***/
	std::vector<std::string> wristMarkers =
 		{"x", "y", "z"};
	// Copy input marker data into Eigen Matrix 
	std::vector<std::string> wSMarkers =	
	{ "RWristX",	"RWristY",	"RWristZ" 
	, "RThumbX",	"RThumbY",	"RThumbZ" 
	, "RLArmX",	"RLArmY",	"RLArmZ"		
	, "RElbX",	"RElbY",	"RElbZ"				
	, "RUArmX",	"RUArmY",	"RUArmZ"		
	, "RShoX",	"RShoY",	"RShoZ"
	, "RChestX",	"RChestY",	"RChestZ"	
	, "MChestX",	"MChestY",	"MChestZ"
	, "RChestX", "RChestY", "RChestZ"
	, "RShoX", "RShoY", "RShoZ" 
	, "RPinkieX",	"RPinkieY",	"RPinkieZ" };
	
	
	int nMarkers = wSMarkers.size();
	// nx3
	std::string metaFileName;

	// (1) PARTICIPANT ID
	string id = getPartID(fileName);

	
	// (2) TimeFrame
	int startF = 0;
	int endF = 0; 
	getStartFrame(fileName, startF, endF);
	

	// (3) Phase
	int phase = 
		whichPhase(fileName);		
	string phaseS = to_string(phase);
	// Calculating the length of string 
	metaFileName = 
		fileName.substr(0,fileName.find("_clean"))+".csv";	
/*OLD DATA*/
	string headerLine = 
		getHeaderRowCSV(metaFileName);
		

		
/*NEW DATA	
	string headerLine = 
		getHeaderRowCSV(fileName);
		
*/	

	
	// (4) Reach Count
	std::string reachCtr = argv[2];

/*OLD DATA	*/
	// (5) reach sample size (n)
	int nPts = countRowsCSV(fileName, wristMarkers[0]);
	
	MatrixXd inPtsAlongRows = MatrixXd::Zero(nPts,nMarkers);
	int nPtsTotal = countRowsCSV(
		metaFileName, "RChestX");
		
	MatrixXd inPtsAlongRowsTotal = 
		MatrixXd::Zero(nPtsTotal,nMarkers);
		

/*NEW DATA
	// (5) reach sample size (n)
	int nPts = countRowsCSV(fileName,  wSMarkers[0])-1;
	MatrixXd inPtsAlongRows = MatrixXd::Zero(nPts,nMarkers);

*/
	
	
	
		
//	string framesFile = argv[2];
//	fillCSVMat(framesFile, metaFileName, wSMarkers);
	
	
//	int rowS = 0;
//	int rowF = 0;
//	fromToWhichFrame (2, framesFile, rowS, rowF);	
	
//	int nPts = rowF - rowS;
//	MatrixXd inPtsAlongRows = MatrixXd::Zero(nPts,nMarkers);

	// (6) Read csv reach data into Eigen format

/*NEW DATA	
	// read csv reach data into Eigen format
	int ctr = 0;
	while ((ctr < nMarkers)) {
		int startMarkerDelim = 
			headerLine.find(wSMarkers[ctr]);
		if (startMarkerDelim!=std::string::npos) { 
			
		//	fillMatCSVFromRowToRow(ctr, metaFileName
		//		, wSMarkers[ctr], wSMarkers[ctr+1]
		//		, wSMarkers[ctr+2], inPtsAlongRows
		//		, rowS, rowF);
	
			fillMatCSV(ctr, fileName
				, wSMarkers[ctr], wSMarkers[ctr+1]
				, wSMarkers[ctr+2], inPtsAlongRows);
		
		}
		ctr +=3;
	}
	
		
*/



/*OLD DATA	*/
	// read csv reach data into Eigen format
	int ctr = 0;
	while ((ctr < nMarkers)) {
		int startMarkerDelim = 
			headerLine.find(wSMarkers[ctr]);
		if (startMarkerDelim!=std::string::npos) { 
		
			fillMatCSV(ctr, metaFileName
				, wSMarkers[ctr], wSMarkers[ctr+1]
				, wSMarkers[ctr+2], inPtsAlongRowsTotal);
		}
		ctr +=3;
	}


	if (phase ==1)	
		inPtsAlongRows = inPtsAlongRowsTotal.topRows(nPts);
	
	else if (phase == 2)	{
		inPtsAlongRows = inPtsAlongRowsTotal.block(nPtsTotal-nPts-1,0,
		nPts,nMarkers);
		//inPtsAlongRows = inPtsAlongRowsTotal.bottomRows(nPts);
		
	}
	else {
		cout << 
			"Error! can't determine the motion Phase" 
		<< endl;
	}
	
	
/**/


	// positions in meter (nMarkers x nPts)
	MatrixXd inPts = 0.01 * inPtsAlongRows.transpose();
	
	//newly annotated data are in mm-> *0.1 to convert to cm
	// Old annotated data are in cm-> no need for conversion
	//MatrixXd inPts = 0.1 * inPtsAlongRows.transpose();
	//MatrixXd inPts = inPtsAlongRows.transpose();
	
	
	
	// Vicon inPts (Same as human, i.e. no ellipse fitting)
	MatrixXd inPtsRSh = inPts.block(Markers::RSh,0,3,nPts)
		, inPtsRCh = inPts.block(Markers::RCh,0,3,nPts)
		,	inPtsMCh = inPts.block(Markers::MCh,0,3,nPts)
		,	inPtsLCh = inPts.block(Markers::LCh,0,3,nPts)
		, inPtsLSh = inPts.block(Markers::LSh,0,3,nPts)

	// ellipse fitted pts
		,	inPtsRPi = inPts.block(Markers::RPi,0,3,nPts)
		,	inPtsRTh = inPts.block(Markers::RTh,0,3,nPts)
		,	inPtsRWr = inPts.block(Markers::RWr,0,3,nPts)
		,	inPtsRLA = inPts.block(Markers::RLA,0,3,nPts)
		,	inPtsREl = inPts.block(Markers::REl,0,3,nPts)
		,	inPtsRUA = inPts.block(Markers::RUA,0,3,nPts);

	//(6) target inPts (end of reach)
	MatrixXd  inTargetPtRPi = inPtsRPi.rightCols(3).rowwise().mean()
		, inTargetPtRTh = inPtsRTh.rightCols(3).rowwise().mean()
		, inTargetPtRWr = inPtsRWr.rightCols(3).rowwise().mean()
		, inTargetPtRLA = inPtsRLA.rightCols(3).rowwise().mean()
		, inTargetPtREl = inPtsREl.rightCols(3).rowwise().mean()
		, inTargetPtRUA = inPtsRUA.rightCols(3).rowwise().mean();
	// drop the z axes -> polar
	MatrixXd  inTargetPolarRPi = twoDCartToPolar(inTargetPtRPi.topRows(2));
	MatrixXd  inTargetPolarRTh = twoDCartToPolar(inTargetPtRTh.topRows(2));
	MatrixXd  inTargetPolarRWr = twoDCartToPolar(inTargetPtRWr.topRows(2));
	MatrixXd  inTargetPolarRLA = twoDCartToPolar(inTargetPtRLA.topRows(2));
	MatrixXd  inTargetPolarREl = twoDCartToPolar(inTargetPtREl.topRows(2));
	MatrixXd  inTargetPolarRUA = twoDCartToPolar(inTargetPtRUA.topRows(2));
	
	
	/*** END - Reading input CSV Motion Files ***/
	/************************************************/
	
	/* Find Which Cluster The Rach (Target) Belongs to */
 		 ClusterFit cfRth(inTargetPtRTh);
 		 int clusterId = cfRth.whichCluster();
 		 
 	
 		 /*	START - A.1 Finding Nominal Plane Fit 
 		 v1,v2 & v2 correspond to three eigenvectors of the plane
 		 v1 and v2 dictate the x y coordinates of the plane (i.e.
 		 the plane direction)
 		 v3 dictates the plane normal vector
 		 **/ 
 		 
 		 
	//(a) ShoToUa
	Plane bfpUa(cartToHom(inPtsRUA));
	Vector3d bfpUaV1 = bfpUa.getPlaneV1N()
		, bfpUaV2 = bfpUa.getPlaneV2N()
		, bfpUaV3 = bfpUa.getPlaneNormalVectN();
	// plane angle in radians 	
	double bfpUaNtoZ = bfpUa.getPlaneNtoZ();
	
	//(b) shoToEl
	Plane bfpEl(cartToHom(inPtsREl));
	Vector3d bfpElV1 = bfpEl.getPlaneV1N()
		, bfpElV2 = bfpEl.getPlaneV2N()
		, bfpElV3 = bfpEl.getPlaneNormalVectN();
	double bfpElNtoZ = bfpEl.getPlaneNtoZ();
	
	//(c) shoToLa
	Plane bfpLa(cartToHom(inPtsRLA));
	Vector3d bfpLaV1 = bfpLa.getPlaneV1N()
		, bfpLaV2 = bfpLa.getPlaneV2N()
		, bfpLaV3 = bfpLa.getPlaneNormalVectN();
	double bfpLaNtoZ = bfpLa.getPlaneNtoZ();
		
	//(d) shoToWr
	Plane bfpWr(cartToHom(inPtsRWr));
	Vector3d bfpWrV1 = bfpWr.getPlaneV1N()
		, bfpWrV2 = bfpWr.getPlaneV2N()
		, bfpWrV3 = bfpWr.getPlaneNormalVectN();
	double bfpWrNtoZ = bfpWr.getPlaneNtoZ();
	
	//(e) shoToTh
	Plane bfpTh(cartToHom(inPtsRTh));
	Vector3d bfpThV1 = bfpTh.getPlaneV1N()
		, bfpThV2 = bfpTh.getPlaneV2N()
		, bfpThV3 = bfpTh.getPlaneNormalVectN();
	double bfpThNtoZ = bfpTh.getPlaneNtoZ();
		
		
	//(f) shoToPi
	Plane bfpPi(cartToHom(inPtsRPi));
	Vector3d bfpPiV1 = bfpPi.getPlaneV1N()
		, bfpPiV2 = bfpPi.getPlaneV2N()
		, bfpPiV3 = bfpPi.getPlaneNormalVectN();
	double bfpPiNtoZ = bfpPi.getPlaneNtoZ();
	
	
	
	/**	END - A.1 Finding Nominal Plane Fit **/ 	
	/************************************************/	

	/*** START - A.2 ELLIPSE FITTING ***/
	/*** obtain ellipse parameters 
		 	a, b & alpha 	from (2d)ellipse
		    center coordinates from the ellipse3d
	***/
	/*	*/
	//(a) ShoToUa
	Ellipse3D e3dUa(cartToHom(inPtsRUA),true);
	//alpha; i.e. ellipse rotation in best-fit plane
	double e2dAlpha = 
	e3dUa.getEllipse().getRT2D().getTheta();
	//a
	double e2dUaA = e3dUa.getEllipse().getA(); 
	//b
	double e2dUaB = e3dUa.getEllipse().getB();
	//center
	MatrixXd e3dUaCenter = e3dUa.getCenter(); 
	//thetas
	MatrixXd thetasUa = e3dUa.findThetas();
	//ellipse-Fitted points
	MatrixXd outPtsRUA = homToCart(
		e3dUa.getPointAtThetasH(fixThetas(thetasUa)));
		
	//(b) shoToEl
	Ellipse3D e3dEl(cartToHom(inPtsREl),true);
	double e2dElA = e3dEl.getEllipse().getA();
	double e2dElB = e3dEl.getEllipse().getB();
	MatrixXd e3dElCenter = e3dEl.getCenter(); 
	MatrixXd thetasEl = e3dEl.findThetas();
	MatrixXd outPtsREl = homToCart(
		e3dEl.getPointAtThetasH(fixThetas(thetasEl)));
		
	//(c) shoToLa
	Ellipse3D e3dLa(cartToHom(inPtsRLA),true);
	double e2dLaA = e3dLa.getEllipse().getA();
	double e2dLaB = e3dLa.getEllipse().getB();
	MatrixXd e3dLaCenter = e3dLa.getCenter(); 
	MatrixXd thetasLa = e3dLa.findThetas();
	MatrixXd outPtsRLA = homToCart(
		e3dLa.getPointAtThetasH(fixThetas(thetasLa)));
		
	//(d) shoToWr
	Ellipse3D e3dWr(cartToHom(inPtsRWr),true);
	MatrixXd thetasWr = e3dWr.findThetas();
	MatrixXd fixedThetasWr = fixThetas(thetasWr);
	double e2dWrA = e3dWr.getEllipse().getA();
	double e2dWrB = e3dWr.getEllipse().getB();
	MatrixXd e3dWrCenter = e3dWr.getCenter(); 
	MatrixXd outPtsRWr = homToCart(
		e3dWr.getPointAtThetasH(thetasWr));
		
		
	//(e) shoToTh
	
	//Ellipse3D e3dTh(cartToHom(inPtsRTh));
	Ellipse3D e3dTh(cartToHom(inPtsRTh),true);
	double e2dThAlpha = 
		e3dTh.getEllipse().getRT2D().getTheta();
	double e2dThA = e3dTh.getEllipse().getA();
	double e2dThB = e3dTh.getEllipse().getB();
	MatrixXd e3dThCenter = e3dTh.getCenter();
	
	double c2dX =  e3dTh.getEllipse().getRT2D().getCX();
	double c2dY =  e3dTh.getEllipse().getRT2D().getCY();
	
	
	MatrixXd thetasTh = e3dTh.findThetas();
	MatrixXd outPtsRTh = homToCart(
		e3dTh.getPointAtThetasH(fixThetas(thetasTh)));	
				
				
	MatrixXd ellipse3dRTh = homToCart(
			e3dTh.ellipticalInterpolator(
					inPtsRTh.cols(),0.0, M_PI * 2.0));	
//	printEigenMathematica( fixedThetasWr.transpose(), cout
//		, "fixedThetasWr");	
		
		
	
	/**/
			
	//(f) shoToPi
	Ellipse3D e3dPi(cartToHom(inPtsRPi),true);
	double e2dPiA = e3dPi.getEllipse().getA();
	double e2dPiB = e3dPi.getEllipse().getB();
	MatrixXd e3dPiCenter = e3dPi.getCenter(); 
	MatrixXd thetasPi = e3dPi.findThetas();
	MatrixXd outPtsRPi = homToCart(
		e3dPi.getPointAtThetasH(fixThetas(thetasPi)));
	
	
	MatrixXd efPts = MatrixXd::Zero(nMarkers,nPts);
	
	
	// Same as human, i.e. no ellipse fitting
	efPts.block(Markers::RSh,0,3,nPts) = inPtsRSh;
	efPts.block(Markers::RCh,0,3,nPts) = inPtsRCh;
	efPts.block(Markers::MCh,0,3,nPts) = inPtsMCh;
	efPts.block(Markers::LCh,0,3,nPts) = inPtsLCh;
	efPts.block(Markers::LSh,0,3,nPts) = inPtsLSh;

//	efPts.block(Markers::RWr,0,3,nPts) = outPtsRWr;
//	efPts.block(Markers::RTh,0,3,nPts) = outPtsRTh;
/**/

	// Fitted ellipse pts
	efPts.block(Markers::RUA,0,3,nPts) = outPtsRUA;
	efPts.block(Markers::REl,0,3,nPts) = outPtsREl;
	efPts.block(Markers::RLA,0,3,nPts) = outPtsRLA;
	efPts.block(Markers::RWr,0,3,nPts) = outPtsRWr;
	efPts.block(Markers::RTh,0,3,nPts) = outPtsRTh;
	efPts.block(Markers::RPi,0,3,nPts) = outPtsRPi;
	
	
/*	
	// Fitted ellipse pts
	efPts.block(Markers::RUA,0,3,nPts) = inPtsRUA;
	efPts.block(Markers::REl,0,3,nPts) = inPtsREl;
	efPts.block(Markers::RLA,0,3,nPts) = inPtsRLA;
	efPts.block(Markers::RWr,0,3,nPts) = inPtsRWr;
	efPts.block(Markers::RTh,0,3,nPts) = inPtsRTh;
	efPts.block(Markers::RPi,0,3,nPts) = inPtsRPi;
*/

	printEigenMathematica( inPtsRSh.transpose(), cout
		, "inPtsRSh");	
	printEigenMathematica( inPtsRCh.transpose(), cout
		, "inPtsRCh");	
	printEigenMathematica( inPtsMCh.transpose(), cout
		, "inPtsMCh");	
	printEigenMathematica( inPtsLCh.transpose(), cout
		, "inPtsLCh");	
	printEigenMathematica( inPtsLSh.transpose(), cout
		, "inPtsLSh");	
		
	// Print input pts and ellipseFit pts to Mathematica
	printEigenMathematica( inPtsRUA.transpose(), cout
		, "inPtsRUA");	

	printEigenMathematica( inPtsREl.transpose(), cout
		, "inPtsREl");	

	printEigenMathematica( inPtsRLA.transpose(), cout
		, "inPtsRLA");	

	printEigenMathematica( inPtsRWr.transpose(), cout
		, "inPtsRWr");	

	printEigenMathematica( inPtsRTh.transpose(), cout
		, "inPtsRTh");	

	printEigenMathematica( inPtsRPi.transpose(), cout
		, "inPtsRPi");	
		
		
	printEigenMathematica( outPtsRUA.transpose(), cout
		, "outPtsRUA");	

	printEigenMathematica( outPtsREl.transpose(), cout
		, "outPtsREl");	

	printEigenMathematica( outPtsRLA.transpose(), cout
		, "outPtsRLA");	

	printEigenMathematica( outPtsRWr.transpose(), cout
		, "outPtsRWr");	

	printEigenMathematica( outPtsRTh.transpose(), cout
		, "outPtsRTh");	

	printEigenMathematica( outPtsRPi.transpose(), cout
		, "outPtsRPi");	
		
		
	// EllipseFit error
	MatrixXd 
		fitErrUA = inPtsRUA - outPtsRUA,
		fitErrEl = inPtsREl - outPtsREl,
		fitErrLA = inPtsRLA - outPtsRLA,
		fitErrWr = inPtsRWr - outPtsRWr,
		fitErrTh = inPtsRTh - outPtsRTh,
		fitErrPi = inPtsRPi - outPtsRPi;
		
		
	// mean error
	double 
  		  fitErrUAMean = fitErrUA.colwise().norm().mean() 
		, fitErrElMean = fitErrEl.colwise().norm().mean() 
		, fitErrLAMean = fitErrLA.colwise().norm().mean() 
		, fitErrWrMean = fitErrWr.colwise().norm().mean() 
		, fitErrThMean = fitErrTh.colwise().norm().mean() 
		, fitErrPiMean = fitErrPi.colwise().norm().mean() ;
/**/		
	//EF modeling fit error	
	cout << "fitErrPi Mean = " 
		 << fitErrPi.colwise().norm().mean() << endl;
	cout << "fitErrTh Mean = " 
		 << fitErrTh.colwise().norm().mean() << endl;
	cout << "fitErrWr Mean = " 
		 << fitErrWr.colwise().norm().mean() << endl;
	cout << "fitErrLA Mean = " 
		 << fitErrLA.colwise().norm().mean() << endl;
	cout << "fitErrEl Mean = " 
		 << fitErrEl.colwise().norm().mean() << endl;
	cout << "fitErrUA Mean = " 
		 << fitErrUA.colwise().norm().mean() << endl;


	printEigenMathematica( efPts.transpose(), cout
		, "efPts");	
		
		
	/*** END - A.2 ELLIPSE FITTING ***/
	/************************************************/
	
	
	
	// calculate linear velocity (x,y,z) of the inpts
	MatrixXd linVelInPtsRTh = linearVel(inPtsRTh);



	/** decide whether to use wrist or thumb based on fit error
		and recalculate plane fit and ellipse fit parameters
		for print in excel
	**/	
	MatrixXd inPtsBf = 
	fitErrThMean < fitErrWrMean ? inPtsRTh: inPtsRWr;
	
	
	//A. best-fit plane parameters
	Plane bfpInPtsBf(cartToHom(inPtsBf));
	Vector3d bfpInPtsBfV1 = bfpInPtsBf.getPlaneV1N()
		, bfpInPtsBfV2 = bfpInPtsBf.getPlaneV2N()
		, bfpInPtsBfV3 = bfpInPtsBf.getPlaneNormalVectN();
	double bfpInPtsBfNtoZ = bfpInPtsBf.getPlaneNtoZ();
	Vector3d bfpInPtsBfEigenVals = bfpInPtsBf.getPlaneEigenVals();
	
	
	//B. ellipse-fit parameters
	Ellipse3D e3dInPtsBf(cartToHom(inPtsBf),true);
	double e2dInPtsBfAlpha = 
		e3dInPtsBf.getEllipse().getRT2D().getTheta();
	double e2dInPtsBfA = e3dInPtsBf.getEllipse().getA();
	double e2dInPtsBfB = e3dInPtsBf.getEllipse().getB();
	MatrixXd e3dInPtsBfCenter = e3dInPtsBf.getCenter();
	
	double c2dInPtsBfX =  e3dInPtsBf.getEllipse().getRT2D().getCX();
	double c2dInPtsBfY =  e3dInPtsBf.getEllipse().getRT2D().getCY();
	
	
	MatrixXd thetasInPtsBf = e3dInPtsBf.findThetas();
	MatrixXd outPtsBf = homToCart(
		e3dInPtsBf.getPointAtThetasH(fixThetas(thetasInPtsBf)));	
				
	
	MatrixXd fitErrPtsBf = inPtsBf - outPtsBf;
	double fitErrPtsBfMean = fitErrPtsBf.colwise().norm().mean();	
	/*
	
	printEigenMathematica( inPtsRTh.transpose(), cout
		, "inPtsRTh");	

	printEigenMathematica( outPtsRTh.transpose(), cout
		, "inPtsRThEf");	
	printEigenMathematica( ellipse3dRTh.transpose(), cout
		, "ellipse3dRTh");
		
	printEigenMathematica( inTargetPtRTh.transpose(), cout
		, "inTargetPtRTh");		
		*/
	
	/*** 		END - 		***/
	/************************************************/
	
	
		
	/*** FOR PRINT IN CSV FORMAT **

	//PICK 4 CORNER CLUSTERS
	if (clusterId == 1 || clusterId == 3
		|| clusterId == 7 || clusterId == 9)	{

	cout << id << ",";
	cout << startF << "_" << endF << ",";

	cout << phaseS << ",";
	cout << reachCtr << ",";

	cout << nPts << ",";	
	cout << inTargetPtRTh(0,0) <<","
		 << inTargetPtRTh(1,0) <<","
		 << inTargetPtRTh(2,0) <<",";	
		 
		 
	cout << inTargetPolarRTh(0,0) <<","
		 << inTargetPolarRTh(1,0) <<",";
		 
	
	cout << bfpThNtoZ <<",";
	 	 
	
	cout << fitErrWrMean <<",";
	cout << fitErrThMean <<",";
	
	cout << clusterId <<",";	
	
	
	cout << e2dThAlpha <<",";
	cout << e2dThA <<",";
	cout << e2dThB <<",";
	cout << e2dThA / e2dThB <<",";
        cout << c2dX << "," 
       	<<c2dY  << ",";
        	
 //       cout << endl;	
	*/
	
	
	size_t sliceN = 25.0;
	double sliceSize = inPtsRTh.cols()/sliceN;
	//double sliceSize = std::ceil(double(inPtsRTh.cols()) / sliceN);

	int TargetWeight = 5;
	int nClusters = cfRth.nClusters();
	MatrixXd clusterCentroidsAlongCols 
		= cfRth.getClusterCentroidsAlongCols();
		
	VectorXd fitErrpredPtsRThSnippetMean 
		= VectorXd::Zero(nClusters);	
	MatrixXd sortedFitErrpredPtsRThSnippetMean
		= MatrixXd::Zero(2,nClusters);
		
	//Velocity Constraint	
	MatrixXd lastInPtRThSnippet = MatrixXd::Zero(3,sliceN+1);
	double linXvelRThSnippet = 0.0;
//	MatrixXd linXvelRThSnippetMat = MatrixXd::Zero(1,sliceN+1);
	bool linXvelRThPastThreshold = false;
	double zOffsetRThSnippet = 0.0;
    
/*
// assign different clusters as the potential 
// target of the motion
for (size_t clusterN = 0; 
		clusterN < nClusters;
			clusterN++) {
				
	
*/


      	
      	vector <int> cornerClusterIds = {0,2,6, 8};
      	// {0,2,6,8};
      		
	lastInPtRThSnippet.col(0) = inPtsRTh.col(0);
			
			
	// trajectory snippet %
	for (size_t i = 1; i <= sliceN; i++) {
	//size_t i = 15;
		int snippetPercent = int(i*100.0/sliceN);
		
		MatrixXd inPtsRThSnippet =
			MatrixXd::Zero(3,i*sliceSize + TargetWeight);
			
		inPtsRThSnippet.leftCols(i*sliceSize)
				= inPtsRTh.leftCols(i*sliceSize);
			
			
		lastInPtRThSnippet.col(i) 
			= inPtsRThSnippet.col(i*sliceSize-1);
		
//	printEigenMathematica( lastInPtRThSnippet.transpose(), cout
//		, "lastInPtRThSnippet");
		
		MatrixXd inPtsRWrSnippet =
			MatrixXd::Zero(3,i*sliceSize + TargetWeight);
			
		inPtsRWrSnippet.leftCols(i*sliceSize)
				= inPtsRWr.leftCols(i*sliceSize);
					
		 // Vector to store element
    		// with respective present index
    		vector<pair<double, int> > vp, probablities;
    
		
		
		
	/**	Corner clusters correspond to 
  		1 -> bottom - left
  		3 -> bottom - right
  		7 -> top - left
  		9 -> top - right
  		clusters 1 & 7 on the left side
  		clusters 3 & 9 on the right side
       **/
      
      		
      		MatrixXd cornerClustersCentroid = 
      			cfRth.getCornerClustersCentroid();
      	
			//printEigenMathematica( cornerClustersCentroid.transpose(), cout
			//, "cornerClustersCentroid");	
      		
      		
      		// assign different clusters as the potential 
		// target of the motion
		
/*
		for (size_t clusterN = 0; 
		clusterN < nClusters;
			clusterN++) {
*/			

	/**/	
		for (auto cluster = begin(cornerClusterIds)
			; cluster != end (cornerClusterIds)
			; ++cluster) {
		
		
		size_t clusterN = *cluster;

	
				
		//size_t clusterN = 0;


/**/	

		 // CONSTRAINT 1
  		int c = i*sliceSize-1;
		bool distConstraint = 
			ptDistFromCluster2D(inPtsRThSnippet.col(c)
		 	, clusterCentroidsAlongCols.col(clusterN));	
		 // use wrist instead of the thumb 	
		 bool distConstraintWr = 
			ptDistFromCluster2D(inPtsRWrSnippet.col(c)
		 	, clusterCentroidsAlongCols.col(clusterN));	
		
		if (clusterId == 7 || clusterId == 9 ) {	
  			if (distConstraint)  
  				continue;
  		}
/*  		 		
		
		if (clusterId == 1 || clusterId == 3 ) {	
  			if (distConstraintWr)  
  				continue;
  		}
  		else {
  			if (distConstraint) 
  				continue;
  		}
*/  			
			
		 // CONSTRAINT 2
		bool handOnLefttHalfYplane =
			isHandOnLeftHalfYplane (inPtsRThSnippet.col(c)
			, cornerClustersCentroid);
		// start at 25% of the motion 2
		if(i > 4) {
	  		if(handOnLefttHalfYplane) {
	  			if (clusterN == 2 || clusterN == 8)
	  				continue;
	  		} else {
	  			if (clusterN == 0 || clusterN == 6)
	  				continue;
	  		}
  		}
			
			
 /*	*/	

		 // CONSTRAINT 3 - instantanous velocity at each snippet
		 
//		if (i < sliceN) {
			linXvelRThSnippet = abs(
				lastInPtRThSnippet(0,i) - lastInPtRThSnippet(0,i-1));
				//cout << "linXvelRThSnippet = " << linXvelRThSnippet << endl;
			if (linXvelRThSnippet > 4.30) {
	  			if (clusterN == 0 || clusterN == 2) {
	  				linXvelRThPastThreshold = true;
					 // erase clusters 0 & 2 from all the snippet loops
  					cornerClusterIds.erase 
  						(cornerClusterIds.begin());
  					cornerClusterIds.erase 
  						(cornerClusterIds.begin());
 // 
//for (auto i: cornerClusterIds)
  // std::cout << i << ' ';

	  				continue;
				}
			}
				
//		}
		
		
	
/*   
		// CONSTRAINT 4 - Z-OOFSET at each snippet
		
		zOffsetRThSnippet = lastInPtRThSnippet(2,i-1);
		// found in excel from part 10
		if (zOffsetRThSnippet > 19.8) {
			if (clusterN == 0 || clusterN == 2)
	  			continue;
		
		}
*/		
		
		// putting weight on the target point	
		for (size_t j = 0; j < TargetWeight; j++) 
			inPtsRThSnippet.col(i*sliceSize + j) = 
				clusterCentroidsAlongCols.col(clusterN);
				//inTargetPtRTh;
				
				
		// (1) Fit Ellipse to InptsSnippet
		Ellipse3D e3dThSnippet(
			cartToHom(inPtsRThSnippet),true);
		// (2) Find Thetas to InptsSnippet
		MatrixXd thetasThSnippet 
			= e3dThSnippet.findThetas();
	
		
					
		
		//double theta_i = M_PI / 2.0;
		//pair<double, double> a =
		//	e3dThSnippet.findOneThetaQuad(
		//		theta_i
		//		, inTargetPtRTh(0,0)
		//		,  inTargetPtRTh(1,0));
		
		//double thetaF = a.second;
		//MatrixXd efTargetPtRTh = 
		//	e3dThSnippet.getPointAtThetaH(thetaF);
		
			
		//(3) get outPts from Thetas
		MatrixXd predPtsRThSnippet = homToCart(
			e3dThSnippet.getPointAtThetasH(
				fixThetas(thetasThSnippet)));

double thetaStart = fixThetas(thetasThSnippet)(0, 0);
double thetaEnd = fixThetas(thetasThSnippet)(0, thetasThSnippet.cols() - 1);

// Normalize thetaStart and thetaEnd
thetaStart = std::fmod(thetaStart, 2.0 * M_PI);
thetaEnd = std::fmod(thetaEnd, 2.0 * M_PI);

// Ensure thetas are positive
if (thetaStart < 0) thetaStart += 2.0 * M_PI;
if (thetaEnd < 0) thetaEnd += 2.0 * M_PI;

// Handle wrap-around: if thetaStart > thetaEnd, assume the arc spans across 0
if (thetaStart > thetaEnd) {
    thetaEnd += 2.0 * M_PI;
}



//cout <<"thetaStart = " <<thetaStart<< ", thetaEnd = " << thetaEnd << endl;
		MatrixXd predEllipse3dRTh = homToCart(
			e3dThSnippet.ellipticalInterpolator(
				inPtsRTh.cols(),0.0, M_PI * 2.0));
				
		MatrixXd partialPredEllipse = homToCart(
   			e3dThSnippet.ellipticalInterpolator(
        			inPtsRTh.cols(), thetaStart, thetaEnd));

	
		//(6) target Pts (end of reach)
		//MatrixXd  predTargetPtTh  = 
		MatrixXd predTargetPtTh = 
			predPtsRThSnippet.rightCols(3).rowwise().mean();
			

		std::string percent = std::to_string(snippetPercent);

		
		
		 //printEigenMathematica( partialPredEllipse.transpose(), cout
		  //	, "predPtsRThC" + to_string(clusterN+1));
		
		
		//cout << "Cluster " << clusterN+1 << endl;
		
	/** Print motion snippet outputs in Mathematica		
	
		cout << "Cluster " << clusterN+1 << endl;
		//inPtsRThSnippet
		printEigenMathematica( inPtsRThSnippet.transpose(), cout
			, "inPtsRThSnippet" + to_string(snippetPercent) 
			+ "C" + to_string(clusterN+1));	
		//predPtsRThSnippet			
		printEigenMathematica( predPtsRThSnippet.transpose(), cout
			, "predPtsRThSnippet" + to_string(snippetPercent) 
			+ "C" + to_string(clusterN+1));	
		//predTargetPtTh
		printEigenMathematica( predTargetPtTh.transpose(), cout
			, "predTargetPtTh" + to_string(snippetPercent) 
			+ "C" + to_string(clusterN+1));
		
		 printEigenMathematica( predEllipse3dRTh.transpose(), cout
		  	, "predEllipse3dRTh"+ to_string(snippetPercent));
		
//			printEigenMathematica( efTargetPtRTh, cout
//		, "efTargetPtRTh");	
	**/


	
	
		


			
	
/**/	
			
		
		// Predict Which Cluster The Rach (Target) Belongs to  		
		// mean error
		double fitErrpredPtsRThSnippetMean = 
			(inPtsRThSnippet - predPtsRThSnippet).colwise().norm().mean();
	
	//	cout << "fitErrpredPts" + to_string(snippetPercent) 
	//		+ "C" + to_string(clusterN+1) + " = " 
	//		<< fitErrpredPtsRThSnippetMean << endl;
		
		

		// Inserting element in pair vector
    		// to keep track of previous indexes
    		vp.push_back(make_pair(fitErrpredPtsRThSnippetMean, clusterN));

		}


	// (1) Compute probabilities using residuals before sorting
	std::vector<double> residuals;
	for (const auto& pair : vp) {
    		residuals.push_back(pair.first); // Extract residuals (fit errors)
	}
	// Use lambda = 1.0, or adjust as needed
	std::vector<double> probabilities = computeProbabilities(residuals, 1.0); 

	// (2) Replace residuals in `vp` with probabilities
	for (size_t i = 0; i < vp.size(); ++i) {
		vp[i].first = probabilities[i]; // Replace residuals with probabilities
	}

// (3) Sort `vp` by probabilities in descending order
std::sort(vp.begin(), vp.end(), [](const auto& a, const auto& b) {
    return a.first > b.first; // Sort probabilities from max to min
});

// (4) Display sorted probabilities for debugging
std::cout << "Snippet Percent: " << snippetPercent << "%\n";
for (size_t i = 0; i < vp.size(); ++i) {
    std::cout << "Cluster " << vp[i].second + 1 
              << " Probability: " << vp[i].first << std::endl;
}
		
			
		
 		// order clusters from minErr to maxErr
		// minErr cluster is the chosen target
		// for this motion snippet
		//std::sort(vp.begin(), vp.end());
              
/*    
		// Displaying sorted element
		// with previous indexes
		// corresponding to each element
		//cout << "Slice " << i << "/" << sliceN << "->";
		cout << i << "/" << sliceN  << ",";
		cout << snippetPercent << "% : " << ",";
		
		
		for (int i = 0; i < vp.size(); i++) {
		   // cout << vp[i].first << "\t"
		    cout << vp[i].second +1 << ",";
		}
		
		cout << " " << endl;
		
		cout << "," << "," << "," << "," 
			 << "," << "," << "," << "," 
			 << "," << "," << "," << "," 
			 << "," << "," << "," << "," 
			 << "," << "," << "," << ",";
		
		
	
		
	
		//cout << "fitErrpredPtsRThSnippet Mean"
		///	 << i << " = ";
		//cout << fitErrpredPtsRThSnippetMean << endl;
		//cout <<" " << endl;

		
	}

	cout << "" << endl;
	*/
	
//	cout << "lastInPtRThSnippetZoffset = " 
//	cout	 << lastInPtRThSnippet.row(2).transpose() << endl;
	
		
/*	
	// calculate linear velocity (x,y,z) of the inpts
	MatrixXd linVelInPtsRThSnippet = linearVel(lastInPtRThSnippet);

//	printEigenMathematica( linVelInPtsRThSnippet.transpose(), cout
//		, "linVelInPtsRThSnippet");
//		
		
	MatrixXf::Index   maxVelIndexRThSnippet[1];


	// Max Velcity & INDEX AT MAX VELOCITY //
	double maxVelRThSnippet = 
		linVelInPtsRThSnippet.row(0).cwiseAbs().maxCoeff(
			&maxVelIndexRThSnippet[0] );

	cout << clusterId <<",";
	cout << maxVelIndexRThSnippet[0]+1 <<",";
	cout << maxVelRThSnippet << endl;


*/	
	}	
	return 0;
}


