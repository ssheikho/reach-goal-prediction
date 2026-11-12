#ifndef CLUSTER_FIT_H
#define CLUSTER_FIT_H


#include <Eigen/Dense>
using namespace Eigen;


struct ClusterFit {
	
	
	ClusterFit(MatrixXd inTargetPt) :
		_nClusters(9)
		, _clusterCentroidsAlongCols(MatrixXd::Zero(3,_nClusters)) {
		
		_inTargetPt =inTargetPt;
		
		//	Centroids are numerically ordered from
		//	bottom-left to top-right 
		
		//** Cluster Centroids **//
		
		//***  START - NEW DATA  ***//

		//***  PART-17  ***//
/*		*/		
		//Cluster 1	
		_clusterCentroidsAlongCols(0,0) = 21.1246;		
		_clusterCentroidsAlongCols(1,0) = 64.8371;
		_clusterCentroidsAlongCols(2,0) = 16.4286;
		//Cluster 2
		_clusterCentroidsAlongCols(0,1) = 22.0743;		
		_clusterCentroidsAlongCols(1,1) = 74.6619;
		_clusterCentroidsAlongCols(2,1) = 16.4286;
		//Cluster 3
		_clusterCentroidsAlongCols(0,2) = 22.6511;				
		_clusterCentroidsAlongCols(1,2) = 84.3181;
		_clusterCentroidsAlongCols(2,2) = 16.4286;
		//Cluster 4
		_clusterCentroidsAlongCols(0,3) = 10.3917; 			
		_clusterCentroidsAlongCols(1,3) = 64.6965;
		_clusterCentroidsAlongCols(2,3) = 16.4286;
		//Cluster 5
		_clusterCentroidsAlongCols(0,4) = 11.3695;				
		_clusterCentroidsAlongCols(1,4) = 74.8374;
		_clusterCentroidsAlongCols(2,4) = 16.4286;
		//Cluster 6
		_clusterCentroidsAlongCols(0,5) = 11.7074;				
		_clusterCentroidsAlongCols(1,5) = 84.8125;
		_clusterCentroidsAlongCols(2,5) = 16.4286;
		//Cluster 7
		_clusterCentroidsAlongCols(0,6) = -0.0923087;				
		_clusterCentroidsAlongCols(1,6) = 64.13;
		_clusterCentroidsAlongCols(2,6) = 16.4286;
		//Cluster 8
		_clusterCentroidsAlongCols(0,7) = 0.533544;				
		_clusterCentroidsAlongCols(1,7) = 74.6798;
		_clusterCentroidsAlongCols(2,7) = 16.4286;
		//Cluster 9
		_clusterCentroidsAlongCols(0,8) = 0.648492;				
		_clusterCentroidsAlongCols(1,8) = 84.6955;
		_clusterCentroidsAlongCols(2,8) = 16.4286;

		//***  PART-14  ***//
/*		
		//Cluster 1	
		_clusterCentroidsAlongCols(0,0) = 24.2657;		
		_clusterCentroidsAlongCols(1,0) = 61.8287;
		_clusterCentroidsAlongCols(2,0) = 15.4159;
		//Cluster 2
		_clusterCentroidsAlongCols(0,1) = 24.0339;		
		_clusterCentroidsAlongCols(1,1) = 72.5223;
		_clusterCentroidsAlongCols(2,1) = 15.4159;
		//Cluster 3
		_clusterCentroidsAlongCols(0,2) = 24.054;				
		_clusterCentroidsAlongCols(1,2) = 83.3262;
		_clusterCentroidsAlongCols(2,2) = 15.4159;
		//Cluster 4
		_clusterCentroidsAlongCols(0,3) = 13.0667; 			
		_clusterCentroidsAlongCols(1,3) = 60.7769;
		_clusterCentroidsAlongCols(2,3) = 15.4159;
		//Cluster 5
		_clusterCentroidsAlongCols(0,4) = 13.7866;				
		_clusterCentroidsAlongCols(1,4) = 71.2454;
		_clusterCentroidsAlongCols(2,4) = 15.4159;
		//Cluster 6
		_clusterCentroidsAlongCols(0,5) = 13.0602;				
		_clusterCentroidsAlongCols(1,5) = 81.6128;
		_clusterCentroidsAlongCols(2,5) = 15.4159;
		//Cluster 7
		_clusterCentroidsAlongCols(0,6) = 3.35952;				
		_clusterCentroidsAlongCols(1,6) = 60.3493;
		_clusterCentroidsAlongCols(2,6) = 15.4159;
		//Cluster 8
		_clusterCentroidsAlongCols(0,7) = 2.99583;				
		_clusterCentroidsAlongCols(1,7) = 70.7675;
		_clusterCentroidsAlongCols(2,7) = 15.4159;
		//Cluster 9
		_clusterCentroidsAlongCols(0,8) = 2.33333;				
		_clusterCentroidsAlongCols(1,8) = 80.882;
		_clusterCentroidsAlongCols(2,8) = 15.4159;
		//_clusterCentroidsAlongCols *= 10.0;
	*/	
		//***  PART-9  ***//
/*		
		//Cluster 1	
		_clusterCentroidsAlongCols(0,0) = 18.4091;		
		_clusterCentroidsAlongCols(1,0) = 60.689;
		_clusterCentroidsAlongCols(2,0) = 15.37;
		//Cluster 2
		_clusterCentroidsAlongCols(0,1) = 18.7514;		
		_clusterCentroidsAlongCols(1,1) = 71.3663;
		_clusterCentroidsAlongCols(2,1) = 15.37;
		//Cluster 3
		_clusterCentroidsAlongCols(0,2) = 18.9106;				
		_clusterCentroidsAlongCols(1,2) = 81.3827;
		_clusterCentroidsAlongCols(2,2) = 15.37;
		//Cluster 4
		_clusterCentroidsAlongCols(0,3) = 7.70663; 			
		_clusterCentroidsAlongCols(1,3) = 60.7039;
		_clusterCentroidsAlongCols(2,3) = 15.37;
		//Cluster 5
		_clusterCentroidsAlongCols(0,4) = 8.05391;				
		_clusterCentroidsAlongCols(1,4) = 71.164;
		_clusterCentroidsAlongCols(2,4) = 15.37;
		//Cluster 6
		_clusterCentroidsAlongCols(0,5) = 7.956;				
		_clusterCentroidsAlongCols(1,5) = 81.2;
		_clusterCentroidsAlongCols(2,5) = 15.37;
		//Cluster 7
		_clusterCentroidsAlongCols(0,6) = -3.26561;				
		_clusterCentroidsAlongCols(1,6) = 60.0063;
		_clusterCentroidsAlongCols(2,6) = 15.37;
		//Cluster 8
		_clusterCentroidsAlongCols(0,7) = -3.05737;				
		_clusterCentroidsAlongCols(1,7) = 70.415;
		_clusterCentroidsAlongCols(2,7) = 15.6307;
		//Cluster 9
		_clusterCentroidsAlongCols(0,8) = -3.03931;				
		_clusterCentroidsAlongCols(1,8) = 80.3685;
		_clusterCentroidsAlongCols(2,8) = 15.37;
*/		
		
		
		//***  PART-11  ***//
/* 
		//Cluster 1	
		_clusterCentroidsAlongCols(0,0) = 22.0223; 		
		_clusterCentroidsAlongCols(1,0) = 57.4479;
		_clusterCentroidsAlongCols(2,0) = 15.6307;
		//Cluster 2
		_clusterCentroidsAlongCols(0,1) = 22.4129;		
		_clusterCentroidsAlongCols(1,1) = 68.6995;
		_clusterCentroidsAlongCols(2,1) = 15.6307;
		//Cluster 3
		_clusterCentroidsAlongCols(0,2) = 22.425;				
		_clusterCentroidsAlongCols(1,2) = 78.9139;
		_clusterCentroidsAlongCols(2,2) = 15.6307;
		//Cluster 4
		_clusterCentroidsAlongCols(0,3) = 11.092; 			
		_clusterCentroidsAlongCols(1,3) = 57.3457;
		_clusterCentroidsAlongCols(2,3) = 15.6307;
		//Cluster 5
		_clusterCentroidsAlongCols(0,4) = 11.3482;				
		_clusterCentroidsAlongCols(1,4) = 68.6739;
		_clusterCentroidsAlongCols(2,4) = 15.6307;
		//Cluster 6
		_clusterCentroidsAlongCols(0,5) = 11.2231;				
		_clusterCentroidsAlongCols(1,5) = 78.6501;
		_clusterCentroidsAlongCols(2,5) = 15.6307;
		//Cluster 7
		_clusterCentroidsAlongCols(0,6) = -0.70935;				
		_clusterCentroidsAlongCols(1,6) = 57.0384;
		_clusterCentroidsAlongCols(2,6) = 15.6307;
		//Cluster 8
		_clusterCentroidsAlongCols(0,7) = 0.42761;				
		_clusterCentroidsAlongCols(1,7) = 68.6653;
		_clusterCentroidsAlongCols(2,7) = 15.6307;
		//Cluster 9
		_clusterCentroidsAlongCols(0,8) = -0.369027;				
		_clusterCentroidsAlongCols(1,8) = 78.6752;
		_clusterCentroidsAlongCols(2,8) = 15.6307;
 */
 
		//***  PART-1  ***//
/*
		//Cluster 1	
		_clusterCentroidsAlongCols(0,0) = 19.6131; 		
		_clusterCentroidsAlongCols(1,0) = 66.1915;
		_clusterCentroidsAlongCols(2,0) = 13.2004;
		//Cluster 2
		_clusterCentroidsAlongCols(0,1) = 21.0249;		
		_clusterCentroidsAlongCols(1,1) = 76.1611;
		_clusterCentroidsAlongCols(2,1) = 13.2004;
		//Cluster 3
		_clusterCentroidsAlongCols(0,2) = 21.9293;				
		_clusterCentroidsAlongCols(1,2) = 86.1882;
		_clusterCentroidsAlongCols(2,2) = 13.2004;
		//Cluster 4
		_clusterCentroidsAlongCols(0,3) = 9.02406; 			
		_clusterCentroidsAlongCols(1,3) = 65.6372;
		_clusterCentroidsAlongCols(2,3) = 13.2004;
		//Cluster 5
		_clusterCentroidsAlongCols(0,4) = 10.4407;				
		_clusterCentroidsAlongCols(1,4) = 76.0767;
		_clusterCentroidsAlongCols(2,4) = 13.2004;
		//Cluster 6
		_clusterCentroidsAlongCols(0,5) = 11.0811;				
		_clusterCentroidsAlongCols(1,5) = 86.1263;
		_clusterCentroidsAlongCols(2,5) = 13.2004;
		//Cluster 7
		_clusterCentroidsAlongCols(0,6) = -0.96413;				
		_clusterCentroidsAlongCols(1,6) = 65.2992;
		_clusterCentroidsAlongCols(2,6) = 13.2004;
		//Cluster 8
		_clusterCentroidsAlongCols(0,7) = -1.05194;				
		_clusterCentroidsAlongCols(1,7) = 76.083;
		_clusterCentroidsAlongCols(2,7) = 13.2004;
		//Cluster 9
		_clusterCentroidsAlongCols(0,8) = -0.359267;				
		_clusterCentroidsAlongCols(1,8) = 86.1222;
		_clusterCentroidsAlongCols(2,8) = 13.2004;
*/
		//***  PART-3  ***//
/*
		//Cluster 1	
		_clusterCentroidsAlongCols(0,0) = 19.3939;		
		_clusterCentroidsAlongCols(1,0) = 69.2732;
		_clusterCentroidsAlongCols(2,0) = 14.4446;
		//Cluster 2
		_clusterCentroidsAlongCols(0,1) = 20.7617;		
		_clusterCentroidsAlongCols(1,1) = 79.5676;
		_clusterCentroidsAlongCols(2,1) = 14.4446;
		//Cluster 3
		_clusterCentroidsAlongCols(0,2) = 21.4479;			
		_clusterCentroidsAlongCols(1,2) = 89.2559;
		_clusterCentroidsAlongCols(2,2) = 14.4446;
		//Cluster 4
		_clusterCentroidsAlongCols(0,3) = 8.83409; 			
		_clusterCentroidsAlongCols(1,3) = 68.94;
		_clusterCentroidsAlongCols(2,3) = 14.4446;
		//Cluster 5
		_clusterCentroidsAlongCols(0,4) = 10.2785;			
		_clusterCentroidsAlongCols(1,4) = 79.0812;
		_clusterCentroidsAlongCols(2,4) = 14.4446;
		//Cluster 6
		_clusterCentroidsAlongCols(0,5) = 11.0006;			
		_clusterCentroidsAlongCols(1,5) = 88.9114;
		_clusterCentroidsAlongCols(2,5) = 14.4446;
		//Cluster 7
		_clusterCentroidsAlongCols(0,6) = -1.05823;		
		_clusterCentroidsAlongCols(1,6) = 68.0962;	
		_clusterCentroidsAlongCols(2,6) = 14.4446;
		//Cluster 8
		_clusterCentroidsAlongCols(0,7) = -0.493057;			
		_clusterCentroidsAlongCols(1,7) =  78.4419;
		_clusterCentroidsAlongCols(2,7) = 14.4446;
		//Cluster 9
		_clusterCentroidsAlongCols(0,8) = 0.277061;				
		_clusterCentroidsAlongCols(1,8) = 88.3665;
		_clusterCentroidsAlongCols(2,8) = 14.4446;
*/		
	
	
	
	
	
	
		
		//***  END - NEW DATA  ***//

		
		//***  START - OLD DATA  ***//

		//***  PART-2  ***//
/*	
		//Cluster 1	
		_clusterCentroidsAlongCols(0,0) = 17.9679;				
		_clusterCentroidsAlongCols(1,0) = 62.9017;
		_clusterCentroidsAlongCols(2,0) = 14.7025;
		//Cluster 2
		_clusterCentroidsAlongCols(0,1) = 18.3856;				
		_clusterCentroidsAlongCols(1,1) = 73.5289;
		_clusterCentroidsAlongCols(2,1) = 14.7025;
		//Cluster 4
		_clusterCentroidsAlongCols(0,3) = 7.56617; 			
		_clusterCentroidsAlongCols(1,3) = 62.32;
		_clusterCentroidsAlongCols(2,3) = 14.7025;
		//Cluster 5
		_clusterCentroidsAlongCols(0,4) = 8.4306;				
		_clusterCentroidsAlongCols(1,4) = 73.494;
		_clusterCentroidsAlongCols(2,4) = 14.7025;
		//Cluster 6
		_clusterCentroidsAlongCols(0,5) = 8.36431;				
		_clusterCentroidsAlongCols(1,5) = 83.1487;
		_clusterCentroidsAlongCols(2,5) = 14.7025;
		//Cluster 7
		_clusterCentroidsAlongCols(0,6) = -2.73778;				
		_clusterCentroidsAlongCols(1,6) = 61.767;
		_clusterCentroidsAlongCols(2,6) = 14.7025;
		//Cluster 8
		_clusterCentroidsAlongCols(0,7) = -3.20951;				
		_clusterCentroidsAlongCols(1,7) = 72.5354;
		_clusterCentroidsAlongCols(2,7) = 14.7025;
		//Cluster 9
		_clusterCentroidsAlongCols(0,8) = -3.44705;				
		_clusterCentroidsAlongCols(1,8) = 82.5641;
		_clusterCentroidsAlongCols(2,8) = 14.7025;
***/			

		//***  PART-10  ***//
/*		
		//Cluster 1	
		_clusterCentroidsAlongCols(0,0) = 19.3558;				
		_clusterCentroidsAlongCols(1,0) = 59.517;
		_clusterCentroidsAlongCols(2,0) = 15.8695;
		//Cluster 2
		_clusterCentroidsAlongCols(0,1) = 19.6032;				
		_clusterCentroidsAlongCols(1,1) = 70.6784;
		_clusterCentroidsAlongCols(2,1) = 15.8695;
		//Cluster 3
		_clusterCentroidsAlongCols(0,2) = 19.2965;				
		_clusterCentroidsAlongCols(1,2) = 80.3479;
		_clusterCentroidsAlongCols(2,2) = 15.8695;
		//Cluster 4
		_clusterCentroidsAlongCols(0,3) = 8.61261;				
		_clusterCentroidsAlongCols(1,3) = 58.7096;
		_clusterCentroidsAlongCols(2,3) = 15.8695;
		//Cluster 5
		_clusterCentroidsAlongCols(0,4) = 8.88048;				
		_clusterCentroidsAlongCols(1,4) = 68.7452;
		_clusterCentroidsAlongCols(2,4) = 15.8695;
		//Cluster 6
		_clusterCentroidsAlongCols(0,5) = 9.27839;				
		_clusterCentroidsAlongCols(1,5) = 79.8222;
		_clusterCentroidsAlongCols(2,5) = 15.8695;
		//Cluster 7
		_clusterCentroidsAlongCols(0,6) = -2.21343;				
		_clusterCentroidsAlongCols(1,6) = 58.4223;
		_clusterCentroidsAlongCols(2,6) = 15.8695;
		//Cluster 8
		_clusterCentroidsAlongCols(0,7) = -2.55877;				
		_clusterCentroidsAlongCols(1,7) = 68.6187;
		_clusterCentroidsAlongCols(2,7) = 15.8695;
		//Cluster 9
		_clusterCentroidsAlongCols(0,8) = -2.94434;				
		_clusterCentroidsAlongCols(1,8) = 79.2046;
		_clusterCentroidsAlongCols(2,8) = 15.8695;
*/	
		
		//***  PART-12  ***//
/*


		//Cluster 1	
		_clusterCentroidsAlongCols(0,0) = 18.2467;				
		_clusterCentroidsAlongCols(1,0) = 62.797;
		_clusterCentroidsAlongCols(2,0) = 15.2312;
		//Cluster 2
		_clusterCentroidsAlongCols(0,1) = 18.9968;		
		_clusterCentroidsAlongCols(1,1) = 74.0356;
		_clusterCentroidsAlongCols(2,1) = 15.2312;
		//Cluster 3
		_clusterCentroidsAlongCols(0,2) = 19.835;				
		_clusterCentroidsAlongCols(1,2) = 83.5955;
		_clusterCentroidsAlongCols(2,2) = 15.8695;
		//Cluster 4
		_clusterCentroidsAlongCols(0,3) = 7.66382; 			
		_clusterCentroidsAlongCols(1,3) = 61.993;
		_clusterCentroidsAlongCols(2,3) = 15.2312;
		//Cluster 5
		_clusterCentroidsAlongCols(0,4) = 8.84133;				
		_clusterCentroidsAlongCols(1,4) = 73.4713;
		_clusterCentroidsAlongCols(2,4) = 15.2312;
		//Cluster 6
		_clusterCentroidsAlongCols(0,5) = 8.41987;				
		_clusterCentroidsAlongCols(1,5) = 82.5947;
		_clusterCentroidsAlongCols(2,5) = 15.2312;
		//Cluster 7
		_clusterCentroidsAlongCols(0,6) = -2.53967;				
		_clusterCentroidsAlongCols(1,6) = 61.7527;
		_clusterCentroidsAlongCols(2,6) = 15.2312;
		//Cluster 8
		_clusterCentroidsAlongCols(0,7) = -2.61657;				
		_clusterCentroidsAlongCols(1,7) = 71.8138;
		_clusterCentroidsAlongCols(2,7) = 15.2312;
		//Cluster 9
		_clusterCentroidsAlongCols(0,8) = -3.21167;				
		_clusterCentroidsAlongCols(1,8) = 82.8078;
		_clusterCentroidsAlongCols(2,8) = 15.2312;
*/
  
 		 //***  PART-13  ***//
/*

		//Cluster 1	
		_clusterCentroidsAlongCols(0,0) = 19.24;				
		_clusterCentroidsAlongCols(1,0) = 66.1877;
		_clusterCentroidsAlongCols(2,0) = 14.1499;
		//Cluster 2
		_clusterCentroidsAlongCols(0,1) = 20.9655;		
		_clusterCentroidsAlongCols(1,1) = 77.9032;
		_clusterCentroidsAlongCols(2,1) = 14.1499;
		//Cluster 3
		_clusterCentroidsAlongCols(0,2) = 21.1602;				
		_clusterCentroidsAlongCols(1,2) = 87.3413;
		_clusterCentroidsAlongCols(2,2) = 14.1499;
		//Cluster 4
		_clusterCentroidsAlongCols(0,3) = 8.6968; 			
		_clusterCentroidsAlongCols(1,3) = 66.5047;
		_clusterCentroidsAlongCols(2,3) = 14.1499;
		//Cluster 5
		_clusterCentroidsAlongCols(0,4) = 10.7739;				
		_clusterCentroidsAlongCols(1,4) = 77.3176;
		_clusterCentroidsAlongCols(2,4) = 14.1499;
		//Cluster 6
		_clusterCentroidsAlongCols(0,5) = 11.1624;				
		_clusterCentroidsAlongCols(1,5) = 86.55;
		_clusterCentroidsAlongCols(2,5) = 14.1499;
		//Cluster 7
		_clusterCentroidsAlongCols(0,6) = -1.72327;				
		_clusterCentroidsAlongCols(1,6) = 65.4738;
		_clusterCentroidsAlongCols(2,6) = 14.1499;
		//Cluster 8
		_clusterCentroidsAlongCols(0,7) = -1.06003;				
		_clusterCentroidsAlongCols(1,7) = 76.0267;
		_clusterCentroidsAlongCols(2,7) = 14.1499;
		//Cluster 9
		_clusterCentroidsAlongCols(0,8) = -1.12584;				
		_clusterCentroidsAlongCols(1,8) = 85.935;
		_clusterCentroidsAlongCols(2,8) = 14.1499;
*/

 		//***  PART-16  ***//
/*

		//Cluster 1	
		_clusterCentroidsAlongCols(0,0) = 20.2365;				
		_clusterCentroidsAlongCols(1,0) = 60.5758;
		_clusterCentroidsAlongCols(2,0) = 15.8011;
		//Cluster 2
		_clusterCentroidsAlongCols(0,1) = 20.2711;		
		_clusterCentroidsAlongCols(1,1) = 71.2489;
		_clusterCentroidsAlongCols(2,1) = 15.8011;
		//Cluster 3
		_clusterCentroidsAlongCols(0,2) = 17.7045;				
		_clusterCentroidsAlongCols(1,2) = 80.8678;
		_clusterCentroidsAlongCols(2,2) = 15.8011;
		//Cluster 4
		_clusterCentroidsAlongCols(0,3) = 10.2204; 			
		_clusterCentroidsAlongCols(1,3) = 60.1306;
		_clusterCentroidsAlongCols(2,3) = 15.8011;
		//Cluster 5
		_clusterCentroidsAlongCols(0,4) = 9.51099;				
		_clusterCentroidsAlongCols(1,4) = 70.0289;
		_clusterCentroidsAlongCols(2,4) = 15.8011;
		//Cluster 6
		_clusterCentroidsAlongCols(0,5) = 10.4293;				
		_clusterCentroidsAlongCols(1,5) =  80.9666;
		_clusterCentroidsAlongCols(2,5) = 15.8011;
		//Cluster 7
		_clusterCentroidsAlongCols(0,6) = -1.34457;				
		_clusterCentroidsAlongCols(1,6) = 60.1122;
		_clusterCentroidsAlongCols(2,6) = 15.8011;
		//Cluster 8
		_clusterCentroidsAlongCols(0,7) = -1.7764;				
		_clusterCentroidsAlongCols(1,7) = 70.2593;
		_clusterCentroidsAlongCols(2,7) = 15.8011;
		//Cluster 9
		_clusterCentroidsAlongCols(0,8) = -1.6553;				
		_clusterCentroidsAlongCols(1,8) = 80.7737;
		_clusterCentroidsAlongCols(2,8) = 15.8011;
*/		

		//***  PART-17  ***//
/*			
		//Cluster 1	
		_clusterCentroidsAlongCols(0,0) = 21.3928;				
		_clusterCentroidsAlongCols(1,0) = 64.4085;
		_clusterCentroidsAlongCols(2,0) = 16.6473;
		//Cluster 2
		_clusterCentroidsAlongCols(0,1) = 0.0;		
		_clusterCentroidsAlongCols(1,1) = 0.0;
		_clusterCentroidsAlongCols(2,1) = 0.0;
		//Cluster 3
		_clusterCentroidsAlongCols(0,2) = 12.0878;				
		_clusterCentroidsAlongCols(1,2) = 85.0675;
		_clusterCentroidsAlongCols(2,2) = 16.6473;
		//Cluster 4
		_clusterCentroidsAlongCols(0,3) = 11.1419; 			
		_clusterCentroidsAlongCols(1,3) = 64.0745;
		_clusterCentroidsAlongCols(2,3) = 16.6473;
		//Cluster 5
		_clusterCentroidsAlongCols(0,4) = 0.0;				
		_clusterCentroidsAlongCols(1,4) = 0.0;
		_clusterCentroidsAlongCols(2,4) = 0.0;
		//Cluster 6
		_clusterCentroidsAlongCols(0,5) = 0.0;				
		_clusterCentroidsAlongCols(1,5) =  0.0;
		_clusterCentroidsAlongCols(2,5) = 0.0;
		//Cluster 7
		_clusterCentroidsAlongCols(0,6) = -0.110431;				
		_clusterCentroidsAlongCols(1,6) = 63.8404;
		_clusterCentroidsAlongCols(2,6) = 16.6473;
		//Cluster 8
		_clusterCentroidsAlongCols(0,7) = 0.0;				
		_clusterCentroidsAlongCols(1,7) = 0.0;
		_clusterCentroidsAlongCols(2,7) = 0.0;
		//Cluster 9
		_clusterCentroidsAlongCols(0,8) = 0.0977335;				
		_clusterCentroidsAlongCols(1,8) = 84.1367;
		_clusterCentroidsAlongCols(2,8) = 16.6473;
*/	

		//***  END - OLD DATA  ***//
	}
	
	int nClusters() {
		return _nClusters;
	}
	
	MatrixXd getClusterCentroidsAlongCols() {
		return _clusterCentroidsAlongCols;
	}
	
	/* Find Which Cluster The Rach (Target) Belongs to */
	int whichCluster() {
	
		double minDistance = _inTargetPt.norm();
		size_t minDistanceId = -1;
		for (size_t i = 0; i < _nClusters; i++) {
			double currDistance = (_inTargetPt
				- _clusterCentroidsAlongCols.col(i)).norm();
			
			minDistanceId = minDistance < currDistance 
				? minDistanceId : i+1;
			minDistance = minDistance < currDistance 
				? minDistance : currDistance;
		}
		
		_clusterId = minDistanceId; 	
		
		return 	_clusterId;
	}
	
	// Numerically order clusters
	void orderinPtsInClusters () {
	
	
	}		
	
	
	/**	Corner clusters correspond to 
  		1 -> bottom - left
  		3 -> bottom - right
  		7 -> top - left
  		9 -> top - right
       **/
	MatrixXd getCornerClustersCentroid() {
	
	//since part 2 is missing cluster 3, add y-component of cluster 9 twice
	// for calculating the centroid 
	
      		vector <int> cornerClusterIds = {0,2,6,8};
      		
      		//vector <int> cornerClusterIds = {0,2,6,8};
      		
		// concatenate corner clusters into a matrix
		MatrixXd cornerClusters = MatrixXd::Zero(3,4);
		int col = 0;
		for (auto clusterId = begin(cornerClusterIds)
			; clusterId != end (cornerClusterIds)
			; ++clusterId) {
		
		
			size_t clusterN = *clusterId;
			cornerClusters.col(col) = 
				_clusterCentroidsAlongCols.col(clusterN);
			col++;	

		}
		MatrixXd cornerClustersCentroid = cornerClusters.rowwise().mean();
		return cornerClustersCentroid;

	}				
	
	int _nClusters;
	MatrixXd _clusterCentroidsAlongCols;
	int _clusterId;
	
	MatrixXd _inTargetPt;

};


#endif 
