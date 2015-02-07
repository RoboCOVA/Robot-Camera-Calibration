#include<stdio.h>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "target_robot.h"
#include "target_camera.h"
#define square_size 29
void trans_target_robot() {
	int n = 3; // the dimension of point cloud
	int N = 4; // The number of cloud point
// the model coordinate point cloud
	float model[] = { square_size, square_size * 6,
			0.01, // sorted in 	X1,X2,X3,X4 column wise
			square_size, square_size, 0.0, square_size * 6, square_size, 0.02,
			square_size * 6, square_size * 6, 0.05 };

	// the robot coordinate point cloud

	float robot_tool[] = { 576.4, -73.3, 408, 436.4, -81.3, 407.1, 444.7,
			-221.3, 407.1, 584.9, -213.3, 407.4

	};

	float laser_point[] = { 736.9, -71.0, 261.1 +300,  //gives point in base frame
	                         596.9, -79.0, 260.2 +300,
	                                605.2,  -219.0, 260.1 +300,
	                                 745.3, -210.9,  260.5 + 300};    // add +300mm in the Z dimension
// laser source

	float laser_source[] = { 736.9, -71.0, 261.1 ,
		                         596.9, -79.0, 260.2 ,
		                                605.2,  -219.0, 260.1 ,
		                                 745.3, -210.9,  260.5 };

// declaration of the matrix header for input data
	CvMat* modelCoords;
	CvMat* robotCoords;

	modelCoords = cvCreateMat(N, n, CV_32FC1); // 4x3
	robotCoords = cvCreateMat(N, n, CV_32FC1);

//fill in the data-> the cloud of point of target and the robot tool
	int i, j;
	printf("the model coordinates\n");
	for (i = 0; i < N; i++) {
		modelCoords->data.fl[i * 3] = robot_tool[i * 3];  /// model
		modelCoords->data.fl[i * 3 + 1] = robot_tool[i * 3 + 1];
		modelCoords->data.fl[i * 3 + 2] = robot_tool[i * 3 + 2];
		printf("%f\t\t%f\t\t%f\n", robot_tool[i * 3], robot_tool[i * 3 + 1], robot_tool[i * 3 + 2]);
	}
	printf("robot tool coordinates\n");
	for (j = 0; j < N; j++) {
		robotCoords->data.fl[j * 3] = laser_source[j * 3];
		robotCoords->data.fl[j * 3 + 1] = laser_source[j * 3 + 1];
		robotCoords->data.fl[j * 3 + 2] = laser_source[j * 3 + 2];
		printf("%f\t\t%f\t\t%f\n", model[j * 3], model[j * 3 + 1],model[j * 3 + 2]);

	}
//===========================================================================
	/*
	 //perspective transformation
	 CvMat* transformation = cvCreateMat(4,4,CV_32FC1);
	 cvPerspectiveTransform(modelCoords,robotCoords, transformation);
	 printf("the transformation matrix \n");
	 int tda;
	 for (tda=0; tda<4;tda++)
	 {
	 printf("%f\t\t\t%f\t\t\t%f\t\t\t%f\n", transformation->data.fl[tda*4],transformation->data.fl[tda*4 +1],
	 transformation->data.fl[tda*4+2], transformation->data.fl[tda*4 +3]);
	 }
	 //================================================
	 */

// declaration of mean of the model coordinates-X,Y,Z
	float meanModelX, meanModelY, meanModelZ;
	meanModelX = (1.0 / 4)
			* (CV_MAT_ELEM(*modelCoords, float, 0,0)
					+ CV_MAT_ELEM(*modelCoords, float, 1,0)
					+ CV_MAT_ELEM(*modelCoords, float, 2,0)
					+ CV_MAT_ELEM(*modelCoords, float, 3,0));
	meanModelY = (1.0 / 4)
			* (CV_MAT_ELEM(*modelCoords, float, 0,1)
					+ CV_MAT_ELEM(*modelCoords, float, 1,1)
					+ CV_MAT_ELEM(*modelCoords, float, 2,1)
					+ CV_MAT_ELEM(*modelCoords, float, 3,1));
	meanModelZ = (1.0 / 4)
			* (CV_MAT_ELEM(*modelCoords, float, 0,2)
					+ CV_MAT_ELEM(*modelCoords, float, 1,2)
					+ CV_MAT_ELEM(*modelCoords, float, 2,2)
					+ CV_MAT_ELEM(*modelCoords, float, 3,2));

	printf("the mean of model point cloud \n%f\n%f\n%f\n", meanModelX,
			meanModelY, meanModelZ);

// Declaration of mean of the robot coordinate frame x, y,z
	float meanRobotX, meanRobotY, meanRobotZ;
	meanRobotX = (1.0 / 4)
			* (CV_MAT_ELEM(*robotCoords,float,0,0)
					+ CV_MAT_ELEM(*robotCoords,float,1,0)
					+ CV_MAT_ELEM(*robotCoords,float,2,0)
					+ CV_MAT_ELEM(*robotCoords,float,3,0));
	meanRobotY = (1.0 / 4)
			* (CV_MAT_ELEM(*robotCoords,float,0,1)
					+ CV_MAT_ELEM(*robotCoords,float,1,1)
					+ CV_MAT_ELEM(*robotCoords,float,2,1)
					+ CV_MAT_ELEM(*robotCoords,float,3,1));
	meanRobotZ = (1.0 / 4)
			* (CV_MAT_ELEM(*robotCoords,float,0,2)
					+ CV_MAT_ELEM(*robotCoords,float,1,2)
					+ CV_MAT_ELEM(*robotCoords,float,2,2)
					+ CV_MAT_ELEM(*robotCoords,float,3,2));
	printf("the mean of point cloud of robot\n%f\n%f\n%f\n", meanRobotX,
			meanRobotY, meanRobotZ);
// CONVERSION OF SCALAR VALUES OF MEAN TO THE ARRAY
	double meanArrayModel[] = { meanModelX, meanModelY, meanModelZ };
	double meanArrayRobot[] = { meanRobotX, meanRobotY, meanRobotZ };

	int te;
	for (te = 0; te < 3; te++) {
		printf("the mean of point cloud of target in x y and z %f\n",
				meanArrayModel[te]);

	}
	int ta;
	for (ta = 0; ta < 3; ta++) {
		printf("the mean of point cloud of robot tool in x y and z %f\n",
				meanArrayRobot[ta]);

	}

// declare and initialize the array

	CvMat* meanMatrixModel;
	CvMat* meanMatrixRobot;

	meanMatrixModel = cvCreateMat(1, 3, CV_32FC1);
	meanMatrixRobot = cvCreateMat(1, 3, CV_32FC1);

/// FILL THE THE THE POINTS
	int t;
	for (t = 0; t < 3; t++) {
		meanMatrixModel->data.fl[t] = meanArrayModel[t];
		meanMatrixRobot->data.fl[t] = meanArrayRobot[t];

	}
	assert(!cvIsNaN(meanMatrixModel->data.fl[0]));
// declaration of repeated mean
	CvMat* repeatedMeanModel;
	CvMat* repeatedMeanRobot;

// initialize the matrix of mean
	repeatedMeanModel = cvCreateMat(4, 3, CV_32FC1);
	repeatedMeanRobot = cvCreateMat(4, 3, CV_32FC1);
// FIILLING THE MATRIX
// to create equal dimension of source and array of mean

	cvRepeat(meanMatrixModel, repeatedMeanModel);
	cvRepeat(meanMatrixRobot, repeatedMeanRobot);
	assert(!cvIsNaN(repeatedMeanModel->data.fl[0]));
	printf("the test is \n%f", repeatedMeanModel->data.fl[3]);
//check the repeated mean value
	int this;
	printf("the repeated mean value for model\n");
	for (this = 0; this < 3; this++) {
		printf("%f\t\t\t%f\t\t\t%f\n", repeatedMeanModel->data.fl[this * 3],
				repeatedMeanModel->data.fl[this * 3 + 1],
				repeatedMeanModel->data.fl[this * 3 + 2]);
	}

	int thas;
	printf("the repeated mean value for model\n");
	for (thas = 0; thas < 3; thas++) {
		printf("%f\t\t\t%f\t\t\t%f\n", repeatedMeanRobot->data.fl[thas * 3],
				repeatedMeanRobot->data.fl[thas * 3 + 1],
				repeatedMeanRobot->data.fl[thas * 3 + 2]);
	}

// refine the values of the point cloud with respect to the centroid of point cloud
// this is for later refinement of the values of the point cloud so that it reduces the distance from the center of the frame

// declare the matrix for storage of refined matrix

	CvMat* refineModel;
	CvMat* refineRobot;
// initialize the values
	refineModel = cvCreateMat(N, n, CV_32FC1);
	refineRobot = cvCreateMat(N, n, CV_32FC1);

//refinement for the model point cloud from the center of the points
//refinedModel=  model[] - repeatedMeanModel
// refinement for the robot tool point cloud from the center of the frame
//refinedRobot = robot_tool - repeatedMeanRobot
	const CvMat* mask = NULL;
	cvSub(modelCoords, repeatedMeanModel, refineModel, mask);
	cvSub(robotCoords, repeatedMeanRobot, refineRobot, mask);

// print to see the difference between the original data and the refined one
	int ts, tf;
	printf("the refined point cloud of the target frame \n");
	for (ts = 0; ts < 4; ts++) {
		printf("%f\t\t\t%f\t\t\t%f\n", refineModel->data.fl[ts * 3],
				refineModel->data.fl[ts * 3 + 1],
				refineModel->data.fl[ts * 3 + 2]);
	}
	printf("the refined point cloud of the robot frame\n");
	for (tf = 0; tf < 4; tf++) {
		printf("%f\t\t\t%f\t\t\t%f\n", refineRobot->data.fl[tf * 3],
				refineRobot->data.fl[tf * 3 + 1],
				refineRobot->data.fl[tf * 3 + 2]);
	}
// Transpose the  refined target frame
	CvMat* refineModelTranspose;
	refineModelTranspose = cvCreateMat(3, 4, CV_32FC1);
	cvTranspose(refineModel, refineModelTranspose);

// check transpose
	int td;
	printf("the transpose of model coordinate frame\n");
	for (td = 0; td < 4; td++) {
		printf("%f\t\t\t%f\t\t\t%f\t\t\t%f\n",
				refineModelTranspose->data.fl[td * 4],
				refineModelTranspose->data.fl[td * 4 + 1],
				refineModelTranspose->data.fl[td * 4 + 2],
				refineModelTranspose->data.fl[td * 4 + 3]);
	}

// Transpose the  refined robotframe
	CvMat* refineRobotTranspose;
	refineRobotTranspose = cvCreateMat(3, 4, CV_32FC1);
	cvTranspose(refineRobot, refineRobotTranspose);
/// determine the covariance matrix that is the product of the distance of each point from center of the cloud in each crossponding frame
// pass this value to singular value decompostion to routines
//covariance matrix = (model- mean model)*(robot_tool - mean robot)

// declare the covariance matrix and initialize the matrix- it should the same as the dimension of the coordinate system

	CvMat* covariance_matrix;
	CvMat* covariance_matrix1;

	covariance_matrix1 = cvCreateMat(4, 4, CV_32FC1);
	covariance_matrix = cvCreateMat(3, 3, CV_32FC1);
	CvMat* optional_mat = NULL;
	float alpha = 1.0;
	float beta = 0.0;
	int tABC = 0;
	// THE COVARAINCE MATRIX = refineModelTranspose * refineRobot
// THE GENERALIZED MATRIX MULTIPLICATION



	cvGEMM(refineModelTranspose, refineRobot, 1.0, optional_mat, 0.0,
			covariance_matrix, tABC);
	cvGEMM(refineModel, refineRobotTranspose, alpha, optional_mat, beta,
			covariance_matrix1, tABC);


	int ty;
	printf("\n covariance matrix\n");
	for (ty = 0; ty < 3; ty++) {
		printf("%f\t\t\t%f\t\t\t%f\n", covariance_matrix->data.fl[ty * 3],
				covariance_matrix->data.fl[ty * 3 + 1],
				covariance_matrix->data.fl[ty * 3 + 2]);
	}
// passing the covariance matrix to the singular value decomposition [U,W,V]= cvSVD(covaraince_matrix, U,W,V, FALGS);
// covaraince_matrix = U.W.V'
	CvMat* U;
	CvMat* V;
	CvMat* W;
//int flags = CV_SVD_U_T;
	U = cvCreateMat(3, 3, CV_32FC1);
	V = cvCreateMat(3, 3, CV_32FC1);
	W = cvCreateMat(3, 3, CV_32FC1);

// singular value decomposition
//=============================================================================================

	cvSVD(covariance_matrix, W, U, V, CV_SVD_U_T); // covariance_matrix = UWV'

//========================================================================================
	// U and V are unitary matrix with dimension of mxm and mxn matrix respectively

	printf("the V matrix \n");
	int tra;
	for (tra = 0; tra < 3; tra++) {
		printf("%f\t\t\t%f\t\t\t%f\n", V->data.fl[tra * 3],
				V->data.fl[tra * 3 + 1], V->data.fl[tra * 3 + 2]);
	}
	printf("the U matrix \n");
	int tm;
	for (tm = 0; tm < 3; tm++) {
		printf("%f\t\t\t%f\t\t\t%f\n", U->data.fl[tm * 3],
				U->data.fl[tm * 3 + 1], U->data.fl[tm * 3 + 2]);
	}
	printf("the W matrix \n");
	int tma;
	for (tma = 0; tma < 3; tma++) {
		printf("%f\t\t\t%f\t\t\t%f\n", W->data.fl[tma * 3],
				W->data.fl[tma * 3 + 1], W->data.fl[tma * 3 + 2]);
	}

	/*if (V->data.fl[2] < 0 && V->data.fl[5] < 0 && V->data.fl[8]< 0 )
	 {
	 V->data.fl[2] = - V->data.fl[2];
	 V->data.fl[5] = - V->data.fl[5];
	 V->data.fl[5] = - V->data.fl[5];
	 }*/

	CvMat* rotation_matrix;
	rotation_matrix = cvCreateMat(3, 3, CV_32FC1);
	// R = VU'rotation_matrix->data.fl[tr*3]
	CvMat* Utranspose;
	Utranspose = cvCreateMat(3, 3, CV_32FC1);
	cvTranspose(U, Utranspose);
	cvGEMM(V, U, 1.0, NULL, beta, rotation_matrix, tABC);
	assert(!cvIsNaN(rotation_matrix->data.fl[3]));
//cvTranspose(rotation_matrix, rotation_matrix);

// modify rotation matrix if its determinant is less than zero -multiply the
// all third element by t-1
//=====================================================================================
	/*vMat* newRotation;
	 CvMat* rotationTR; //rotation matrix transpose
	 rotationTR = cvCreateMat(3,3, CV_32FC1);
	 newRotation = cvCreateMat(3,3, CV_32FC1);
	 cvTranspose(rotation_matrix, rotationTR);
	 int kk;
	 for(kk=0;kk<5 ;kk++){

	 newRotation->data.fl[kk]= rotationTR->data.fl[kk];
	 }
	 newRotation->data.fl[5]= -rotationTR->data.fl[5];
	 newRotation->data.fl[6]= -rotationTR->data.fl[6];
	 newRotation->data.fl[7]= rotationTR->data.fl[7];
	 newRotation->data.fl[8]= rotationTR->data.fl[8];
	 cvTranspose(newRotation, rotation_matrix);*/

//=================================================================
	float detRotation;
	detRotation = cvDet(rotation_matrix);
	if (detRotation < 0) {
		rotation_matrix->data.fl[2] = -rotation_matrix->data.fl[2];
		rotation_matrix->data.fl[5] = -rotation_matrix->data.fl[5];
		rotation_matrix->data.fl[8] = -rotation_matrix->data.fl[8];
	}
	float a = cvDet(rotation_matrix);

	printf("the determinant of rotation matrix\n %f\n", detRotation);
	printf("the determinant of new rotation matrix %f\n", a);
	printf("the rotation matrix target- robot\n");
	int tr;
	for (tr = 0; tr < 3; tr++) {
		printf("%f\t\t\t%f\t\t\t%f\n", rotation_matrix->data.fl[tr * 3],
				rotation_matrix->data.fl[tr * 3 + 1],
				rotation_matrix->data.fl[tr * 3 + 2]);
	}

// find the translation vector between the two frames
// t= -RTtrgetcenter + robot center;
//tranpose mean matrix both for the robot and the target
	CvMat* meanModelTR = cvCreateMat(3, 1, CV_32FC1);
	CvMat* meanRobotTR = cvCreateMat(3, 1, CV_32FC1);

	cvTranspose(meanMatrixModel, meanModelTR);
	cvTranspose(meanMatrixRobot, meanRobotTR);
	CvMat* translation_vector = cvCreateMat(3, 1, CV_32FC1);
	CvMat* roto_translation = cvCreateMat(3, 1, CV_32FC1);
	cvGEMM(rotation_matrix, meanModelTR, alpha, optional_mat, beta,
			roto_translation, tABC);

	assert(!cvIsNaN(roto_translation->data.fl[0]));
	//find translation
	// t= -roto_translation + robot center;
//CvMat* mask= NULL;
	cvSub(meanRobotTR, roto_translation, translation_vector, 0);

	printf(
			"the translation vector from target base frame to robot frame\n%f\n%f\n%f\n",
			translation_vector->data.fl[0], translation_vector->data.fl[1],
			translation_vector->data.fl[2]);

//====================================================================

	printf(
			"\n THE TRANSFORMATION MATRIX FROM TARGET COORDINATE FRAME TO ROBOT TOOL FRAME\n");

	transformation_matrix1 = cvCreateMat(4, 4, CV_32FC1);
	transformation_matrix1->data.fl[12] = 0;
	transformation_matrix1->data.fl[13] = 0;
	transformation_matrix1->data.fl[14] = 0;
	transformation_matrix1->data.fl[15] = 1;
	transformation_matrix1->data.fl[3] = translation_vector->data.fl[0];
	transformation_matrix1->data.fl[7] = translation_vector->data.fl[1];
	transformation_matrix1->data.fl[11] = translation_vector->data.fl[2];
	transformation_matrix1->data.fl[0] = rotation_matrix->data.fl[0];
	transformation_matrix1->data.fl[1] = rotation_matrix->data.fl[1];
	transformation_matrix1->data.fl[2] = rotation_matrix->data.fl[2];
	transformation_matrix1->data.fl[4] = rotation_matrix->data.fl[3];
	transformation_matrix1->data.fl[5] = rotation_matrix->data.fl[4];
	transformation_matrix1->data.fl[6] = rotation_matrix->data.fl[5];
	transformation_matrix1->data.fl[8] = rotation_matrix->data.fl[6];
	transformation_matrix1->data.fl[9] = rotation_matrix->data.fl[7];
	transformation_matrix1->data.fl[10] = rotation_matrix->data.fl[8];

	// pass the transformation matrix
	// pass_trans_target_robot(&transformation_matrix1);
CvMat*  invt = cvCreateMat(4,4, CV_32FC1);

  cvInvert(transformation_matrix1, invt, CV_LU);


	int xy;
	for (xy = 0; xy < 4; xy++) {
		printf("\n%f\t\t\t%f\t\t\t%f\t\t\t%f\n",
				invt->data.fl[xy * 4],
		     	invt->data.fl[xy * 4 + 1],
				invt->data.fl[xy * 4 + 2],
			    invt->data.fl[xy * 4 + 3]);

	}
//=========================================================================
	///test

	CvMat* test = cvCreateMat(3, 1, CV_32FC1);
	CvMat* test2 = cvCreateMat(3, 1, CV_32FC1);
	CvMat* test3 = cvCreateMat(3, 1, CV_32FC1);

	test->data.fl[0] = modelCoords->data.fl[0];
	test->data.fl[1] = modelCoords->data.fl[1];
	test->data.fl[2] = modelCoords->data.fl[2];
	cvGEMM(rotation_matrix, test, alpha, 0, beta, test2, 0);
	assert(!cvIsNaN(test2->data.fl[0]));
	printf("\n the test result for point cloud of base rotation\n%f\n%f\n%f\n",
			test2->data.fl[0], test2->data.fl[1], test2->data.fl[2]);

	cvAdd(test2, translation_vector, test3, 0);
	printf("\n the test result\n%f\n%f\n%f\n", test3->data.fl[0],
			test3->data.fl[1], test3->data.fl[2]);
// clean the memory
	cvReleaseMat(&modelCoords);
	cvReleaseMat(&robotCoords);
	cvReleaseMat(&repeatedMeanModel);
	cvReleaseMat(&repeatedMeanRobot);
	cvReleaseMat(&refineModelTranspose);
	cvReleaseMat(&U);
	cvReleaseMat(&V);
	cvReleaseMat(&W);
	cvReleaseMat(&covariance_matrix);
	cvReleaseMat(&rotation_matrix);
	cvReleaseMat(&translation_vector);
     cvReleaseMat(&invt);
}
//////////////////////////////////////////////////////////////////
// pass the transformation matrix


/*
 CvMat* reMeanA = cvCreateMat(n,N, CV_64FC1);
 CvMat* reMeanB = cvCreateMat(n,N, CV_64FC1);
 CvMat* refinedA= cvCreateMat(n, N,CV_64FC1);
 CvMat* refinedB = cvCreateMat(n,N,CV_64FC1);
 CvMat*
 //double C[];
 CvMat Ma, Mb;
 int step = CV_AUTOSTEP;
 cvInitMatHeader(&Ma, 3,4,CV_64FC1,A, step);1
 cvInitMatHeader(&Mb,3,4,CV_64FC1,B,step);


 //cvInitMatHeader(&Mc, 3,3, CV_64FC1,C);
 int n= 3;// the number of column
 int N = 4;



 //cvMat* avgA= cvCreateMat(n,1,CV_64FC1);
 //cvMat* avgB= cvCreateMat(n,1,CV_64FC1);
 //cvMat* covariance_matrix = cvCreateMat(,n,CV_64FC1);
 //average


 //CvScalar meanA = cvAvg(&Ma,0);
 double meanA1 = (1/N)*(A[0]+ A[1]+ A[2] +A[3]);
 double meanA2 = (1/N)*(A[4]+ A[5]+ A[6] +A[7]);
 double meanA3 = (1/N)*(A[8]+ A[9]+ A[10] +A[11]);


 double meanB1 = (1/N)*(B[0]+ B[1]+ B[2] +B[3]);
 double meanB2 = (1/N)*(B[4]+ B[5]+ B[6] +B[7]);
 double meanB3 = (1/N)*(B[8]+ B[9]+ B[10] +B[11]);
 //CvScalar meanB  = cvAvg(&Mb,0);
 //CvMat averageA, averageB;
 double meanAllA[] = {meanA1, meanA2, meanA3};
 double meanAllB[] = {meanB1, meanB2, meanB3};


 CvMat averageA, averageB;
 cvInitMatHeader(&averageA,3,1,CV_64FC1, meanAllA,step);
 cvInitMatHeader(&averageB ,3,1,CV_64FC1, meanAllB,step);

 CvMat* reMeanA = cvCreateMat(n,N, CV_64FC1);// THE MEAN OF POINT CLOUD A    4*3
 CvMat* reMeanB = cvCreateMat(n,N, CV_64FC1);// THE MEAN OF POINT CLOD OF B 4*3
 CvMat* refinedA= cvCreateMat(n, N,CV_64FC1);
 CvMat* refinedB = cvCreateMat(n,N,CV_64FC1);



 cvRepeat(&averageA, reMeanA);
 cvRepeat(&averageB, reMeanB);

 cvSub(&Ma,reMeanA, refinedA, 0);   //refineA = Ma - remeanA
 cvSub(&Mb,reMeanB, refinedB, 0);   // refinedB= Ma - remeanB



 CvMat* transB =cvCreateMat(N,n,CV_64FC1);
 cvTranspose(refinedB,transB);


 CvMat* result =cvCreateMat(n,n,CV_64FC1);
 cvMul( refinedA,transB,result,1);
 //int rows = result->rows;
 // int cols = result->cols;

 //  int i, j;
 //for (i =0 ;i< rows; i++){
 //	 for ( j=0 ;j<cols ; j++){
 //printf("the covariance is %d\n", &result[i][j]);
 //    }
 //  printf("\n");
 //  }


 CvMat* U =cvCreateMat(n,n,CV_64FC1);
 CvMat* V= cvCreateMat(n,n,CV_64FC1);
 CvMat* W   =cvCreateMat(n,n,CV_64FC1);

 //cvSVD(covariance_matrix, CvArr* w, CvArr* U =NULL,CvArr* V= NULLL, int flags =0);

 cvSVD(result,U,W,V ,CV_SVD_U_T|CV_SVD_V_T);
 CvMat* rotation_matrix = cvCreateMat(n,n,CV_64FC1);
 CvMat* transU = cvCreateMat(n,n,CV_64FC1);
 cvTranspose(U, transU);
 // rotation_matrix=V*U';
 cvMul(V,transU,rotation_matrix,1);

 //printf("%d",rotation_matrix);
 */

