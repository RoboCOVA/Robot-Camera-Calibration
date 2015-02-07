#include<stdio.h>
#include<string.h>
#include<math.h>
#include<GL/glut.h>
#include <GL/gl.h>
#include"opencv2/highgui/highgui.hpp"
#include"opencv2/core/core.hpp"
#include"opencv2/calib3d/calib3d.hpp"
#include  "finalTransformation2.h"
#include "target_robot.h"
#include "target_camera.h"
#include "given_image_coords.h"
//#include "glModel.h"
/*float model_view[16];
 float projection_mat[16];*/
//the main loop of the program
//void renderCube(float x, float y, float z);
void euler_angle(CvMat* mat);
void quaternion_angle(CvMat* mmat);

//===============================================================================================
//THE TRANSFORMATION MATRIX FROM OBJECT COORDINATE FRAME TO ROBOT BASE  FRAME

/*0.058476			0.998278			-0.004632			563.814575

-0.998286			0.058464			-0.002570			-49.583046

-0.002295			0.004774			0.999986			560.203308

0.000000			0.000000			0.000000			1.000000*/

//THE TRANSFORMATION MATRIX FROM LASER SOURCE COORDINATE FRAME TO OBJECT COORDINATE  FRAME

/*0.058476			-0.998286			0.002295			-83.065887

0.998278			0.058464			-0.004774			-558.700073

0.004632			0.002570			0.999986			-263.187103

0.000000			0.000000			0.000000			1.000000*/




//THE TRANSFORMATION MATRIX FROM ROBOT TOOL COORDINATE FRAME TO LASER SOURCE FRAME

/*
1.000000			-0.000021			-0.000336			160.608551

0.000021			1.000000			-0.000377			2.467819

0.000336			0.000377			1.000000			-147.040680

0.000000			0.000000			0.000000			1.000000
*/


float sizex = 200.0;
float sizey = 50.0;
float sizez = 100.0;

int main(int argc, char** argv) {
	trans_target_robot();
	trans_target_camera();
	test_point_cloud();
	//  model_view_matrix();
	//projection_matrix();
//}
	CvMat* optional_mat = NULL;
	float alpha = 1.0;
	float beta = 0.0;
	int tABC = 0;
	CvMat* pixel_coords;

	float OBJRO[]= {0.058476		,	0.998278	,		-0.004632	,		563.814575,
	-0.998286	,		0.058464		,	-0.002570	,		-49.583046,
	-0.002295	,		0.004774	,		0.999986		,	560.203308,
	0.000000,			0.000000	,		0.000000	,		1.000000};

	//THE TRANSFORMATION MATRIX FROM LASER SOURCE COORDINATE FRAME TO OBJECT COORDINATE  FRAME

	float LSOBJ[] ={0.058476	,		-0.998286	,		0.002295	,		-83.065887,
	0.998278	,		0.058464	,		-0.004774			-558.700073,
#include "given_image_coords.h"
	0.004632	,		0.002570	,		0.999986	,		-263.187103,
	0.000000		,	0.000000		,	0.000000		,	1.000000};


	//THE TRANSFORMATION MATRIX FROM ROBOT TOOL COORDINATE FRAME TO LASER SOURCE FRAME


	float TOOLAS[] ={1.000000	,		-0.000021	,		-0.000336		,	160.608551,
	0.000021	,		1.000000	,		-0.000377		,	2.467819,
	0.000336	,		0.000377	,		1.000000	,		-147.040680,
	0.000000		,	0.000000	,		0.000000	,		1.000000};

	CvMat* OBJRO_mat = cvCreateMat(4,4, CV_32FC1);
	CvMat* LSOBJ_mat= cvCreateMat(4,4, CV_32FC1);
	CvMat* TOOLAS_mat= cvCreateMat(4,4, CV_32FC1);
	CvMat* semi_mat = cvCreateMat(4,4, CV_32FC1);
	CvMat* TOOLROBOT = cvCreateMat(4,4, CV_32FC1);


	int dub;

	for (dub = 0;dub<16;dub++)
	{
		OBJRO_mat->data.fl[dub] = OBJRO[dub];
		LSOBJ_mat->data.fl[dub] = OBJRO[dub];
		TOOLAS_mat->data.fl[dub] = OBJRO[dub];

	}

	cvGEMM(LSOBJ_mat, TOOLAS_mat, alpha, optional_mat,beta, semi_mat, tABC);
	cvGEMM(OBJRO_mat,semi_mat, alpha, optional_mat,beta,TOOLROBOT, tABC);

	int boo;
	printf("\n THISIISIIIIIIIIIIIII\n");
	for (boo = 0;boo<4;boo++)
	{
		printf("\n%f\t%f\t%f\t%f\n", TOOLROBOT->data.fl[boo*4],TOOLROBOT->data.fl[boo*4 +1 ],TOOLROBOT->data.fl[boo*4 + 2],
				TOOLROBOT->data.fl[boo*4  +3]);

	}

   cvReleaseMat(&OBJRO_mat);
   cvReleaseMat(&LSOBJ_mat);
   cvReleaseMat(&TOOLAS_mat);
   cvReleaseMat(&LSOBJ_mat);
   cvReleaseMat(&TOOLROBOT);

	//===================================================================================
	float object_coord[] = { 0.0, 0.0, 0.0, 1.0,

	0.0, sizey, 0.0, 1.0, 0.0, sizey, sizez, 1.0, sizex, 0.0, 0.0, 1.0, sizex,
			sizey, 0.0, 1.0,

			sizex, sizey, sizez, 1.0, 0.0, 0.0, sizez, 1.0, sizex, 0.0, sizez,
			1.0 // point 8
			};
	CvMat* world_obj;
	world_obj = cvCreateMat(4, 8, CV_32FC1);
	int aah;
	for (aah = 0; aah < 8; aah++) {

		world_obj->data.fl[aah * 4] = object_coord[aah * 4];
		world_obj->data.fl[aah * 4 + 1] = object_coord[aah * 4 + 1];
		world_obj->data.fl[aah * 4 + 2] = object_coord[aah * 4 + 2];
		world_obj->data.fl[aah * 4 + 3] = object_coord[aah * 4 + 3];

	}

	CvMat* result_cam;
//CvMat* object_coords_trans;
	result_cam = cvCreateMat(4, 8, CV_32FC1);
///object_coords_trans = cvCreateMat(6,4,CV_32FC1);
//cvTranspose(world_obj, object_coords_trans);
	cvGEMM(transformation_matrix, world_obj, alpha, optional_mat, beta,
			result_cam, tABC);

	CvMat* resultTrans;
	resultTrans = cvCreateMat(8, 4, CV_32FC1);
	cvTranspose(result_cam, resultTrans);
	/*printf("\n PRINT THE WORLD COORDINATE IN CAMERA COORDINATE\n");
	int ca;
	for (ca = 0; ca < 8; ca++) {
		printf("%f\t\t%f\t\t%f\t\t%f\n", resultTrans->data.fl[ca * 4],
				resultTrans->data.fl[ca * 4 + 1],
				resultTrans->data.fl[ca * 4 + 2],
				resultTrans->data.fl[ca * 4 + 3]);

	}
*/
//=============================================================================================

	// TEST NEW PIXEL COORDINATES

	pixel_coords = cvCreateMat(6, 4, CV_32FC1);
	float pixelValue[] = { 141, 457, 1, 1, 120, 382, 1, 1, 80, 231, 1, 1, 558,
			333, 1, 1, 565, 252, 1, 1, 471, 130, 1, 1 };
	int pix;
	for (pix = 0; pix < 6; pix++) {
		pixel_coords->data.fl[pix * 4] = pixelValue[pix * 4];

		pixel_coords->data.fl[pix * 4 + 1] = pixelValue[pix * 4 + 1];
		pixel_coords->data.fl[pix * 4 + 2] = pixelValue[pix * 4 + 2];
		pixel_coords->data.fl[pix * 4 + 3] = pixelValue[pix * 4 + 3];
	}
	//==============================================================================================
	/*  CvMat* pixel_coords1;
	 pixel_coords1 = cvCreateMat(6,3, CV_32FC1);
	 float pixelValue1[]= {  141, 457,1,
	 120 ,382,1,
	 80, 231,1,
	 558,333,1,
	 565, 252,1,
	 471,130,1} ;
	 int pix1;
	 for (pix1= 0 ; pix1 <6; pix1++){
	 pixel_coords1->data.fl[pix1*3]= pixelValue1[pix1*3];

	 pixel_coords1->data.fl[pix1*4+ 1]= pixelValue1[pix1*4 +1];
	 pixel_coords1->data.fl[pix1*4+ 2]= pixelValue1[pix1*4 +2];

	 }
	 CvMat* cam_iv;
	 CvMat* inv_pixel;
	 inv_pixel=cvCreateMat(3,6,CV_32FC1);
	 cam_iv = cvCreateMat(3,3,CV_32FC1);
	 cvTranspose(pixel_coords1, inv_pixel);
	 cvInvert(camera_matrix,cam_iv, CV_LU );
	 CvMat* cam_value;
	 cam_value = cvCreateMat(3,6,CV_32FC1);
	 cvGEMM(cam_iv,inv_pixel, alpha, optional_mat, beta, cam_value, tABC );
	 CvMat* in_cam_value;
	 in_cam_value = cvCreateMat(6,3,CV_32FC1);
	 cvTranspose(cam_value, in_cam_value);

	 printf("\n THE COORDINATES IN THE CAMERA NEW COORDINATE\n");
	 int ss;
	 for (ss=0; ss<6;ss++)
	 {
	 printf("%f\t\t%f\t\t%f\n", in_cam_value->data.fl[ss*3],in_cam_value->data.fl[ss*3 +1],
	 in_cam_value->data.fl[ss*3+2]) ;*/
	//  }
	//////////////////////////////////////////////////////////////
	CvMat* trans_matrix_camera_tool;
	trans_matrix_camera_tool = cvCreateMat(4, 4, CV_32FC1);
	cvGEMM(newTransformation_mat, transformation_matrix1, alpha, optional_mat,
			beta, trans_matrix_camera_tool, tABC);
	printf("\n\t\t\tTHE FINAL RESULT OF TRANSFORMATION MATRIX");
	printf(
			"\n THE TRANSFORMATION MATRIX FROM CAMERA TO TARGET- TARGET TO TOOL/ROBOT END EFFECTOR");
	printf(
			"\n\t\t\t CAMERA TO ROBOT TOOL FRAME -CAMERA POSE IN TOOL/END EFFECTOR FRAME\n");

	int ft;
	for (ft = 0; ft < 4; ft++) {
		printf("%f\t\t%f\t\t%f\t\t%f\n",
				trans_matrix_camera_tool->data.fl[ft * 4],
				trans_matrix_camera_tool->data.fl[ft * 4 + 1],
				trans_matrix_camera_tool->data.fl[ft * 4 + 2],
				trans_matrix_camera_tool->data.fl[ft * 4 + 3]);

	}
	//   =================================================================================================================
	//TEST
	CvMat* image_camera;
	CvMat* image_tool;
	image_camera = cvCreateMat(4, 4, CV_32FC1);
	image_tool = cvCreateMat(4, 4, CV_32FC1);

	CvMat* pixel_coords_trans;
	pixel_coords_trans = cvCreateMat(4, 6, CV_32FC1);
	CvMat* tool_tip;
	tool_tip = cvCreateMat(4, 6, CV_32FC1);
	cvTranspose(pixel_coords, pixel_coords_trans);

	cvGEMM(inv_image_world, transformation_matrix, alpha, optional_mat, beta,
			image_camera, tABC);
	cvGEMM(image_camera, trans_matrix_camera_tool, alpha, optional_mat, beta,
			image_tool, tABC);

// image to camera transformation - pixel coordinate to camera coordinates
	CvMat* camera_tip;
	camera_tip = cvCreateMat(4, 6, CV_32FC1);

	cvGEMM(image_camera, pixel_coords_trans, alpha, optional_mat, beta,
			camera_tip, tABC);
	cvGEMM(image_tool, pixel_coords_trans, alpha, optional_mat, beta, tool_tip,
			tABC);

	CvMat* trans_camera_tip;
	trans_camera_tip = cvCreateMat(6, 4, CV_32FC1);
	cvTranspose(camera_tip, trans_camera_tip);
	/*/////////////print in camera coordinate
	int tata;
	printf("\n THE COORDINATES IN THE CAMERA COORDINATE FRAME COORDINATE\n");
	for (tata = 0; tata < 6; tata++) {
		printf("%f\t\t%f\t\t%f\t\t%f\n", trans_camera_tip->data.fl[tata * 4],
				trans_camera_tip->data.fl[tata * 4 + 1],
				trans_camera_tip->data.fl[tata * 4 + 2],
				trans_camera_tip->data.fl[tata * 4 + 3]);
	}*/

	CvMat* trans_tool_tip;
	trans_tool_tip = cvCreateMat(6, 4, CV_32FC1);
	cvTranspose(tool_tip, trans_tool_tip);

	/*printf("\n THE COORDINATES IN THE TOOL FRAME COORDINATE\n");
	for (ccc = 0; ccc < 6; ccc++) {
		printf("%f\t\t%f\t\t%f\t\t%f\n", trans_tool_tip->data.fl[ccc * 4],
				trans_tool_tip->data.fl[ccc * 4 + 1],
				trans_tool_tip->data.fl[ccc * 4 + 2],
				trans_tool_tip->data.fl[ccc * 4 + 3]);
	}
*/
	/////////////////////////////////////////////////////////////////////////////////////////////////

	printf(
			"\n WHAT WE ARE INTERESTED IN IS END EFFECTOR FRAME IN CAMERA FRAME\n");
	printf("\t\t\t END EFFECTOR POSE IN CAMERA FRAME\n");
	CvMat* end_effector_camera;
	end_effector_camera = cvCreateMat(4, 4, CV_32FC1);
	int method = CV_LU; // BASED ON GAUSSIAN ELIMINATION -The determinant of the source returned-
	//singularity checked
	cvInvert(trans_matrix_camera_tool, end_effector_camera, method);
	int ftf;
	for (ftf = 0; ftf < 4; ftf++) {
		printf("\n%f\t\t%f\t\t\t%f\t\t%f\n",
				end_effector_camera->data.fl[ftf * 4],
				end_effector_camera->data.fl[ftf * 4 + 1],
				end_effector_camera->data.fl[ftf * 4 + 2],
				end_effector_camera->data.fl[ftf * 4 + 3]);

	}
	euler_angle(end_effector_camera);
	quaternion_angle(end_effector_camera);

	return 0;
}
void euler_angle(CvMat* mat) {

	printf("\n THE EULER ANGLE OF TRANSFORMATION MATRIX\n");
	float thetaX, thetaY, thetaZ, thetaNewX, thetaNewY, thetaNewZ;
	double r3233 = (mat->data.fl[9] * mat->data.fl[9])
			+ (mat->data.fl[10] * mat->data.fl[10]);
	//printf("r323 = %f\n", r3233);
	thetaX = atan2(mat->data.fl[9], mat->data.fl[10]);
	thetaNewX = (thetaX * 180) / PI;

	thetaY = atan2(-mat->data.fl[8], sqrt(r3233));
	thetaNewY = (thetaY * 180) / PI;
	thetaZ = atan2(mat->data.fl[4], mat->data.fl[0]);
	thetaNewZ = (thetaZ * 180) / PI;
	printf("\n EULER ANGE IN RADIAN\n thetaX=%f\n thetaY=%f\n thetaZ =%f\n",
			thetaX, thetaY, thetaZ);
	printf(
			"\n EULER ANGLE IN DEGREE\n thetaNewX =%f\nthetNewY =%f\nthetaNewZ=%f\n",
			thetaNewX, thetaNewY, thetaNewZ);

}

void quaternion_angle(CvMat* mmat) {

	float q1, q2, q3, q4; // quaternion angles q = q1 +q2 *i + q3 *j + q4* k

// q4 = 1/2(sqrt(1+ A11 +A22 + A33))
// q1 = 1/(4*q4)(A32-A23)
// q2 = 1/(4*q4)(A13 - A31)
// q3 = 1/(4*q4)(A21-A12)
	q4 = (1.0 / 2)
			* sqrt(
					1.0 + mmat->data.fl[0] + mmat->data.fl[5]
							+ mmat->data.fl[10]);
	q1 = 1 / (4.0 * q4) * (mmat->data.fl[9] - mmat->data.fl[6]);
	q2 = 1 / (4.0 * q4) * (mmat->data.fl[2] - mmat->data.fl[8]);
	q3 = 1 / (4.0 * q4) * (mmat->data.fl[4] - mmat->data.fl[1]);

	printf("\n THE QUATERNION ANGLE CONVERSION OF ROTATION MATRIX\n");
	printf("%f\n%f\n%f\n%f\n", q1, q2, q3, q4);
	printf("\nthe square of quaternion%f \n",
			q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);

}

/*

 buildRealPose( float *rot, float *trans )
 {
 glMatrixMode( GL_MODELVIEW );
 glLoadIdentity();
 glTranslatef( trans[0], trans[1], trans[2] );
 glRotatef( rot[0], 1.0f, 0.0f, 0.0f );
 glRotatef( rot[1], 0.0f, 1.0f, 0.0f );
 glRotatef( rot[2], 0.0f, 0.0f, 1.0f );
 glGetFloatv(GL_MODELVIEW_MATRIX, poseReal );
 }
 */

