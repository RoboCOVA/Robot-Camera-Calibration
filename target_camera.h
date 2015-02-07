
#ifndef TARGET_CAMERA_H_
#define TARGET_CAMERA_H_

#include<stdio.h>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/calib3d/calib3d.hpp"

#include "target_robot.h"

#define focal_length 8
#define fx 6
#define fy 6
#define cx 0
#define cy 0
#define k4 0
#define k5 0
#define PI 3.14134
#define square_size 29 // the unit is mm

CvMat* transformation_matrix;  // TRANSFORMATION FROM OBJECT TO CAMERA
CvMat* newTransformation_mat;   // TRANSFRMATION FROM CAMERA TO OBJECT
CvMat* camera_matrix;
CvMat* rotationInMatrix;
CvMat* translation_vector;
CvMat* distortion_coefficients;

void trans_target_camera();




//void trasformation_TarCam(CvMat*);
//void take_transformation(CvMat* mat);

#endif /* TARGET_CAMERA_H_ */
