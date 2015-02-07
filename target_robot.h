
#ifndef TARGET_ROBOT_H_
#define TARGET_ROBOT_H_

#include<stdio.h>

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/calib3d/calib3d.hpp"
CvMat* transformation_matrix1;

void trans_target_robot();


//CvMat* pass_trans_target_robot(CvMat* mat);
#endif /* TARGET_ROBOT_H_ */
