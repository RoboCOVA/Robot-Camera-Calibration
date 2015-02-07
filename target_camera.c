#include"opencv2/highgui/highgui.hpp"
#include"opencv2/core/core.hpp"
#include"opencv2/calib3d/calib3d.hpp"
#include<stdio.h>
#include<string.h>

#include<math.h>
#include"target_camera.h"
#include "target_robot.h"
#include "given_image_coords.h"
//#define square_size 28
//void cvFindExtrinsicCameraParams2(const CvMat* object_points, const CvMat* image_points, const CvMat* camera_matrix,
	//	const CvMat* distortion_coeffs, CvMat* rotation_vector, CvMat* translation_vector, 0 );

void trans_target_camera()
{

int number_points = 4;

CvMat* model_points;
CvMat* image_points;

CvMat* rotation_matrix;

// initialization of the matrices/data
model_points = cvCreateMat(number_points,3,CV_32FC1);
image_points = cvCreateMat(number_points,2,CV_32FC1);//number_ points is the number of measurement
distortion_coefficients= cvCreateMat(5,1,CV_32FC1);
camera_matrix = cvCreateMat(3,3,CV_32FC1);
rotation_matrix = cvCreateMat(3,1,CV_32FC1);
translation_vector = cvCreateMat(3,1,CV_32FC1);


float model[]  = {square_size,square_size*6,0.01 ,       // sorted in 	X1,X2,X3,X4 column wise
	              square_size, square_size,0.02,
	     	      square_size*6,square_size, 0.03,
	     	      square_size*6,square_size*6,0.01};

float image[] = { 208.0,166.0,
		     	 176.0,335.0,
		     	 457.0,331.0,
		     	    444.0,159.0             };
//===========================================================================
   float cam_parameters[] = {1167.7086 ,0.0 ,324.2511,0.0, 1168.1241,
		   239.8463, 0.0, 0.0, 1.0};

/*double distortion[] = {0,0,0,
		                  k4,k5  };*/
  float distortion[] = {0.01419,-0.12238,  -0.0063, -0.005697, 0.0};
int i;
printf("\n model and image coordinates in target- camera transformation respectively\n");
for(i=0;i<number_points;i++)
{
model_points->data.fl[i*3] = model[i*3];
model_points->data.fl[i*3+1] = model[i*3 +1];
model_points->data.fl[i*3+2] = model[i*3 +2];

printf("%f \t\t\t%f\t\t\t\t %f\n ", model[i*3],model[i*3 +1],model[i*3 +2]);
image_points->data.fl[i*2]=    image[i*2];
image_points->data.fl[i*2+1]= image[i*2 +1];
printf("%f \t\t\t%f\n ", image[i*2],image[i*2 +1]);
}
 //double ddd = CV_MAT_ELEM(*model_points, double,0,0);
//printf("a=%f", ddd);

int j;
for (j=0;j<5;j++){
	distortion_coefficients->data.fl[i] = distortion[j];
printf("\n distortion coefficients %f\n", distortion[j]);
}
int k;
for(k=0;k<3;k++){

	camera_matrix->data.fl[k*3] = cam_parameters[k*3];
	camera_matrix->data.fl[k*3 +1] = cam_parameters[k*3 +1];
	camera_matrix->data.fl[k*3 +2] = cam_parameters[k*3+2];
	printf("%f \t\t\t%f\t\t\t\t %f\n ", cam_parameters[k*3],cam_parameters[k*3 +1],cam_parameters[k*3 +2]);
}
//int use_extrinsic_guess CV_DEFAULT();
cvFindExtrinsicCameraParams2(model_points, image_points,camera_matrix,distortion_coefficients,rotation_matrix,translation_vector,0);

// conversion from rotation vector/rotation matrix to rotation matrix/ rotation vector
CvMat* jacobian = 0;
rotationInMatrix = cvCreateMat(3,3,CV_32FC1);
cvRodrigues2(rotation_matrix,rotationInMatrix, jacobian );
float deterOfR = cvDet(rotationInMatrix);
printf("\n the determinant of rotation matrix \n%f\n",deterOfR);

//following gets the magnitude of the rotationVector, which is conveniently
//scaled to be the radians of rotation around the vector
//rotationVector represents the axis around which a rotation of degrees or radians happens in a counterclockwise direction
//(right hand rule )

// double radians[] = cvNorm(rotation_matrix, NULL, CV_L2, NULL);

printf("\n Rotation Vector target - camera coordinate \n");
        float rv[]={rotation_matrix->data.fl[0],rotation_matrix->data.fl[1], rotation_matrix->data.fl[2]};

       for (k=0;k<3;k++){
        printf("%f\n", rv[k]);
       }
       float tv[]={translation_vector->data.fl[0],translation_vector->data.fl[1],translation_vector->data.fl[2]};
    printf("\n the translation vector \n");
     int a;
     for (a= 0; a<3;a++){

    	 printf("%f\n",tv[a]);

     }
     //====================================================================================
     // transformation matrix
     CvMat* inverseRotation;
     inverseRotation = cvCreateMat(3,3,CV_32FC1);
     cvTranspose(rotationInMatrix, inverseRotation);
     printf("\n the inverse rotation matrix is the transpose of rotation matrix\n");
     int gg;
     for(gg=0;gg<3;gg++)
     {
    	 printf("%f\t\t\t%f\t\t\t\t%f\n", inverseRotation->data.fl[gg*3], inverseRotation->data.fl[gg*3 +1],
    			 inverseRotation->data.fl[gg*3 +2]);

     }
//===========================================================================================
     // translation vector inverse
     CvMat* inverseTranslation;
     inverseTranslation =cvCreateMat(3,1,CV_32FC1);
     CvMat* optional_mat = NULL;
      float alpha = 1.0;
      float beta = 0.0;
      int tABC = 0;
      cvGEMM(inverseRotation, translation_vector, alpha, optional_mat, beta,inverseTranslation, tABC);


       float rm[9];

     int x;
     printf("\n the rotation matrix entries \n");
        for(x=0;x<9;x++){
           rm[x]=rotationInMatrix->data.fl[x];
           printf("%f\n", rm[x]);
        }
        int f;
        printf("\n the rotation-matrix in matrix form \n");
        for(f = 0 ;f<3;f++)
        {
        printf(" %f\t\t%f\t\t%f\n", rm[f*3], rm[f*3+1],rm[f*3+2]);
        }
        // transformation matrix

        transformation_matrix = cvCreateMat(4,4,CV_32FC1);
        transformation_matrix->data.fl[12]=0;
        transformation_matrix->data.fl[13]=0;
        transformation_matrix->data.fl[14]=0;
        transformation_matrix->data.fl[15]=1;
        transformation_matrix->data.fl[3]=  translation_vector->data.fl[0];
        transformation_matrix->data.fl[7]= translation_vector->data.fl[1];
        transformation_matrix->data.fl[11] =translation_vector->data.fl[2];
        transformation_matrix->data.fl[0] = rotationInMatrix ->data.fl[0];
        transformation_matrix->data.fl[1] = rotationInMatrix ->data.fl[1];
        transformation_matrix->data.fl[2] = rotationInMatrix ->data.fl[2];
        transformation_matrix->data.fl[4] = rotationInMatrix ->data.fl[3];
        transformation_matrix->data.fl[5] = rotationInMatrix ->data.fl[4];
        transformation_matrix->data.fl[6] = rotationInMatrix ->data.fl[5];
        transformation_matrix->data.fl[8] = rotationInMatrix ->data.fl[6];
        transformation_matrix->data.fl[9] =rotationInMatrix ->data.fl[7];
        transformation_matrix->data.fl[10] = rotationInMatrix ->data.fl[8];


      printf("\n THE TRANSFORMATION MATRIX FROM TARGET TO CAMERA- TARGET FRAME IN CAMERA COORDINATE FRAME\n");


        int ffff;
        for(ffff=0;ffff<4; ffff++)
        {
        	printf("\n%f\t\t\t%f\t\t\t%f\t\t\t%f\n",transformation_matrix->data.fl[ffff*4],
        			transformation_matrix->data.fl[ffff*4 +1],
        			transformation_matrix->data.fl[ffff*4 +2],transformation_matrix->data.fl[ffff*4 +3]);

        	        }


   newTransformation_mat = cvCreateMat(4,4,CV_32FC1);

 cvInvert(transformation_matrix, newTransformation_mat, CV_LU);

 printf("\n THE TRANSFORMATION MATRIX FROM CAMERA TO TARGET- CAMERA IN TARGET COORDINATE FRAME\n");


        int ttt;
        for(ttt=0;ttt<4; ttt++)
        {
        	printf("\n%f\t\t\t%f\t\t\t%f\t\t\t%f\n",newTransformation_mat->data.fl[ttt*4],
        			newTransformation_mat->data.fl[ttt*4 +1],
        			newTransformation_mat->data.fl[ttt*4 +2],newTransformation_mat->data.fl[ttt*4 +3]);

        	        }



 // =========================================================================================
        // the euler angle from rotation matrix- the three euler angles
        // the rotation matrix R = ZYX
         float thetaX, thetaY, thetaZ, thetaNewX, thetaNewY, thetaNewZ;
           float r3233= (rotationInMatrix->data.fl[7]*rotationInMatrix->data.fl[7]) +(rotationInMatrix->data.fl[8]*rotationInMatrix->data.fl[8]);
   //printf("r323 = %f\n", r3233);
        	thetaX = atan2(rotationInMatrix->data.fl[7],rotationInMatrix->data.fl[8] );
        	  thetaNewX = (thetaX*180)/PI;

        		   	thetaY = atan2(-rotationInMatrix->data.fl[6],sqrt(r3233));
                           thetaNewY = (thetaY * 180)/PI;
        					thetaZ = atan2(rotationInMatrix->data.fl[3], rotationInMatrix->data.fl[0]);
                               thetaNewZ = (thetaZ *180)/PI;
printf("\n the euler angles are= \n thetaX=%f\n thetaY=%f\n thetaZ =%f\n",thetaX, thetaY,thetaZ);
printf("\n the euler angles in degree=\n thetaNewX =%f\nthetNewY =%f\nthetaNewZ=%f\n", thetaNewX, thetaNewY, thetaNewZ);



/*
        float temp;
        float outputMat[3][3];
        int c,d,e;
            for( c = 0; c < 3; c++ )
            {
                for( d = 0; d< 3; d++ )
                {
                    temp = CV_MAT_ELEM(*rotation_matrix, float, c, d);
                    assert(!cvIsNaN(temp));

                   outputMat[i][j] = temp;
                  // printf("rotation matrix = %f\n", outputMat[c][d]);
                }
                //printf("\n");
            }

            for( e = 0; e < 3; e++ )
            {
                temp = CV_MAT_ELEM(*translation_vector, float, 0, e);
                assert(!cvIsNaN(temp));

                // multiply units to get meters in real world

                //cameraTransformationOutput[3][i] = temp;

            }*/
               /*cvReleaseMat(&rotation_matrix);
                cvReleaseMat(&rotationInMatrix);
                cvReleaseMat(&translation_vector);
               cvReleaseMat(&distortion_coefficients);
               cvReleaseMat(&camera_matrix);
                cvReleaseMat(&image_points);
               cvReleaseMat(&model_points);
                cvReleaseMat(&rotationInMatrix);

*/
        }




/*
/print the value of rotation matrix
CvMat* rotation_value= cvCreateMat(3,3,CV_64FC1);
int x,y;//CvMat* translation_value= cvCreateMat(1,3,CV_64FC1);
for(x =0;x<3;x++){
	for (y=0;y<3;y++){
float x2 = (CV_MAT_ELEM(*rotation_matrix,float,x,y));
CV_MAT_ELEM(*rotation_matrix, float, 0, x) = CV_MAT_ELEM(*rotation_matrix, float, x, y);
   printf("rotation_value =%f\n", x2); //rotation_matrix(x,y)
    printf("%f",CV_MAT_ELEM(*rotation_matrix, float, 0, x));
}
///printf("\n");
//}
// string file = dir + "Rotation" + substring + ".xml";
  // cvSave(file.c_str(), rotation_matrix);
 // file = dir + "Translation" + substring + ".xml";
 // cvSave(file.c_str(), translation_vector);


double rotation_result[]= {
		              &rotation_matrix[0],&rotation_matrix[1],&rotation_matrix[2],
		               &rotation_matrix[3],&rotation_matrix[4],&rotation_matrix[5],
		                 &rotation_matrix[6],&rotation_matrix[7],&rotation_matrix[8]
                                             };
double translation_result ={
		translation_vector[0],translation_vector[1],translation_vector[2]
};

int t;
for(t=0;t<3;t++){
rotation_value->data.db[i*3] = rotation_result[i*3] ;
rotation_value->data.db[i*3+1] = rotation_result[i*3+1] ;
rotation_value->data.db[i*3+2] = rotation_result[i*3 +2] ;
translation_value->data.db[i]= translation_result[i];
	printf("the rotation matrix = \n%d,\t\t\t ,%d,\t\t\\t%d\n ", rotation_result[i*3],rotation_result[i*3+1],rotation_result[i*3+2]);
   printf("the translation vector= \n%d\n", translation_result[i]);

CvMat model_points, image_points;
int step = CV_AUTOSTEP;
 cvInitMatHeader(&model_points, 3,4,CV_64FC1,model, step);
  cvInitMatHeader(&image_points,3,4, CV_64FC1, image, step);

  // camera matrix -> intrinsic camera parameters


  CvMat camera_matrix,distortion_coefficient;
   cvInitMatHeader(&camera_matrix,3,3,	CV_64FC1,cam_parameters, step );
   cvInitMatHeader(&distortion_coefficient, 5,1,CV_64FC1, distortion, step);

   // initialization o the rotation matrix and the translation vector
   CvMat* rotation_matrix = cvCreateMat(3,3,	CV_64FC1);
   CvMat* translation_vector =cvCreateMat(3,1, CV_64FC1);


   // solve pnp point to point crosspondence
   cvFindExtrinsicCameraParams2(model_points, image_points,distortion_coefficient,rotation_matrix,translation_vector, 0);
// extract rotation and tranlation here
   // object coordinate in camera frame object -> camera
// we need the inverse- where the camera is in the world coordinate camera-> world
   Rodrigues (rotation_matrix, rotation_matrix);
*/
