#include<stdio.h>
#include<string.h>
#include<math.h>
#include<stdlib.h>
#include"opencv2/highgui/highgui.hpp"
#include"opencv2/core/core.hpp"
#include"opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/flann/flann.hpp"
#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/legacy/legacy.hpp"
#include "opencv2/contrib/contrib.hpp"
#include "given_image_coords.h"
#include "target_camera.h"


void test_point_cloud()
{

	 CvMat* optional_mat = NULL;
		   float alpha = 1.0;
		   float beta = 0.0;
		   int tABC = 0;
		   // this test is under the assumption that the distortion effect is low
		   // the effect of scaling is also neglected
		   CvMat* rotation_translation;
		   rotation_translation = cvCreateMat(3,4,CV_32FC1);
 CvMat* intrinsic_extrinsic;
 intrinsic_extrinsic = cvCreateMat(3,4,CV_32FC1);
 //CvMat* extrinsic_translation; /// this is 3* 4 -pose of the object in camera coordinate frame
 CvMat* inv_transformation;
 inv_transformation = cvCreateMat(4,4,CV_32FC1);
 int method = CV_LU;
cvInvert(transformation_matrix,inv_transformation,method);
assert(!cvIsNaN(inv_transformation->data.fl[2]));

//  extract the 3*4 extrinsic camera parameters - rotation matrix and translation vector
int ab;
for (ab= 0 ;ab<12;ab++)
{
rotation_translation->data.fl[ab] = transformation_matrix->data.fl[ab];

}
int abc;
printf("\n R|T -THE CAMERA MATRIX- EMBEDDED INTRINSIC AND EXTRINSIC PARAMETERS\n");
for(abc =0; abc<3;abc++)
{

	printf("%f\t\t%f\t\t\%f\t\t%f\n", rotation_translation->data.fl[abc *3],rotation_translation->data.fl[abc*3 +1],
			rotation_translation->data.fl[abc*3 +2],
			rotation_translation->data.fl[abc*3 +3]);

}
 cvGEMM(camera_matrix,rotation_translation,alpha,optional_mat, beta, intrinsic_extrinsic,tABC);

 assert(!cvIsNaN(intrinsic_extrinsic->data.fl[2]));
/// intrinsic_extrinsic matrix transforms points in world frame to corresponding points in image frame
 //we need  the reverse - given pints in image plane where is the point in the camera frame
 CvMat* intrinsicExTrans;
 intrinsicExTrans = cvCreateMat(4,4,CV_32FC1);
 int tr;
 for(tr = 0 ; tr<12 ; tr++)
		{
intrinsicExTrans->data.fl[tr]= intrinsic_extrinsic->data.fl[tr];

		}
 intrinsicExTrans->data.fl[12] = 0;
 intrinsicExTrans->data.fl[13] = 0;
 intrinsicExTrans->data.fl[14] = 0;
 intrinsicExTrans->data.fl[15] = 1;

 int aa;
 printf("\n GIVEN WORLD COORDINATE- TARGET POINT CLOUD IN IMAGE PLANE\n");
 printf( "transformation from world to image frame\n");
 for (aa = 0 ; aa < 4;aa++)
 {
	printf( " %f\t\t%f\t\t%f\t\t%f\n",intrinsicExTrans->data.fl[aa*4],
			intrinsicExTrans->data.fl[aa*4 + 1],intrinsicExTrans->data.fl[aa*4 +2],
			intrinsicExTrans->data.fl[aa*4+3]	);

 }
// to find the transformation from image to world do inverse transformation

 inv_image_world = cvCreateMat(4,4,CV_32FC1);
 cvInvert(intrinsicExTrans, inv_image_world,method);

 int tt;
  printf("\n GIVEN IMAGE COORDINATE- IMAGE POINT IN WORLD FRAME\n");
  for (tt= 0 ; tt < 4;tt++)
  {
 	printf( "\n%f\t\t%f\t\t%f\t\t%f\n",inv_image_world->data.fl[tt*4],
 			inv_image_world->data.fl[tt*4 + 1],inv_image_world->data.fl[tt*4 +2],
 			inv_image_world->data.fl[tt*4+3]	);
  }


///====================================================================================================
  	    IplImage* distorted_img ;

  	  distorted_img  = cvLoadImage("test.bmp", CV_LOAD_IMAGE_GRAYSCALE); // argv[1]
IplImage* checkBoard = cvCloneImage(distorted_img);

  	/*cvNamedWindow("distorted_image", CV_WINDOW_AUTOSIZE);
  	   cvNamedWindow("undistorted_image", CV_WINDOW_AUTOSIZE );
  cvNamedWindow("smoothImage", CV_WINDOW_AUTOSIZE);
  cvNamedWindow("edge_detection", CV_WINDOW_AUTOSIZE);
  cvNamedWindow("corner_detected", CV_WINDOW_AUTOSIZE);*/
  	   ////cvShowImage("distorted_image",distorted_img);// distorted image

  	 IplImage* smooth_image= cvCreateImage( cvGetSize(distorted_img),8, 1 );

  	   cvSmooth(distorted_img,smooth_image, CV_GAUSSIAN, 3,3 ,0,0);
  	//// cvShowImage("smoothImage", smooth_image);

  	 double threshold1 =10.0;
  	   double threshold2 =150.0;
  	    int aperture_size =3;
  	   // IplImage* edges;
   	   // IplImage* edges= cvCreateImage( cvGetSize(distorted_img),8, 1 );
  	    //cvCopy(distorted_img, edges, NULL);
  	    IplImage* edges = cvCreateImage( cvSize(distorted_img->width,distorted_img->height),distorted_img->depth,distorted_img->nChannels );
  	    cvCanny(smooth_image, edges, threshold1, threshold2,  aperture_size );
  	  ////  cvShowImage("edge_detection", edges);

// find contours
  	   // CvMemStorage* storage = cvCreateMemStorage();
  	    // find the corners given the image of chessboard




  	 CvSize pattern_size =cvSize(6,6);
  	 int  board_width =6;
    int board_height =6;
  	 int  numberOfBoard = board_width*board_height;

  	CvPoint2D32f* corners = (CvPoint2D32f*)cvAlloc(numberOfBoard * sizeof(CvPoint2D32f));
  	 int corner_count ;
  	int flagOf  = CV_CALIB_CB_ADAPTIVE_THRESH|CV_CALIB_CB_NORMALIZE_IMAGE;

  	//Find chessboard corners:

  	int found = cvFindChessboardCorners (
  			 distorted_img,  pattern_size, corners,  &corner_count , flagOf );



/*if (found == 0){
// find subpixel corner to increase accuracy of location of pixel
   printf("\nthe corner is not found\n");
}*/
 cvFindCornerSubPix(checkBoard, corners, corner_count, cvSize(10,10),cvSize(-1,-1), cvTermCriteria(
  	CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 30, 0.1 ));

      /*printf("the number of corner is %i\n", corner_count);
*/

    //IplImage *cornerResult = cvCreateImage(cvGetSize(distorted_img), distorted_img->depth, distorted_img->nChannels);
  	cvDrawChessboardCorners(checkBoard, pattern_size, corners,
  		corner_count, found);
  	         assert(corners);


  	printf("\n%f\t\t%f\t\t%i\n", corners->x, corners->y, found);

    ////cvShowImage("corner_detected",checkBoard);





 if (!camera_matrix && distortion_coefficients){
printf("check camera and distortion coefficient");
 }

IplImage* mapx = cvCreateImage( cvGetSize(distorted_img), IPL_DEPTH_32F, 1 );
IplImage* mapy = cvCreateImage( cvGetSize(distorted_img), IPL_DEPTH_32F, 1 );


cvInitUndistortMap(
camera_matrix, distortion_coefficients,mapx, mapy
                   );
if (!mapx && mapy)
{
printf("mapping in x and y direction is none\n");
}
IplImage* mapped = cvCloneImage(distorted_img);
int flags = CV_INTER_LINEAR|CV_WARP_FILL_OUTLIERS;
CvScalar fillval = cvScalarAll(0) ;

//cvRemap( distorted_img, undist_image, mapx, mapy,flags, fillval);

cvRemap( mapped, distorted_img, mapx, mapy,flags, fillval);
 // cvReleaseImage(&mapped);
 printf("%i\t\t\%i\n",distorted_img->height,distorted_img->width);


  /*IplImage* r = cvCreateImage(cvGetSize(distorted_img), IPL_DEPTH_8U, 1);
  IplImage* g = cvCreateImage(cvGetSize(distorted_img), IPL_DEPTH_8U, 1);
  IplImage* b = cvCreateImage(cvGetSize(distorted_img), IPL_DEPTH_8U, 1);
 IplImage* r2 = cvCreateImage(cvGetSize(distorted_img), IPL_DEPTH_8U, 1);
  IplImage* g2 = cvCreateImage(cvGetSize(distorted_img), IPL_DEPTH_8U, 1);
  IplImage* b2 = cvCreateImage(cvGetSize(distorted_img), IPL_DEPTH_8U, 1);
  cvSplit(distorted_img, r, g, b, NULL);
  cvRemap(r, r2, mapx, mapy);
  cvRemap(g, g2, mapx, mapy);
  cvRemap(b, b2, mapx, mapy);
  cvMerge(r2, g2, b2, NULL, remapped);*/
if (!distorted_img)
{
printf("the image is not distorted\n");
}
// show the result

 ///  cvShowImage("undistorted_image",distorted_img);// undistorted  image

cvSaveImage("/home/tesfu/workspace/undistorted.bmp",distorted_img,0);





   int c = cvWaitKey(10000);
   if(c == 'p'){
   c = 0;
   while(c !=  'p'&& c != 27){
   c = cvWaitKey(250);
   }
   }
   if(c == 27)
   {
 printf("done");
   }







  // save undistorted image
    //char* filename = "undistorted.bmp";
   // int* params =0;
   // cvSaveImage(filename,undist_image,  params );
      cvReleaseImage(&smooth_image);
     cvReleaseImage(&edges);
    cvReleaseImage(&distorted_img);
   //cvReleaseImage(&undist_image);
    ////cvDestroyWindow("distorted_image");
    ///cvDestroyWindow("undistorted_image");
    cvReleaseMat(&camera_matrix);
    cvReleaseMat(&distortion_coefficients);
/////////////////////////////////////////////////////////////////////////////////////



// inverse camera  to target frame to  obtain  target to camera transformation

  CvMat* inv_object_cam;
  inv_object_cam = cvCreateMat(4,4,CV_32FC1);
  cvInvert(transformation_matrix, inv_object_cam,method);


}







