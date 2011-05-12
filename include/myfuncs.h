#include <XnCppWrapper.h>
#include <opencv2/opencv.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>


#pragma once

/*F*/
void xnpts2cvpts(XnPoint3D* xnpoints, CvMat* mat);
/*
*INPUT: Takes N 3D points in XnPoint3D* format
*OUTPUT: Returns an OpenCV matrix of N 3D points (M(N*3), one point per row [x y z])
*F*/

/*F*/
cv::Mat getCOM(const cv::Mat points);
/*
*INPUT: Takes a matrix of n 3D points (M(n*3), one point per row [x y z])
*OUTPUT: Returns the Center of Mass  as a row vector
*F*/

/*F*/
cv::Mat cvCalcXCovarMatrix(cv::Mat pts1,cv::Mat pts2,cv::Mat com1,cv::Mat com2);
/*
*INPUT: Takes 2 maxtrix of 3D points matrix of the same size (1 point per row) and the center of mass of each set.
*OUTPUT: Returns the Cross-Covariance Matrix
*F*/


void calcTransformSVD(cv::Mat model, cv::Mat observed, cv::Mat Rotation, cv::Mat Translation);
/*
*INPUT: Takes 2 maxtrix of 3D points matrix of the same size (1 point per row)
*OUTPUT: Returns the Least Squares 3D Transformation (Rot,Trans)
*F*/

void calcTransformHorn(const cv::Mat model,const cv::Mat observed, cv::Mat* Rotation, cv::Mat* Translation);
/*
*INPUT: Takes 2 maxtrix of 3D points matrix of the same size (1 point per row)
*OUTPUT: Returns the Least Squares 3D Transformation (Rot,Trans)
*F*/
 void printCvMat(CvMat* matrix);
// void printMat(Mat matrix);
template <class T>
void printMat(cv::Mat matrix){
	printf("\n");
	for (int row = 0;row<matrix.rows;row++){
		printf("|	");
		for (int col=0;col<matrix.cols;col++)
			printf("%f	",matrix.at<T>(row,col));
		printf("|\n");
	}
}
 
 void printMat2(cv::Mat matrix);
 //printrs a matrix
	


 int findTransformationRANSAC(const cv::Mat model_pts,const cv::Mat current_pts,int* consensus_set,cv::Mat Rotation, cv::Mat Translation,float error);
 /*
*INPUT: Proposed Matches with outliers: model_pts->current_pts
*OUTPUT: Returns the Least Squares 3D Transformation (Rot,Trans)
*F*/
 void ICP(const cv::Mat model,const cv::Mat observed, cv::Mat Rotation, cv::Mat Translation);

