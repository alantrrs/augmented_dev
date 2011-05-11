#include <poseestimator.h>
#include <iostream>


Fiducial::Fiducial(char* file){
	FileStorage fs(file,FileStorage::READ);
	const FileNode fd= fs["fiducial"];
	FileNode mat = fd["templates"];
	Mat pts;
	read(mat[0],pts,Mat());
	points.resize(pts.rows);
	points.assign (pts.begin<Point3f> (),pts.end<Point3f> ());
	FileNode cc = fd["corner_counts"];
	size.width = (int)cc[0]["width"];
	size.height = (int)cc[0]["height"];
	found=false;
}

bool Fiducial::find(Mat gray){
	found = findChessboardCorners(gray, size, corners,CV_CALIB_CB_ADAPTIVE_THRESH 
														+ CV_CALIB_CB_NORMALIZE_IMAGE 
														+ CV_CALIB_CB_FAST_CHECK);
	//accurate 
	/*if(found){
	  cornerSubPix(gray, corners, Size(11, 11), Size(-1, -1),TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
	}
	*/
	return found;
}

Pose::Pose()
{
	rvec =  Mat_<double>::zeros(3, 1);
	tvec = Mat_<double>::zeros(3, 1);
	found =  false;
}

void PoseEstimator::cvEstimate(Mat gray,Fiducial& f,Pose& p){

	printf("Rotation");
			printMat<double>(p.rvec);
			printf("Translation");
			printMat<double>(p.tvec);
	if (f.found){
		printf("Corners2d:");
		printMat2(Mat(f.corners));
		printf("Points3d:");
		printMat2(Mat(f.points));
		Mat K = (Mat_<float>(3,3) << 525., 0., 320., 0., 525., 240., 0., 0., 1.);
		solvePnP (Mat(f.points).t(), Mat(f.corners), K, Mat(), p.rvec, p.tvec, false);
		p.found = true;
	}
	else
		p.found = false;
}
	
void PoseEstimator::xnEstimate(Mat gray,Fiducial& f,Pose& p){

}

void PoseEstimator::draw(Mat rgb,Fiducial f){
	drawChessboardCorners(rgb,f.size,f.corners,f.found);
}
	
