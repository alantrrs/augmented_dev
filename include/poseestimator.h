#include <opencv2/core/core.hpp>
#include <myfuncs.h>
using namespace cv;

class Fiducial {
	Size size;
	vector<Point3f> points;
	vector<Point2f> corners;
	bool found;
	public:
	Fiducial(char* file);
	bool find(Mat gray);
};

class Pose {
	public:
	Pose();
	Mat rvec;
	Mat tvec;
	bool found;
};

class PoseEstimator{
	public:
	void cvEstimate(Mat gray,Fiducial& f,Pose& p);
	void xnEstimate(Mat gray,Fiducial& f, Pose& p);
	void draw(Mat rgb,Fiducial f);
};
