#include <opencv2/opencv.hpp>
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
	void draw(Mat rgb_img);
	friend class Pose;
};

class Pose {
	Mat rvec;
	Mat tvec;
	bool found;
public:
	Pose();
	void estimate(Mat gray, Fiducial& f);
	inline bool isFound(){ return found; }
	inline Mat getR(){return rvec;}
	inline Mat getT(){return tvec;}
};
