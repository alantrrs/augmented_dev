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
	inline vector<Point3f> getPoints(){return points;}
	inline vector<Point2f> getCorners(){return corners;}
	bool find(Mat gray,bool accurate);
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
	inline Mat getR(){Mat R; cv::Rodrigues(rvec, R); return R;}
	inline Mat getT(){return tvec;}
};
