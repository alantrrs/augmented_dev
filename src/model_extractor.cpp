#include <Xn_sensor.h>
#include <myfuncs.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <poseestimator.h>


#define WIDTH	640
#define	HEIGHT	480

using namespace xn;

int main (int argc,char* argv[]){
	if (argc != 2 && argc != 3){
		printf("usage:\n %s /path/to/recoding/filename.oni\n",argv[0]);
		return 0;
	}
	Xn_sensor sensor(WIDTH,HEIGHT);
	sensor.play(argv[1]);
	
	IplImage* rgb_image = cvCreateImageHeader(cvSize(WIDTH,HEIGHT), 8, 3);
	IplImage* gray = cvCreateImage(cvSize(WIDTH,HEIGHT), 8, 1);
	Mat img;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
	cloud->width  = sensor.getX();
	cloud->height = sensor.getY();
	cloud->points.resize (cloud->width * cloud->height);
	pcl::visualization::CloudViewer viewer("Model Extractor Viewer");
	
	//Read Fiducial from file
	Fiducial fiducial("fiducial.yml");
	Pose pose;
	  while(!viewer.wasStopped()){
		//Get the frame 
		sensor.update();
		sensor.getPCL(cloud);
		sensor.getImage(rgb_image);
		//Estimate Camera Pose from fiducial
		cvCvtColor(rgb_image,gray,CV_BGR2GRAY);
		//img=rgb_image;
		if (fiducial.find(gray)){
			pose.estimate(gray,fiducial);
		}
		if (pose.found()){
			//pe.draw(img,fiducial);
			printf("Rotation");
			printMat<double>(pose.rvec);
			printf("Translation");
			printMat<double>(pose.tvec);
		}
		//Segment volume around the fiducial			
		//Create 3D model
		viewer.showCloud (cloud);
	}
	sensor.shutdown();
return 0;
}
