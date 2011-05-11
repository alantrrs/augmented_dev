#include <Xn_sensor.h>
#include <myfuncs.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <poseestimator.h>


#define WIDTH	640
#define	HEIGHT	480

int main (int argc,char* argv[]){
	if (argc != 2 && argc != 3){
		printf("usage:\n %s /path/to/recoding/filename.oni\n",argv[0]);
		return 0;
	}
	Xn_sensor sensor(WIDTH,HEIGHT);
	sensor.play(argv[1]);
	cvNamedWindow( "Model Extractor Viewer", 1 );
	IplImage* rgb_image = cvCreateImageHeader(cvSize(WIDTH,HEIGHT), 8, 3);
	IplImage* test = cvCreateImageHeader(cvSize(WIDTH,HEIGHT), 8, 3);
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
		cvSetData(rgb_image,sensor.rgb,rgb_image->widthStep);
		//Estimate Camera Pose from fiducial
		cvCvtColor(rgb_image,gray,CV_BGR2GRAY);
		if (fiducial.find(gray)){
			pose.estimate(gray,fiducial);
			//fiducial.draw(rgb_image);
		}
		if (pose.isFound()){
			printf("Rotation");
			printMat<double>(pose.getR());
			printf("Translation");
			printMat<double>(pose.getT());
		}
		//Segment volume around the fiducial
		/*
		int box = 200;
		pcl::PassThrough<Cloud_t::PointType> pass_z, pass_x, pass_y;
		pass_z.setFilterFieldName("s");
		pass_z.setFilterLimits(0.02, box);
		pass_x.setFilterFieldName("u");
		pass_x.setFilterLimits(-box, box);
		pass_y.setFilterFieldName("v");
		pass_y.setFilterLimits(-box, box);
		pass_z.setInputCloud(cloud_);
		pass_z.filter(*cloud_);
		pass_y.setInputCloud(cloud_);
		pass_y.filter(*cloud_);
		pass_x.setInputCloud(cloud_);
		pass_x.filter(*cloud_);
		 */ 			
		//Create 3D model
		viewer.showCloud (cloud);
	}
	sensor.shutdown();
return 0;
}
