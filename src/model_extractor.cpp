#include <Xn_sensor.h>
#include <myfuncs.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/registration/transforms.h>
#include <pcl/registration/icp.h>
#include <poseestimator.h>
#include <opencv2/core/eigen.hpp>

#define WIDTH	640
#define	HEIGHT	480

bool first = true;

void boxFilter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, Pose pose){		
	//Transform the cloud
	//convert the tranform from our fiducial markers to the Eigen
    Eigen::Matrix<float, 3, 3> R;
    Eigen::Vector3f T;
    cv::cv2eigen(pose.getT(), T);
    cv::cv2eigen(pose.getR(), R);
    //get the inverse transform to bring the point cloud's into the
    //same coordinate frame
    Eigen::Affine3f transform;
    transform = Eigen::AngleAxisf(R.transpose());
    transform *= Eigen::Translation3f(-T);
    //transform the cloud in place
    pcl::transformPointCloud(*cloud, *cloud, transform);
	
	//Define the box
	float box = 200.00;
	pcl::PassThrough<pcl::PointXYZRGB> pass_z, pass_x, pass_y;
	//Filters in x
	pass_x.setFilterFieldName("x");
	pass_x.setFilterLimits(-box, box);
	pass_x.setInputCloud(cloud);
	pass_x.filter(*cloud);
	//Filters in y
	pass_y.setFilterFieldName("y");
	pass_y.setFilterLimits(-box, box);
	pass_y.setInputCloud(cloud);
	pass_y.filter(*cloud);
	//Filters in z
	pass_z.setFilterFieldName("z");
	pass_z.setFilterLimits(0,box);
	pass_z.setInputCloud(cloud);
	pass_z.filter(*cloud);	
}

void buildModel(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,pcl::PointCloud<pcl::PointXYZRGB>::Ptr model){
	if (first){
		*model = (*cloud);
		first=false;
	}
	else {
		pcl::IterativeClosestPoint<pcl::PointXYZRGB,pcl::PointXYZRGB> ICP;
		ICP.setMaxCorrespondenceDistance( 10 );
		ICP.setTransformationEpsilon( 0.5 );
		ICP.setMaximumIterations( 100 );
		ICP.setInputTarget( boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB> >(*model) );
		ICP.setInputCloud(cloud);
		pcl::PointCloud<pcl::PointXYZRGB> Final;
		ICP.align( Final );
		if( ICP.hasConverged() )
			*model += Final;
	}
}

int main (int argc,char* argv[]){
	if (argc != 2 && argc != 3){
		printf("usage:\n %s /path/to/recoding/filename.oni\n",argv[0]);
		return 0;
	}
	Xn_sensor sensor(WIDTH,HEIGHT);
	sensor.play(argv[1],false);
	cvNamedWindow( "Model Extractor Viewer", 1 );
	IplImage* rgb_image = cvCreateImageHeader(cvSize(WIDTH,HEIGHT), 8, 3);
	IplImage* test = cvCreateImageHeader(cvSize(WIDTH,HEIGHT), 8, 3);
	IplImage* gray = cvCreateImage(cvSize(WIDTH,HEIGHT), 8, 1);
	Mat img;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr model (new pcl::PointCloud<pcl::PointXYZRGB>);
	
	pcl::visualization::CloudViewer viewer("Model Extractor Viewer");

	
	//Read Fiducial from file
	Fiducial fiducial("fiducial.yml");
	Pose pose;
	  while(!viewer.wasStopped() && !sensor.endPlaying()){
		//Get the frame 
		sensor.update();
		sensor.getPCL(cloud);
		cvSetData(rgb_image,sensor.rgb,rgb_image->widthStep);
		//Estimate Camera Pose from fiducial
		cvCvtColor(rgb_image,gray,CV_BGR2GRAY);
		if (fiducial.find(gray,true)){
			pose.estimate(gray,fiducial);
			//fiducial.draw(rgb_image);
		}
		if (pose.isFound()){
			printf("Rotation");
			printMat<double>(pose.getR());
			printf("Translation");
			printMat<double>(pose.getT());
			//Segment volume around the fiducial
			boxFilter(cloud,pose);
			//Create 3D model
			buildModel(cloud,model);
		}
		viewer.showCloud (model);
	}
	pcl::io::savePCDFileASCII ("model.pcd", *model);
	sensor.shutdown();
	return 0;
}
