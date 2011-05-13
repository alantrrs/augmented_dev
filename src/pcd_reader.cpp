 #include <iostream>
 #include <pcl/io/pcd_io.h>
 #include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include "pcl/filters/statistical_outlier_removal.h"
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>	

int main (int argc, char** argv){
	//Loading the cloud
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	if (pcl::io::loadPCDFile (argv[1], *cloud) == -1){
		std::cerr << "Couldn't read file" << std::endl;
		return (-1);
	}
   
	//Applying Statistical Outlier Removal
	pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
	sor.setInputCloud (cloud);
	sor.setMeanK (50);
	sor.setStddevMulThresh (1.0);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
	sor.filter (*cloud_filtered);
	/*
	//Refine
	// Create a KD-Tree
	pcl::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::KdTreeFLANN<pcl::PointXYZRGB>);
	tree->setInputCloud (cloud_filtered);
	// Output has the same type as the input one, it will be only smoothed
	pcl::PointCloud<pcl::PointXYZRGB> mls_points;
	// Init object (second point type is for the normals, even if unused)
	pcl::MovingLeastSquares<pcl::PointXYZRGB,pcl::Normal> mls;
	// Optionally, a pointer to a cloud can be provided, to be set by MLS
	pcl::PointCloud<pcl::Normal>::Ptr mls_normals (new pcl::PointCloud<pcl::Normal> ());
	mls.setOutputNormals (mls_normals);
	// Set parameters
	mls.setInputCloud (cloud_filtered);
	mls.setPolynomialFit (true);
	mls.setSearchMethod (tree);
	mls.setSearchRadius (10);
	//Reconstruct
	mls.reconstruct (mls_points);
	//Concatenate fields for saving
	pcl::PointCloud<pcl::PointNormal> mls_cloud;
	pcl::concatenateFields (mls_points, *mls_normals, mls_cloud);
	*/
	//Save output
	pcl::io::savePCDFileBinary ("model_good.pcd", *cloud_filtered);
	
	//Visualization
	pcl::visualization::CloudViewer cloud_viewer(argv[1]);
	cloud_viewer.showCloud (cloud_filtered);
	while (!cloud_viewer.wasStopped()){    
	    sleep(100);
	}	    
   return (0);
 }
