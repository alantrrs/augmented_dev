#include <Xn_sensor.h>
#include <pcl/visualization/cloud_viewer.h>
#include <string.h>

#define WIDTH	640
#define	HEIGHT	480

int main (int argc,char* argv[]){
	if (argc != 3 || (strcmp(argv[1],"-play")!=0 && strcmp(argv[1],"-rec")!=0)) {
		printf("usage:\n %s [-play | -rec ] /path/to/recoding/filename.oni\n",argv[0]);
		printf("press e to quit\n");
		return 0;
	}
	Xn_sensor sensor(WIDTH,HEIGHT);
	if (strcmp(argv[1],"-play")==0){
		//Play .oni recording
		sensor.play(argv[2]);
	}
	if (strcmp(argv[1],"-rec")==0){
		//Record to .oni recording
		sensor.rgbd_init();
		sensor.record(argv[2]);
	}
	//init point cloud
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
	//create viewer
	pcl::visualization::CloudViewer viewer("Player");
	//press 'e' to quit
		while(!viewer.wasStopped()){
		//update sensor 
		sensor.update();
		//fill the pointclud
		sensor.getPCL(cloud);
		//show
		viewer.showCloud (cloud);
	}
	printf("Shuting down..\n");
	sensor.shutdown();
	return 0;
}
