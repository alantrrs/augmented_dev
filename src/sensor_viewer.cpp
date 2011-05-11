#include <Xn_sensor.h>
#include <string.h>
#include <opencv2/opencv.hpp>
#define WIDTH	640
#define	HEIGHT	480

 int main(int argc, char *argv[]) {
	
	if (argc > 1 && strcmp(argv[1],"-r") !=0){
		 printf("usage:\n %s -r //path//to//file//filename.oni\n",argv[0]);
		 return 0;
	 }
	Xn_sensor sensor(WIDTH,HEIGHT);
	sensor.rgbd_init();

	if (argc ==3 && (strcmp(argv[1],"-r")==0)){
		sensor.record(argv[2]);
		printf("Recording to %s\n",argv[2]);
	} 
	IplImage* rgb_image = cvCreateImageHeader(cvSize(WIDTH,HEIGHT),8,3);
	while(true){
		sensor.update();
		cvSetData(rgb_image,sensor.getImageData(),rgb_image->widthStep);
		cvCvtColor(rgb_image,rgb_image,CV_RGB2BGR);
		cv::imshow("Sensor Viewer",rgb_image);
		char key = cvWaitKey(30);
		if (key == 'q')
			break;
	}
	sensor.shutdown();
return 0;
}
