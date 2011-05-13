#include <XnCppWrapper.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class Xn_sensor{
	xn::Context context;
	xn::ImageGenerator Xn_image;
	xn::DepthGenerator Xn_depth;
	xn::ImageMetaData rgb_md;
	xn::DepthMetaData depth_md;
	xn::Recorder Xn_recorder;
	xn::Player Xn_player; 
	XnPoint3D* depthpx;
	int x_res;
	int y_res;
public:
	XnUInt8* rgb;
	Xn_sensor(int width, int height);
	inline int getX(){return x_res;}
	inline int getY(){return y_res;}
	void rgbd_init();
	void record(char* file);
	void play(char* file,bool replay = true );
	inline bool endPlaying(){return xnIsPlayerAtEOF(Xn_player);}
	inline void update(){XnStatus nRetVal = context.WaitAndUpdateAll(); if (nRetVal != XN_STATUS_OK) printf("Failed to update sensor: %s\n", xnGetStatusString(nRetVal));}
	XnUInt8* getImageData();
	void getPCL(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
	inline void shutdown(){ context.Shutdown(); }
};
