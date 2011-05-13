#include "Xn_sensor.h"

//Xn_Sensor : PrimeSense
Xn_sensor::Xn_sensor(int width,int height){
	x_res = width;
	y_res = height;
	//pre-allocate data for speed
	depthpx = (XnPoint3D*)malloc(x_res*y_res*sizeof(XnFloat)*3);
}

void Xn_sensor::rgbd_init(){
	XnStatus nRetVal = XN_STATUS_OK;
	/// Initialize context object
	nRetVal = context.Init();
	if (nRetVal != XN_STATUS_OK){
			printf("Failed to initialize context: %s\n", xnGetStatusString(nRetVal));
		}

	/// Configure output
	XnMapOutputMode outputMode;
	outputMode.nXRes = x_res;
	outputMode.nYRes = y_res;
	outputMode.nFPS = 30;

	/// Create a DepthGenerator node
	nRetVal = Xn_depth.Create(context);
	if (nRetVal != XN_STATUS_OK){
			printf("Failed to create depth generator: %s\n", xnGetStatusString(nRetVal));
		}
	Xn_depth.SetMapOutputMode(outputMode);
	/// Create a ImageGenerator node
	nRetVal = Xn_image.Create(context);
	if (nRetVal != XN_STATUS_OK){
			printf("Failed to create image generator: %s\n", xnGetStatusString(nRetVal));
		}
	Xn_image.SetMapOutputMode(outputMode);
	//Para el PrimeSense
	Xn_depth.SetIntProperty("RegistrationType",1);
	Xn_image.SetIntProperty("InputFormat",1);
	/// Make it start generating data	
	nRetVal = context.StartGeneratingAll();
	if (nRetVal != XN_STATUS_OK){
			printf("Failed generating data: %s\n", xnGetStatusString(nRetVal));
		}
    
	/// Set the view point of the DepthGenerator to match the ImageGenerator
	nRetVal = Xn_depth.GetAlternativeViewPointCap().SetViewPoint(Xn_image);
	if (nRetVal != XN_STATUS_OK){
			printf("Failed to match Depth and RGB points of view: %s\n", xnGetStatusString(nRetVal));
	}
}

void Xn_sensor::record(char* file){
	XnStatus nRetVal = XN_STATUS_OK;
	nRetVal = Xn_recorder.Create(context);
	if (nRetVal != XN_STATUS_OK){
		printf("Failed to initialize recorder: %s\n", xnGetStatusString(nRetVal));
	}
	nRetVal = Xn_recorder.SetDestination(XN_RECORD_MEDIUM_FILE,file);
	if (nRetVal != XN_STATUS_OK){
		printf("Failed to set recording destination: %s\n", xnGetStatusString(nRetVal));
	}
	nRetVal = Xn_recorder.AddNodeToRecording(Xn_depth, XN_CODEC_16Z_EMB_TABLES);
	if (nRetVal != XN_STATUS_OK){
		printf("Failed to add node to recording: %s\n", xnGetStatusString(nRetVal));
	}
	nRetVal = Xn_recorder.AddNodeToRecording(Xn_image, XN_CODEC_JPEG);
	if (nRetVal != XN_STATUS_OK){
		printf("Failed to add node to recording: %s\n", xnGetStatusString(nRetVal));
	}
}

void  Xn_sensor::play(char* file,bool replay ){
	XnStatus nRetVal = XN_STATUS_OK;
	nRetVal = context.Init();
	if (nRetVal != XN_STATUS_OK){
		printf("Failed to set initalize context: %s\n", xnGetStatusString(nRetVal));
	}
	nRetVal = context.OpenFileRecording(file);
	if (nRetVal != XN_STATUS_OK){
		printf("Failed to open recording file: %s\n", xnGetStatusString(nRetVal));
	}
	nRetVal = context.FindExistingNode(XN_NODE_TYPE_DEPTH, Xn_depth);
	if (nRetVal != XN_STATUS_OK){
		printf("Failed to find depth node: %s\n", xnGetStatusString(nRetVal));
	}
	nRetVal = context.FindExistingNode(XN_NODE_TYPE_IMAGE, Xn_image);
	if (nRetVal != XN_STATUS_OK){
		printf("Failed to find image node: %s\n", xnGetStatusString(nRetVal));
	}
	nRetVal = context.FindExistingNode(XN_NODE_TYPE_PLAYER,Xn_player); 
	if (nRetVal != XN_STATUS_OK){
		printf("Failed to find player node: %s\n", xnGetStatusString(nRetVal));
	}
	Xn_player.SetRepeat(replay); 
}

XnUInt8* Xn_sensor::getImageData(){
	Xn_image.GetMetaData(rgb_md);
	rgb = rgb_md.WritableData();
	return rgb;
}

void Xn_sensor::getPCL(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud){
	 // Take current depth map
    const XnDepthPixel* depthmap = Xn_depth.GetDepthMap();
    // Take current rgb data
    Xn_image.GetMetaData(rgb_md);
	rgb = rgb_md.WritableData(); 
	// Fill in the cloud data
	cloud->width  = x_res;
	cloud->height = y_res;
	cloud->points.resize (cloud->width * cloud->height);
	int i=0;
	for (int y=0; y<cloud->height; y++){
		for (int x=0;x<cloud->width; x++){
			depthpx[i].X= x;
			depthpx[i].Y= y;
			depthpx[i].Z= depthmap[i];
			i++;
		}
	}
	Xn_depth.ConvertProjectiveToRealWorld(i,depthpx,depthpx);		
	i=0;
	int32_t color;
	for (int y = 0; y < cloud->height; y++){
		for (int x=0;x<cloud->width; x++){
			cloud->points[i].x = depthpx[i].X;
			cloud->points[i].y = depthpx[i].Y;
			cloud->points[i].z = depthpx[i].Z;
			color =  (rgb[3*i]<<16)|(rgb[3*i+1]<<8)|(rgb[3*i+2]);
			cloud->points[i].rgb = *(float *)(&color);
			i++;
		}
	}
}
