//Ogre3d includes
#include <OgreRoot.h>
#include <OISEvents.h>
#include <OISInputManager.h>
#include <OISKeyboard.h>
#include <OgreWindowEventUtilities.h>
#include <OgreHardwarePixelBuffer.h>
#include <OgrePixelFormat.h>
#include <OgreEntity.h>
#include <OgreAnimation.h>
#include <OgreManualObject.h>
#include <OgreSceneManager.h>
#include <OgreMaterialManager.h>
#include <OgreMeshManager.h>
//OpenCV and BRIEF includes
#include <cv.h>
//OpenNi includes
#include <XnCppWrapper.h>

#include <stdio.h>

//My Library Functions
#include "myfuncs.h"

#define WIDTH	640
#define	HEIGHT	480

using namespace xn;
using namespace cv;
//Global Variables
Context context;
DepthGenerator Xn_depth;
ImageGenerator Xn_image;
XnStatus nRetVal = XN_STATUS_OK;
ImageMetaData rgb_md;
DepthMetaData depth_md;

//FAST
const int DESIRED_FTRS = 500;
GridAdaptedFeatureDetector gridDetector(new FastFeatureDetector,DESIRED_FTRS, 4, 4);
DynamicAdaptedFeatureDetector detector(new FastAdjuster(20,true),80,120,10);
//BRIEF
BriefDescriptorExtractor descriptor(32);
BruteForceMatcher<Hamming> desc_matcher;

class AugmentedApp : public Ogre::WindowEventListener, public Ogre::FrameListener
{
public:
    AugmentedApp(void);
    virtual ~AugmentedApp(void);
    bool go(void);
protected:
	//virtual void windowResized(Ogre::RenderWindow* rw);
	virtual void windowClosed(Ogre::RenderWindow* rw);
	virtual bool frameRenderingQueued(const Ogre::FrameEvent& evt);
private:
	//Augmeted App Members
    Ogre::Root* mRoot;
    Ogre::String mPluginsCfg;
	Ogre::String mResourcesCfg;
	Ogre::RenderWindow* mWindow;
	Ogre::SceneManager* mSceneMgr;
	Ogre::Camera* mCamera;
	Ogre::TexturePtr mBackground;
	Ogre::Entity* mCharacter;
	Ogre::AnimationState* mAnims[13];
	
	//Template members
	XnPoint3D* template_pts;
	int template_inliers;
	float A,B,C,D;
	XnPoint3D template_com;
	XnPoint3D* model_kpts;
	float template_error;
	std::vector<KeyPoint> template_kpts;
	Mat template_feats;
	std::vector<DMatch> template_matches;

	bool haveModel;
	//Frame members
	IplImage* currentFrame;
	IplImage* gsCurrentFrame;
	std::vector<KeyPoint> current_kpts;
	Mat current_feats;
	std::vector<DMatch> current_matches;
	CvMat* Rot;
	CvMat* Tran;

	//Plane members
	Ogre::Plane mPlane;
	Ogre::Vector3 p1;
	Ogre::Vector3 p2;
	Ogre::Vector3 p3;

	//OIS Input devices
	OIS::InputManager* mInputManager;
	OIS::Keyboard* mKeyboard;
	
	//OpenNI Functions
	void rgbd_init(){
	/// Initialize context object
	nRetVal = context.Init();
	if (nRetVal != XN_STATUS_OK){
			printf("Failed to initialize context: %s\n", xnGetStatusString(nRetVal));
		}

	/// Configure output
	XnMapOutputMode outputMode;
	outputMode.nXRes = WIDTH;
	outputMode.nYRes = HEIGHT;
	outputMode.nFPS = 30;

	/// Create a DepthGenerator node
	nRetVal = Xn_depth.Create(context);
	if (nRetVal != XN_STATUS_OK){
			printf("Failed to create depth generator: %s\n", xnGetStatusString(nRetVal));
		}
	Xn_depth.SetMapOutputMode(outputMode);
	//Xn_depth.GetMirrorCap().SetMirror(mirrored);
	/// Create a ImageGenerator node
	nRetVal = Xn_image.Create(context);
	if (nRetVal != XN_STATUS_OK){
			printf("Failed to create image generator: %s\n", xnGetStatusString(nRetVal));
		}
	Xn_image.SetMapOutputMode(outputMode);
	//Para el PrimeSense
	//Xn_depth.SetIntProperty("RegistrationType",1);
	//Xn_image.SetIntProperty("InputFormat",1);
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
	void getFrame(){
		 // Wait for new data to be available
		nRetVal = context.WaitAndUpdateAll();
		Xn_image.GetMetaData(rgb_md);
		currentFrame = cvCreateImageHeader(cvSize(640,480), 8, 3);
		XnUInt8* rgb_data = rgb_md.WritableData();
		//RGB Frame
		cvSetData(currentFrame,rgb_data,currentFrame->widthStep);
		//Gray Scale Frame
		gsCurrentFrame = cvCreateImage(cvGetSize(currentFrame),currentFrame->depth,1);
		cvCvtColor(currentFrame,gsCurrentFrame,CV_RGB2GRAY);
		//TODO: Smooth the image
	}

	void update_background(){
		Ogre::TexturePtr mBackground = Ogre::TextureManager::getSingleton().getByName("DynamicTexture");
		// Get the pixel buffer
		Ogre::HardwarePixelBufferSharedPtr pixelBuffer = mBackground->getBuffer();
		// Lock the pixel buffer and get a pixel box
		pixelBuffer->lock(Ogre::HardwareBuffer::HBL_DISCARD); // for best performance use HBL_DISCARD!	
		const Ogre::PixelBox& pixelBox = pixelBuffer->getCurrentLock();
		char* pDest = static_cast<char*>(pixelBox.data);
		for (int j = 0; j < HEIGHT; j++){
			for (int i=0; i< WIDTH; i++){
				pDest[j*WIDTH*4 + 4*i + 0] = ((uchar*)(currentFrame->imageData + currentFrame->widthStep*j))[i*3+2];//rgb_data[j*WIDTH*3 + 3*fixed_i + 2]; // B
				pDest[j*WIDTH*4 + 4*i + 1] = ((uchar*)(currentFrame->imageData + currentFrame->widthStep*j))[i*3+1];//rgb_data[j*WIDTH*3 + 3*fixed_i + 1]; // G
				pDest[j*WIDTH*4 + 4*i + 2] = ((uchar*)(currentFrame->imageData + currentFrame->widthStep*j))[i*3+0];//rgb_data[j*WIDTH*3 + 3*fixed_i + 0]; // R
				pDest[j*WIDTH*4 + 4*i + 3] = 255;//A
			}
		}
		// Unlock the pixel buffer
		pixelBuffer->unlock();
	}


	int planeFittingRANSAC(XnPoint3D* pts, int num, CvMat* model, XnPoint3D* best_cset){
		//RANSAC parameters
		template_error = FLT_MAX;
		int th = 10;
		int min_inliers = 1000;
		
		float f_support = 0.7*num;// the maximum probable number of points belonging to the same plane.
		float alpha = 0.99;//probability of finding at least one good set of observation in N trials
		int iterations=cvRound(log(1-alpha)/log(1-pow(1-(1-f_support/num),3)));
		
		XnPoint3D* consensus_set = (XnPoint3D*)malloc(num*sizeof(XnFloat)*3);
		int initpts = 10;
		int randompt;
		int inliers=0;
		int binliers=0;
		float msq_error;
		float dist;
		
		//plane parameters
		float d = 1000;
		D = d; 
		float c1, c2, c3;
				
		//initial matrix
		CvMat* bmat = cvCreateMat(initpts,1,CV_32FC1);
		CvMat* plane = cvCreateMat(initpts,3,CV_32FC1);
	
		//coefficients matrix
		CvMat* coef = cvCreateMat(3,1,CV_32FC1);
		
		srand((unsigned)time(0));
		for(int i=0;i<iterations;i++){
			//creates a plane using random points
			for (int pt=0;pt<initpts;pt++){
				randompt = rand()%num;
				CV_MAT_ELEM( *plane, float, pt, 0 ) = pts[randompt].X;
				CV_MAT_ELEM( *plane, float, pt, 1 ) = pts[randompt].Y;
				CV_MAT_ELEM( *plane, float, pt, 2 ) = pts[randompt].Z;
				CV_MAT_ELEM( *bmat, float, pt, 0 ) = -d;
			}
			cvSolve(plane,bmat,coef,cv::DECOMP_SVD);
			c1 = CV_MAT_ELEM(*coef,float,0,0);
			c2 = CV_MAT_ELEM(*coef,float,1,0);
			c3 = CV_MAT_ELEM(*coef,float,2,0);
			
			//add points within a threshold to the consensus_set (inliers)
			inliers=0;
			for (int p=0;p<num;p++){
				dist = abs(c1*pts[p].X + c2*pts[p].Y + c3*pts[p].Z + d)/sqrt(pow(c1,2)+pow(c2,2)+pow(c3,2));
				if (dist<th){
					consensus_set[inliers]=pts[p];
					inliers++;
				}
			}
			//fine adjustment - error minimization	
			if (inliers > min_inliers && inliers < WIDTH*HEIGHT) {
				CvMat* planefit = cvCreateMat(inliers,3,CV_32FC1);
				CvMat* bmat2 = cvCreateMat(inliers,1,CV_32FC1);
				for(int n=0;n<inliers;n++){
					CV_MAT_ELEM( *bmat2, float, n, 0 ) = -d;
					CV_MAT_ELEM( *planefit, float, n, 0 ) = pts[n].X;
					CV_MAT_ELEM( *planefit, float, n, 1 ) = pts[n].Y;
					CV_MAT_ELEM( *planefit, float, n, 2 ) = pts[n].Z;
				}
				cvSolve(planefit,bmat2,coef,cv::DECOMP_SVD);
				c1 = CV_MAT_ELEM(*coef,float,0,0);
				c2 = CV_MAT_ELEM(*coef,float,1,0);
				c3 = CV_MAT_ELEM(*coef,float,2,0);
				msq_error=0;
				for(int n=0;n<inliers;n++){
					msq_error+= pow((c1*pts[n].X + c2*pts[n].Y + c3*pts[n].Z + d),2)/(pow(c1,2)+pow(c2,2)+pow(c3,2));
				}
				//update best fit
				if (msq_error<template_error){
					//recalculate inliers
					inliers=0;
					for (int p=0;p<num;p++){
						dist = abs(c1*pts[p].X + c2*pts[p].Y + c3*pts[p].Z + d)/sqrt(pow(c1,2)+pow(c2,2)+pow(c3,2));
						if (dist<th){
							consensus_set[inliers]=pts[p];
							inliers++;
						}
					}
					binliers=inliers;
					template_error = msq_error;
					model = coef;
					memcpy(best_cset,consensus_set,num*sizeof(XnFloat)*3);//best_cset = consensus_set;
				}
				cvReleaseMat(&planefit);
				cvReleaseMat(&bmat2);
			}
					}
		free(consensus_set);
		return binliers;
	}

	void getTemplate(){
		Xn_depth.GetMetaData(depth_md);
		const XnDepthPixel* depth_data = depth_md.Data();
		XnPoint3D* pts = (XnPoint3D*)malloc(HEIGHT*WIDTH*sizeof(XnFloat)*3);
		//Get the 3d points before the threshold
		int threshold = 1000;
		int ac = 0; //number of points before the threshold
		for (int j = 0; j < HEIGHT; j++){
			for (int i=0; i< WIDTH; i++){
				if (depth_data[j*WIDTH+i]<threshold && depth_data[j*WIDTH+i]>0){
					pts[ac].X=i;
					pts[ac].Y=j;
					pts[ac].Z=depth_data[j*WIDTH+i];
					ac++;
					//highlight points
					((uchar*)(currentFrame->imageData + currentFrame->widthStep*j))[i*3]=255;
					((uchar*)(currentFrame->imageData + currentFrame->widthStep*j))[i*3+1]=255;
					((uchar*)(currentFrame->imageData + currentFrame->widthStep*j))[i*3+2]=0;
				}
			}
		}
		template_inliers=0;
		int minx=WIDTH,miny=HEIGHT,maxx=0,maxy=0;
		float comx=0,comy=0,comz=0;
		
		if (ac>500){
			//Fit plane
			xnConvertProjectiveToRealWorld(Xn_depth,ac,pts,pts);
			template_pts = (XnPoint3D*)malloc(ac*sizeof(XnFloat)*3);
			CvMat* best_model = cvCreateMat(3,1,CV_32FC1);
			template_inliers = planeFittingRANSAC(pts,ac,best_model,template_pts);
			//Get model info
			A = CV_MAT_ELEM(*best_model,float,0,0);
			B = CV_MAT_ELEM(*best_model,float,1,0);
			C = CV_MAT_ELEM(*best_model,float,2,0);
			//Highlight PLANE
			XnPoint3D* best_cset2D = (XnPoint3D*)malloc(template_inliers*sizeof(XnFloat)*3);;
			xnConvertRealWorldToProjective(Xn_depth,template_inliers,template_pts,best_cset2D);
			int x, y;
			for(int i=0; i<template_inliers; i++){
				x = best_cset2D[i].X;
				y = best_cset2D[i].Y;
				((uchar*)(currentFrame->imageData + currentFrame->widthStep*y))[x*3]=0;
				((uchar*)(currentFrame->imageData + currentFrame->widthStep*y))[x*3+1]=255;
				((uchar*)(currentFrame->imageData + currentFrame->widthStep*y))[x*3+2]=0;
				//Get Bounding box coordinates
				if (x<minx) minx=x;
				if (x>maxx) maxx=x;
				if (y<miny) miny=y;
				if (y>maxy) maxy=y;
				//Get center of mass
				comx +=template_pts[i].X;
				comy +=template_pts[i].Y;
				comz +=template_pts[i].Z;
			}
			template_com.X = comx/template_inliers;
			template_com.Y = comy/template_inliers;
			template_com.Z = comz/template_inliers;
		}
		free(pts);
		//Extract Features within the Bounding Box
		CvMat* mask = cvCreateMat(gsCurrentFrame->height,gsCurrentFrame->width,CV_8U);
		cvZero(mask);
		cvRectangle(mask,cvPoint(minx,miny),cvPoint(maxx,maxy),cvScalar(1,0,0),-1);

		if (template_inliers>10000){
			detector.detect(gsCurrentFrame, template_kpts,mask);
			
			//compute descriptors
			if (template_kpts.size()>0){
				
				descriptor.compute(gsCurrentFrame,template_kpts,template_feats);
				model_kpts = (XnPoint3D*)malloc(template_kpts.size()*sizeof(XnFloat)*3);
				for (int k=0;k<template_kpts.size();k++){
					model_kpts[k].X=template_kpts[k].pt.x;
					model_kpts[k].Y=template_kpts[k].pt.y;
					model_kpts[k].Z=depth_data[(int)(template_kpts[k].pt.y*WIDTH+template_kpts[k].pt.x)];
					//show features
					cvLine(currentFrame,cvPoint(template_kpts[k].pt.x-5,template_kpts[k].pt.y),cvPoint(template_kpts[k].pt.x+5,template_kpts[k].pt.y),cvScalar(0,0,255));
					cvLine(currentFrame,cvPoint(template_kpts[k].pt.x,template_kpts[k].pt.y-5),cvPoint(template_kpts[k].pt.x,template_kpts[k].pt.y+5),cvScalar(0,0,255));
				}
				xnConvertProjectiveToRealWorld(Xn_depth,template_kpts.size(),model_kpts,model_kpts);
			}
		}
}

	bool goodModel(){
		if (template_error < 90000 && template_inliers > 10000 && template_kpts.size() > 7 && abs(A)<0.1 && abs(B)<0.1){
			haveModel=true;
			return true;
		}
		else {
			template_kpts.clear();
			//TODO: Clear template features
			return false;
		}
	}

	void initPlane(){
		mPlane.normal=Ogre::Vector3::UNIT_Z;
		mPlane.d=0;
		// create a grid mesh resource
		Ogre::MeshManager::getSingleton().createPlane("grid", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
			mPlane,200, 250, 20, 25, true, 1, 20, 25, Ogre::Vector3::UNIT_Y);
		// create a floor entity, give it a material, and place it at the origin
        Ogre::Entity* grid = mSceneMgr->createEntity("Grid", "grid");
		grid->setMaterialName("Examples/grid");
		Ogre::SceneNode* gridnode = mSceneMgr->getRootSceneNode()->createChildSceneNode("GridNode");
		gridnode->attachObject(grid);
		mSceneMgr->getSceneNode("GridNode")->setPosition(template_com.X,template_com.Y,-template_com.Z);
		setupCharacter();
	}

		void setupCharacter(){
		mCharacter = mSceneMgr->createEntity("Character", "Sinbad.mesh");
		//Ogre::SceneNode* CharacterNode = mSceneMgr->getRootSceneNode()->createChildSceneNode();
		Ogre::SceneNode* charNode = mSceneMgr->getSceneNode("GridNode")->createChildSceneNode("charnode",Ogre::Vector3::ZERO,Ogre::Quaternion(sqrt(0.5),sqrt(0.5),0,0));
		charNode->attachObject(mCharacter);
		charNode->setPosition(0,0,50);
		charNode->scale(10,10,10);
		charNode->setInheritOrientation(true);
	}

	void updatePlane(){
		CvMat* com= cvCreateMat(3,1,CV_32FC1);
		CV_MAT_ELEM(*com,float,0,0)=template_com.X;
		CV_MAT_ELEM(*com,float,1,0)=template_com.Y;
		CV_MAT_ELEM(*com,float,2,0)=template_com.Z;
		cvGEMM(Rot,com,1,Tran,1,com);
		Ogre::Matrix3 Ro = Ogre::Matrix3(
			CV_MAT_ELEM(*Rot,float,0,0),CV_MAT_ELEM(*Rot,float,0,1),CV_MAT_ELEM(*Rot,float,0,2),
			CV_MAT_ELEM(*Rot,float,1,0),CV_MAT_ELEM(*Rot,float,1,1),CV_MAT_ELEM(*Rot,float,1,2),
			-CV_MAT_ELEM(*Rot,float,2,0),-CV_MAT_ELEM(*Rot,float,2,1),-CV_MAT_ELEM(*Rot,float,2,2));
		mSceneMgr->getSceneNode("GridNode")->setOrientation(Ro);
		/*mSceneMgr->getSceneNode("GridNode")->setPosition(template_com.X+CV_MAT_ELEM(*Tran,float,0,0),
														 template_com.Y+CV_MAT_ELEM(*Tran,float,1,0),
														 -template_com.Z-CV_MAT_ELEM(*Tran,float,2,0));
												*/
		mSceneMgr->getSceneNode("GridNode")->setPosition(CV_MAT_ELEM(*com,float,0,0),
														 CV_MAT_ELEM(*com,float,1,0),
														 -CV_MAT_ELEM(*com,float,2,0));
	
			
	}


	void xnpts2cvpts(XnPoint3D* xnpoints, CvMat* mat){
		for(int row=0; row<mat->rows; row++ ) {
			float* ptr = (float*)(mat->data.ptr + row * mat->step);
			ptr[0] = xnpoints[row].X;
			ptr[1] = xnpoints[row].Y;
			ptr[2] = xnpoints[row].Z;

		}
	}

void matches2points3D(XnPoint3D* train, const std::vector<KeyPoint>& query,
                    const std::vector<cv::DMatch>& matches, XnPoint3D* pts_train,XnPoint3D* pts_query){
	const XnDepthPixel* depth_data = depth_md.Data();
	size_t i = 0;
  for (; i < matches.size(); i++){
	  const DMatch & dmatch = matches[i];
	  pts_query[i].X = query[dmatch.trainIdx].pt.x;
	  pts_query[i].Y = query[dmatch.trainIdx].pt.y;
	  pts_query[i].Z = depth_data[(int)(pts_query[i].Y*WIDTH+pts_query[i].X)];
	  pts_train[i]=train[dmatch.queryIdx];
  }
  xnConvertProjectiveToRealWorld(Xn_depth,matches.size(),pts_query,pts_query);
}

// Drwa Matches
void drawMatchesRelative(const std::vector<KeyPoint>& train, const std::vector<KeyPoint>& query,
                         std::vector<cv::DMatch>& matches, IplImage* img){
  for (int i = 0; i < (int)matches.size(); i++)
  {
      Point2f pt_new = query[matches[i].queryIdx].pt;
      Point2f pt_old = train[matches[i].trainIdx].pt;
      //cvLine(img, pt_new, pt_old, Scalar(125, 255, 125), 1);
     cvCircle(img, pt_old, 2, Scalar(255, 0, 125), 1);//rosa
	 cvCircle(img, pt_new, 2, Scalar(255, 255, 0), 1);//amarillo
  }
}

void drawInliers(const std::vector<KeyPoint>& train, const std::vector<KeyPoint>& query,
                         std::vector<cv::DMatch>& matches,int* inliers,int num, IplImage* img){
	for (int i = 0; i < num; i++)
  {
      Point2f pt_new = query[matches[inliers[i]].queryIdx].pt;
      Point2f pt_old = train[matches[inliers[i]].trainIdx].pt;
      cvLine(img, pt_new, pt_old, Scalar(0, 255, 0), 1);
      cvCircle(img, pt_old, 2, Scalar(250, 125, 125), 1);//fiusha jaja
	  cvCircle(img, pt_new, 2, Scalar(0, 255, 125), 1);//verde
  }
}

	void do_matching(){
		//New Features Extraction
		gridDetector.detect(gsCurrentFrame,current_kpts);
		//New Features Description
		descriptor.compute(gsCurrentFrame,current_kpts,current_feats);
		
		if (current_kpts.size()>3){
			//Matching
			std::vector<Point2f> m_pts, c_pts;
			//desc_matcher.match(current_feats,template_feats,current_matches);
			desc_matcher.match(template_feats,current_feats,current_matches);
			//Draw matches
			//drawMatchesRelative(template_kpts,current_kpts,current_matches,currentFrame);
			drawMatchesRelative(current_kpts,template_kpts,current_matches,currentFrame);
			
			//Convert matches to 3D points
			XnPoint3D* model_mpts = (XnPoint3D*)malloc((int)current_matches.size()*sizeof(XnFloat)*3);
			XnPoint3D* current_mpts = (XnPoint3D*)malloc((int)current_matches.size()*sizeof(XnFloat)*3);
			matches2points3D(model_kpts,current_kpts,current_matches,model_mpts,current_mpts);
	
			//Find transformation
			int *consensus_set= new int[current_matches.size()];
			int in;
			CvMat* mpts = cvCreateMat(current_matches.size(),3,CV_32FC1);
			CvMat* cpts = cvCreateMat(current_matches.size(),3,CV_32FC1);
			xnpts2cvpts(model_mpts,mpts);
			xnpts2cvpts(current_mpts,cpts);

			float error;
			in = findTransformationRANSAC(mpts,cpts,consensus_set,Rot,Tran,error);
			//ICP(mpts,cpts,Rot,Tran);
			//Apply transformation
			if (in>3){
				drawInliers(current_kpts,template_kpts,current_matches,consensus_set,in,currentFrame);
				updatePlane();
			}
			cvReleaseMat(&mpts);
			cvReleaseMat(&cpts);

			free(current_mpts);
			free(model_mpts);
			current_matches.clear();
		}
		current_kpts.clear();
		//clear features
	}

};