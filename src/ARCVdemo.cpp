//Ogre3d includes
#include "AugmentedCV.h"
#include <OgreException.h>
#include <OgreConfigFile.h>
#include <OgreRenderSystem.h>
#include <OgreCamera.h>
#include <OgreViewport.h>
#include <OgreSceneManager.h>
#include <OgreRenderWindow.h>
//#include <OgreEntity.h>
#include <OgreWindowEventUtilities.h>
#include <OgreMaterial.h>
#include <OgreMaterialManager.h>
#include <OgreOverlayElement.h>
#include <OgreOverlayManager.h>
using namespace xn;
using namespace Ogre;

//-------------------------------------------------------------------------------------
AugmentedApp::AugmentedApp(void): 
	mRoot(0), 
	mPluginsCfg(Ogre::StringUtil::BLANK),
	mResourcesCfg(Ogre::StringUtil::BLANK)
{
}
//-------------------------------------------------------------------------------------
AugmentedApp::~AugmentedApp(void)
{
	//Remove ourself as a Window listener
	WindowEventUtilities::removeWindowEventListener(mWindow, this);
	windowClosed(mWindow);
	delete mRoot;
	context.Shutdown();
}

 
//Unattach OIS before window shutdown (very important under Linux)
void AugmentedApp::windowClosed(Ogre::RenderWindow* rw)
{
    //Only close for window that created OIS (the main window in these demos)
    if( rw == mWindow )
    {
        if( mInputManager )
        {
            mInputManager->destroyInputObject( mKeyboard );
            OIS::InputManager::destroyInputSystem(mInputManager);
            mInputManager = 0;
        }
    }
}

bool AugmentedApp::frameRenderingQueued(const Ogre::FrameEvent& evt)
{
	//Get Window Events
    if(mWindow->isClosed())
        return false;
    //Get Keys
    mKeyboard->capture();
    if(mKeyboard->isKeyDown(OIS::KC_ESCAPE))
        return false;
	//Update Background and Animation
	getFrame();
	if (!haveModel){    
		getTemplate();
		if (goodModel()){
			initPlane();
		}
		
	}
	else {
			do_matching();
			//updatePlane();
		}
	update_background();
	
	return true;
}

bool AugmentedApp::go(void)
{
	//Initilize sensor
	rgbd_init();
#ifdef _DEBUG
    mPluginsCfg = "plugins_d.cfg";
	mResourcesCfg = "resources_d.cfg";
#else
    mPluginsCfg = "plugins.cfg";
	mResourcesCfg = "resources.cfg";
#endif 
    // construct Ogre::Root
    mRoot = new Ogre::Root(mPluginsCfg);
	// setup resources
	// Load resource paths from config file
	Ogre::ConfigFile cf;
	cf.load(mResourcesCfg);
	// Go through all sections & settings in the file
	Ogre::ConfigFile::SectionIterator seci = cf.getSectionIterator();
	Ogre::String secName, typeName, archName;
	while (seci.hasMoreElements())
	{
		secName = seci.peekNextKey();
		Ogre::ConfigFile::SettingsMultiMap *settings = seci.getNext();
		Ogre::ConfigFile::SettingsMultiMap::iterator i;
		for (i = settings->begin(); i != settings->end(); ++i)
		{
			typeName = i->first;
			archName = i->second;
			Ogre::ResourceGroupManager::getSingleton().addResourceLocation(archName, typeName, secName);
		}
	}
	// configure
	RenderSystem *rs = mRoot->getRenderSystemByName("OpenGL Rendering Subsystem");
	mRoot->setRenderSystem(rs);
	rs->setConfigOption("Full Screen", "No");
	rs->setConfigOption("Video Mode", "640 x 480 @ 32-bit colour");

	//create a window
	mWindow = mRoot->initialise(true, "AugmentedApp Render Window");
	// Set default mipmap level (NB some APIs ignore this)
	Ogre::TextureManager::getSingleton().setDefaultNumMipmaps(5);
	// initialise all resource groups
	Ogre::ResourceGroupManager::getSingleton().initialiseAllResourceGroups();
	//creates a SceneManager
	mSceneMgr = mRoot->createSceneManager("DefaultSceneManager");
	// Setup the camera
	mCamera = mSceneMgr->createCamera("Kinect");
	// Position it at Origin
	mCamera->setPosition(Vector3(0,0,0));
	// Look back along -Z
	mCamera->lookAt(Ogre::Vector3(0,0,-2000));
	mCamera->setNearClipDistance(0.1);
	// Create one viewport, entire window
	Ogre::Viewport* vp = mWindow->addViewport(mCamera);
	//vp->setBackgroundColour(Ogre::ColourValue(1,1,1));
	// Alter the camera aspect ratio to match the viewport
	mCamera->setAspectRatio(Ogre::Real(vp->getActualWidth()) / Ogre::Real(vp->getActualHeight()));
	//Background Image
	// Create the texture
	mBackground = TextureManager::getSingleton().createManual(
		"DynamicTexture",
		ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
		TEX_TYPE_2D,      // type
		WIDTH, HEIGHT,         // width & height
		0,                // number of mipmaps
		PF_BYTE_BGRA,     // pixel format
		TU_DYNAMIC_WRITE_ONLY_DISCARDABLE);      // usage; should be TU_DYNAMIC_WRITE_ONLY_DISCARDABLE for
						  // textures updated very often (e.g. each frame)
	// Create a material using the texture
	MaterialPtr material = Ogre::MaterialManager::getSingleton().create("DynamicTextureMaterial",ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME); 
	material->getTechnique(0)->getPass(0)->createTextureUnitState("DynamicTexture");
	material->getTechnique(0)->getPass(0)->setDepthCheckEnabled(false);
	material->getTechnique(0)->getPass(0)->setDepthWriteEnabled(false);
	material->getTechnique(0)->getPass(0)->setLightingEnabled(false);
	//material->getTechnique(0)->getPass(0)->setVertexColourTracking(TVC_AMBIENT);
	// Create background rectangle covering the whole screen
	Rectangle2D* rect = new Ogre::Rectangle2D(true);
	rect->setCorners(-1.0, 1.0, 1.0, -1.0);
	rect->setMaterial("DynamicTextureMaterial"); 
	// Render the background before everything else
	rect->setRenderQueueGroup(RENDER_QUEUE_BACKGROUND);
	// Attach background to the scene
	SceneNode* node = mSceneMgr->getRootSceneNode()->createChildSceneNode("Background",Vector3(0,0,-1000));
	node->attachObject(rect);
	//init variables
	haveModel = false;
	Rot = cvCreateMat(3,3,CV_32FC1);
	Tran = cvCreateMat(3,1,CV_32FC1);		
	
	
	// Set ambient light
	mSceneMgr->setAmbientLight(Ogre::ColourValue(0.5, 0.5, 0.5));
	// Create a light
	Light* l = mSceneMgr->createLight("MainLight");
	l->setPosition(20,80,-50);
	//Input System
	Ogre::LogManager::getSingletonPtr()->logMessage("*** Initializing OIS ***");
	OIS::ParamList pl;
	size_t windowHnd = 0;
	std::ostringstream windowHndStr;
	mWindow->getCustomAttribute("WINDOW", &windowHnd);
	windowHndStr << windowHnd;
	pl.insert(std::make_pair(std::string("WINDOW"), windowHndStr.str())); 
	mInputManager = OIS::InputManager::createInputSystem( pl );
	mKeyboard = static_cast<OIS::Keyboard*>(mInputManager->createInputObject( OIS::OISKeyboard, false ));
	//Register as a Window listener
	WindowEventUtilities::addWindowEventListener(mWindow, this);
	mRoot->addFrameListener(this);	
	//Render Loop
	mRoot->startRendering();
    return true;
}
 
#if OGRE_PLATFORM == OGRE_PLATFORM_WIN32
#define WIN32_LEAN_AND_MEAN
#include "windows.h"
#endif
 
#ifdef __cplusplus
extern "C" {
#endif
 
    int main(int argc, char *argv[])
    {
        AugmentedApp app;
        try {
            app.go();
        } 
		catch( Ogre::Exception& e ) {
            std::cerr << "An exception has occured: " <<
                e.getFullDescription().c_str() << std::endl;

        } 
        return 0;
    }
 
#ifdef __cplusplus
}
#endif