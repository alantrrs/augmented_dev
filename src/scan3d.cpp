#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/common/time.h>
#include <pcl/visualization/cloud_viewer.h>
class SimpleOpenNIProcessor
{
  public:
	SimpleOpenNIProcessor():viewer("PCL Viewer"){}
    void cloud_cb_ (const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud)
    {
	if (!viewer.wasStopped())
         viewer.showCloud (cloud);
      static unsigned count = 0;
      static double last = pcl::getTime ();
      if (++count == 30)
      {
        double now = pcl::getTime ();
        std::cout << "distance of center pixel :" << cloud->points [(cloud->width >> 1) * (cloud->height + 1)].z << " mm. Average framerate: " << double(count)/double(now - last) << " Hz" <<  std::endl;
        count = 0;
        last = now;
      }
    }

    void run ()
    {
      // create a new grabber for OpenNI devices
      pcl::Grabber* interface = new pcl::OpenNIGrabber();

      // make callback function from member function
      boost::function<void (const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr&)> f =
        boost::bind (&SimpleOpenNIProcessor::cloud_cb_, this, _1);

      // connect callback function for desired signal. In this case its a point cloud with color values
      boost::signals2::connection c = interface->registerCallback (f);

      // start receiving point clouds
      interface->start ();

      // wait until user quits program with Ctrl-C, but no busy-waiting -> sleep (1);
      while (!viewer.wasStopped())
        sleep(1);

      // stop the grabber
      interface->stop ();
    }
    pcl::visualization::CloudViewer viewer;
};

int main ()
{
  SimpleOpenNIProcessor v;
  v.run ();
  return 0;
}
