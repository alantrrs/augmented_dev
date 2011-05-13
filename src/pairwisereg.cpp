/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id$
 *
 */

/* \author Radu Bogdan Rusu */

#include <boost/make_shared.hpp>
//#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/point_representation.h>

#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>

#include "pcl/visualization/pcl_visualizer.h"

using namespace std;
using pcl::visualization::PointCloudColorHandlerGenericField;
using pcl::visualization::PointCloudColorHandlerCustom;

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointNormal PointNormalT;
typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals; 

// This is a tutorial so we can afford having global variables  
pcl::visualization::PCLVisualizer *p;
int vp_1, vp_2;

struct PCD
{
  PointCloud cloud;
  std::string f_name;
};

struct PCDComparator
{
  bool operator () (const PCD& p1, const PCD& p2)
  {
    return (p1.f_name < p2.f_name);
  }
};

// Define a new point representation for < x, y, z, curvature >
class MyPointRepresentation : public pcl::PointRepresentation <PointNormalT>
{
  using pcl::PointRepresentation<PointNormalT>::nr_dimensions_;
public:
  MyPointRepresentation ()
  {
    // Define the number of dimensions
    nr_dimensions_ = 4;
  }

  // Override the copyToFloatArray method to define our feature vector
  virtual void copyToFloatArray (const PointNormalT &p, float * out) const
  {
    // < x, y, z, curvature >
    out[0] = p.x;
    out[1] = p.y;
    out[2] = p.z;
    out[3] = p.curvature;
  }
};

////////////////////////////////////////////////////////////////////////////////
/** \brief Load a set of PCD files that we want to register together
  * \param argc the number of arguments (pass from main ())
  * \param argv the actual command line arguments (pass from main ())
  * \param models the resultant vector of point cloud datasets
  */
void
  loadData (int argc, char **argv, std::vector<PCD> &models)
{
  std::string extension (".pcd");
  // Suppose the first argument is the actual test model
  for (int i = 1; i < argc; i++)
  {
    string fname = string (argv[i]);
    // Needs to be at least 5: .plot
    if (fname.size () <= extension.size ())
      continue;
 
    transform (fname.begin (), fname.end (), fname.begin (), (int(*)(int))tolower);
    if (fname.compare (fname.size () - extension.size (), extension.size (), extension) == 0)
    {
      // Load the histogram model and saves it into the global list of models
      PCD m;
      m.f_name = argv[i];
      pcl::io::loadPCDFile (argv[i], m.cloud);
      models.push_back (m);
    }
  }

  //sort (models.begin (), models.end (), PCDComparator ());
}

////////////////////////////////////////////////////////////////////////////////
/** \brief Align a pair of PointCloud datasets and return the result
  * \param cloud_src the source PointCloud
  * \param cloud_tgt the target PointCloud
  * \param output the resultant aligned source PointCloud
  */
void
  pairAlign (const PointCloud &cloud_src, const PointCloud &cloud_tgt, PointCloud &output, bool downsample = false)
{
  //
  // Downsample for consistency and speed
  // \note enable this for large datasets
  PointCloud src, tgt;
  pcl::VoxelGrid<PointT> grid;
  if (downsample)
  {
    grid.setLeafSize (0.001, 0.001, 0.001);
    grid.setFilterFieldName ("z");            // Assuming that "z" means distance from the sensor
    grid.setFilterLimits (-1, 5.0);
  
    grid.setInputCloud (boost::make_shared<const PointCloud> (cloud_src));
    grid.filter (src);

    grid.setInputCloud (boost::make_shared<const PointCloud> (cloud_tgt));
    grid.filter (tgt);
  }
  else
  {
    src = cloud_src;
    tgt = cloud_tgt;
  }

  //
  // Compute surface normals and curvature
  PointCloudWithNormals points_with_normals_src, points_with_normals_tgt;

  pcl::NormalEstimation<PointT, PointNormalT> norm_est;
  norm_est.setSearchMethod (boost::make_shared<pcl::KdTreeFLANN<PointT> > ());
  norm_est.setKSearch (30);
  
  norm_est.setInputCloud (boost::make_shared<const PointCloud> (src));
  norm_est.compute (points_with_normals_src);
  pcl::copyPointCloud (src, points_with_normals_src);

  norm_est.setInputCloud (boost::make_shared<const PointCloud> (tgt));
  norm_est.compute (points_with_normals_tgt);
  pcl::copyPointCloud (tgt, points_with_normals_tgt);

  //
  // Instantiate our custom point representation (defined above) ...
  MyPointRepresentation point_representation;
  // ... and weight the 'curvature' dimension so that it is balanced against x, y, and z
  float alpha[4] = {1.0, 1.0, 1.0, 1.0};
  point_representation.setRescaleValues (alpha); 

  //
  // Align
  //pcl::IterativeClosestPoint<PointNormalT, PointNormalT> reg;
  pcl::IterativeClosestPointNonLinear<PointNormalT, PointNormalT> reg;
  reg.setTransformationEpsilon (1e-6);
  // Set the maximum distance between two correspondences (src<->tgt) to 10cm
  // Note: adjust this based on the size of your datasets
  reg.setMaxCorrespondenceDistance (0.1);
  // Set the point representation
  reg.setPointRepresentation (boost::make_shared<const MyPointRepresentation> (point_representation));

  reg.setInputCloud (boost::make_shared<const PointCloudWithNormals> (points_with_normals_src));
  reg.setInputTarget (boost::make_shared<const PointCloudWithNormals> (points_with_normals_tgt));

  PointCloudWithNormals unused;
  //reg.align (unused); -- running the registration in a loop to visualize intermediate results

    
  PointCloudColorHandlerGenericField<PointNormalT> tgt_color_handler (points_with_normals_tgt, "curvature");//0, 255, 0);
  if (!tgt_color_handler.isCapable ())
    printf ("Cannot create curvature color handler!");
  p->addPointCloud (points_with_normals_tgt, tgt_color_handler, "target", vp_2);
  PointCloudColorHandlerGenericField<PointNormalT> src_color_handler (points_with_normals_src, "curvature");//255, 0, 0);

  //
  // Run the same optimization in a loop and visualize the results
  Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity (), prev;
  PointCloudWithNormals reg_result = points_with_normals_src;
  reg.setMaximumIterations (2);
  for (int i = 0; i < 300; ++i)
  {
    // Visualize
    points_with_normals_src = reg_result;
    p->addPointCloud (points_with_normals_src, src_color_handler, "source", vp_2);

    // Estimate
    reg.setInputCloud (boost::make_shared<const PointCloudWithNormals> (points_with_normals_src));
    reg.align (reg_result);
    //if (!reg.hasConverged ())
    //  reg.setMaxCorrespondenceDistance (reg.getMaxCorrespondenceDistance () + 0.02);

    Ti = reg.getFinalTransformation () * Ti;

/*    // Break
    if (fabs ((reg.getLastIncrementalTransformation () - prev).sum ()) < reg.getTransformationEpsilon ())
    {
      p->removePointCloud ("source"); 
      break;
    }*/

    if (fabs ((reg.getLastIncrementalTransformation () - prev).sum ()) < reg.getTransformationEpsilon ())
      reg.setMaxCorrespondenceDistance (reg.getMaxCorrespondenceDistance () - 0.001);
    
    prev = reg.getLastIncrementalTransformation ();
    p->spinOnce (); p->removePointCloud ("source"); 
  }
  p->removePointCloud ("target");

  //
  // Get transformation that aligns source to target
  //Eigen::Matrix4f T = reg.getFinalTransformation ();

  //
  // Apply transformation to source and concatenate with target
  pcl::transformPointCloud (cloud_src, output, Ti);

  PointCloudColorHandlerCustom<PointT> cloud_tgt_h (cloud_tgt, 0, 255, 0);
  PointCloudColorHandlerCustom<PointT> cloud_src_h (output, 255, 0, 0);
  p->addPointCloud (cloud_tgt, cloud_tgt_h, "target", vp_2); p->addPointCloud (output, cloud_src_h, "source", vp_2); p->spin (); p->removePointCloud ("source"); p->removePointCloud ("target");

  output += cloud_tgt;
 }

int main (int argc, char** argv)
{
  vector<PCD> data;
  loadData (argc, argv, data);
  if (data.empty ())
  {
    printf ("Syntax is: %s <source.pcd> <target.pcd> [*]", argv[0]);
    printf ("[*] - multiple files can be added. The registration results of (i, i+1) will be registered against (i+2), etc");
    printf ("Example: %s `rospack find pcl`/test/bun0.pcd `rospack find pcl`/test/bun4.pcd", argv[0]);
    return (-1);
  }
	printf("Loaded %d datasets.", (int)data.size ());


  // Create a PCLVisualizer object
  p = new pcl_visualization::PCLVisualizer (argc, argv, "Pairwise Incremental Registration example");
  p->createViewPort (0.0, 0, 0.5, 1.0, vp_1);
  p->createViewPort (0.5, 0, 1.0, 1.0, vp_2);

  PointCloud result = data[0].cloud;
  for (size_t i = 1; i < data.size (); ++i)
  {
    // Add visualization data
    PointCloudColorHandlerCustom<PointT> tgt_h (data[i].cloud, 0, 255, 0); PointCloudColorHandlerCustom<PointT> src_h (result, 255, 0, 0);
    p->addPointCloud (data[i].cloud, tgt_h, "vp1_target", vp_1); p->addPointCloud (result, src_h, "vp1_source", vp_1);
    printf ("Press q to begin the registration.");
    p->spin ();

    PointCloud temp;
    printf ("Aligning %s (%d) with %s (%d).", data[i-1].f_name.c_str (), (int)result.points.size (), data[i].f_name.c_str (), (int)data[i].cloud.points.size ());
    pairAlign (result, data[i].cloud, temp);
    result = temp;

    std::stringstream ss;
    ss << i << ".pcd";
    pcl::io::savePCDFile (ss.str (), result, true);

    // Remove visualization data
    p->removePointCloud ("vp1_target"); p->removePointCloud ("vp1_source");
  }
}

