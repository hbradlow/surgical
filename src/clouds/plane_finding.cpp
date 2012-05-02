#include "plane_finding.h"
#include <boost/foreach.hpp>
#include <pcl/segmentation/organized_multi_plane_segmentation.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/apps/dominant_plane_segmentation.h>

typedef pcl::PointXYZRGB PointT;
using namespace std;
using namespace Eigen;

GetPlanes::GetPlanes(ColorCloudPtr cloud) {

  /*pcl::IntegralImageNormalEstimation<PointT, pcl::Normal> ne;
  ne.setNormalEstimationMethod (ne.COVARIANCE_MATRIX);
  ne.setMaxDepthChangeFactor (0.03f);
  ne.setNormalSmoothingSize (20.0f);*/

  /*for(int i = 0; i<cloud->points.size(); i++){
	  cout << "Points: " << cloud->points[i] << endl;
  }*/

  pcl::apps::DominantPlaneSegmentation<pcl::PointXYZRGB> dps;
  dps.setInputCloud(cloud);
  vector<ColorCloudPtr> clusters;
  dps.compute_table_plane();
  
}

GetPlanes2::GetPlanes2(ColorCloudPtr cloud) {

  pcl::ModelCoefficients::Ptr coeffs (new pcl::ModelCoefficients ());
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
  pcl::SACSegmentation<pcl::PointXYZRGB> seg;
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setMaxIterations(100);
  seg.setDistanceThreshold(.0025);
  seg.setInputCloud(cloud);
  seg.segment(*inliers,*coeffs); 


  vector<int> inds;
  Vector4f coeffsEigen = Vector4f::Map(coeffs->values.data());
  for (int i=0; i < cloud->size(); i++) {
    Vector4f xyz1 = cloud->points[i].getVector4fMap();
    xyz1(3) = 1;
    if (fabs(xyz1.dot(coeffsEigen)) < .01) inds.push_back(i);
  }

  cout << "number of inliers" << inds.size() << endl;
  plane_inds.push_back(inds);
  plane_params.push_back(coeffs->values);
  nPlanes = 1;
}
