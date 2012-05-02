#pragma once
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include "utils_pcl.h"

class GetPlanes {
 public:
  int nPlanes;
  std::vector< std::vector<float> > plane_params;
  std::vector< std::vector<int> > plane_inds;
  GetPlanes(ColorCloudPtr in);

};

class GetPlanes2 {
 public:
  int nPlanes;
  std::vector< std::vector<float> > plane_params;
  std::vector< std::vector<int> > plane_inds;
  GetPlanes2(ColorCloudPtr in);

};
