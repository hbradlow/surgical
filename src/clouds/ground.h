#pragma once
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include "utils_pcl.h"

Eigen::Affine3f findGroundTransform(ColorCloudPtr); // transform so that ground is plane z=0
