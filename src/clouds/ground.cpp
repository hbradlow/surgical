#include "ground.h"
#include "utils_pcl.h"
#include <cmath>
#include <Eigen/Geometry>
#include "plane_finding.h"
#include <iostream>
using namespace std;
typedef pcl::PointXYZRGB PointT;
using namespace Eigen;

Affine3f planeTransform(Vector4f abcd) {
  Vector3f perp = abcd.block(0,0,3,1);
  Affine3f rotation; rotation = makePerpBasis(perp).transpose();

  Translation3f translation(0,0,abcd(3));

  Affine3f transform = translation * rotation;
  return transform;

}

Affine3f findGroundTransform(ColorCloudPtr in) {
  GetPlanes gp(in);
  if (gp.nPlanes == 0) throw std::runtime_error("no planes found");

  VectorXf scores(gp.nPlanes);
  for (int i=0; i < gp.nPlanes; i++) {
    scores(i) = gp.plane_inds[i].size() * fabs(gp.plane_params[i][3]);
  }
  int iBest;
  scores.maxCoeff(&iBest);

  Vector4f coeffs(gp.plane_params[iBest].data());
  if (coeffs(2) > 0) coeffs *= -1;
  Affine3f transform = planeTransform(coeffs);
  cout << "transform" << transform.matrix() << endl;
  return transform;
}
