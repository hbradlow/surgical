#include "utils_pcl.h"
#include <pcl/io/pcd_io.h>
#include "my_exceptions.h"

using namespace Eigen;
using namespace pcl;
using namespace std;

pcl::PointCloud<pcl::PointXYZRGB>::Ptr readPCD(const std::string& pcdfile) {
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (pcdfile, *cloud) == -1) {
    throw FileOpenError(pcdfile);
    }
  return cloud;
}

Eigen::MatrixXi xyz2uv(const Eigen::MatrixXf& xyz) {
 // http://www.pcl-users.org/Using-Kinect-with-PCL-How-to-project-a-3D-point-x-y-z-to-the-depth-rgb-image-and-how-to-unproject-a--td3164499.html
  VectorXf x = xyz.col(0);
  VectorXf y = xyz.col(1);
  VectorXf z = xyz.col(2);

  float cx = 320-.5;
  float cy = 240-.5;
  float f = 525;
  VectorXf v = f*(x.array() / z.array()) + cx;
  VectorXf u = f*(y.array() / z.array()) + cy;
  MatrixXi uv(u.rows(),2);
  uv.col(0) = u.cast<int>();
  uv.col(1) = v.cast<int>();
  return uv;
}

Eigen::MatrixXf toEigenMatrix(ColorCloudPtr cloud) {
  return cloud->getMatrixXfMap(3,8,0);
}


MatrixXb toBGR(ColorCloudPtr cloud) {
  MatrixXf bgrFloats = cloud->getMatrixXfMap(1,8,4);
  MatrixXb bgrBytes4 = Map<MatrixXb>(reinterpret_cast<uint8_t*>(bgrFloats.data()), bgrFloats.rows(),4);
  MatrixXb bgrBytes3 = bgrBytes4.block(0,0,bgrBytes4.rows(),3);
  return bgrBytes3;
}

MatrixXf getDepthImage(ColorCloudPtr cloud) {
  MatrixXf xyz = toEigenMatrix(cloud);
  MatrixXf norms = xyz.rowwise().norm();
  norms.resize(cloud->height, cloud->width);
  return norms;
}

cv::Mat toCVMat(Eigen::MatrixXf in) {
  return cv::Mat(in.rows(), in.cols(), CV_32FC1, in.data());
}

pcl::PointCloud<pcl::PointXYZ>::Ptr toPointCloud(const std::vector< std::vector<float> >& in) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr out(new pcl::PointCloud<pcl::PointXYZ>());
  BOOST_FOREACH(vector<float> v, in) out->push_back(PointXYZ(v[0],v[1],v[2]));
  out->width=in.size();
  out->height=1;
  return out;
}

bool pointIsFinite(const pcl::PointXYZRGB& pt) {
  return std::isfinite(pt.x) && std::isfinite(pt.y) && std::isfinite(pt.z);
}

ColorCloudPtr extractInds(ColorCloudPtr in, std::vector<int> inds) {
  ColorCloudPtr out(new ColorCloud());
  out->reserve(inds.size());
  out->width = inds.size();
  out->height = 1;
  out->is_dense = false;

  BOOST_FOREACH(int i, inds) {
    out->push_back(in->points[i]);
  }
  return out;
}

CloudPtr stripColor(ColorCloudPtr in) {
  CloudPtr out(new Cloud());
  BOOST_FOREACH(pcl::PointXYZRGB& pt, in->points) {
    out->push_back(pcl::PointXYZ(pt.x, pt.y, pt.z));
  }
  out->width = in->width;
  out->height = in->height;
  out->is_dense = in->is_dense;
  return out;

}

float randf() {return (float)rand()/(float)RAND_MAX;}

ColorCloudPtr addRandomColor(CloudPtr in) {  
  int r = randf()*256;
  int g = randf()*256;
  int b = randf()*256;
  ColorCloudPtr out(new ColorCloud());
  BOOST_FOREACH(pcl::PointXYZ& pt, in->points) {
    pcl::PointXYZRGB outpt;
    outpt.x = pt.x;
    outpt.y = pt.y;
    outpt.z = pt.z;
    outpt.r = r;
    outpt.g = g;
    outpt.b = b;
    out->push_back(outpt);
  }
  out->width = in->width;
  out->height = in->height;
  out->is_dense = in->is_dense;
  return out;

}

ColorCloudPtr addRandomColor(ColorCloudPtr in) {  
  int r = randf()*256;
  int g = randf()*256;
  int b = randf()*256;
  ColorCloudPtr out(new ColorCloud());
  BOOST_FOREACH(pcl::PointXYZRGB& pt, in->points) {
    pcl::PointXYZRGB outpt;
    outpt.x = pt.x;
    outpt.y = pt.y;
    outpt.z = pt.z;
    outpt.r = r;
    outpt.g = g;
    outpt.b = b;
    out->push_back(outpt);
  }
  out->width = in->width;
  out->height = in->height;
  out->is_dense = in->is_dense;
  return out;

}

ColorCloudPtr setColor(ColorCloudPtr in, uint8_t r, uint8_t g, uint8_t b) {  
  ColorCloudPtr out(new ColorCloud());
  BOOST_FOREACH(pcl::PointXYZRGB& pt, in->points) {
    pcl::PointXYZRGB outpt;
    outpt.x = pt.x;
    outpt.y = pt.y;
    outpt.z = pt.z;
    outpt.r = r;
    outpt.g = g;
    outpt.b = b;
    out->push_back(outpt);
  }
  out->width = in->width;
  out->height = in->height;
  out->is_dense = in->is_dense;
  return out;

}


float planeDist(vector<float> abcd) {
  return abcd[3] / (sq(abcd[0]) + sq(abcd[1]) + sq(abcd[2]));
}

std::vector<int> arange(int lo, int hi, int step) {
  int n = (hi - lo) / step;
  std::vector<int> out(n);
  for (int i=0; i < n; i++) out[i] = lo + step*i;
  return out;
}

Matrix3f  makePerpBasis(const Vector3f& zdir) {
  Vector3f newz = zdir.normalized();
  Matrix3f out;
  if ((newz.x() == 0) && (newz.y() == 0)) {
    return Matrix3f::Identity();
  }
  else {
    Vector3f newx(newz.y(), -newz.x(), 0);
    newx.normalize();
    Vector3f newy = newz.cross(newx);
    out << 
      newx.x(), newy.x(), newz.x(),
      newx.y(), newy.y(), newz.y(),
      newx.z(), newy.z(), newz.z();
    return out;
  }
}

ColorCloudPtr transformPointCloud1(ColorCloudPtr in, Eigen::Affine3f transform) {
  ColorCloudPtr out(new ColorCloud(*in));
  BOOST_FOREACH(PointXYZRGB& p, out->points) {
    p.getVector3fMap() = transform * p.getVector3fMap();
  }
  return out;
}

