#pragma once
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <string>
#include <Eigen/Dense>
#include <opencv2/core/core.hpp>


typedef pcl::PointCloud<pcl::PointXYZRGB> ColorCloud;
typedef pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr ConstColorCloudPtr;
typedef pcl::PointCloud<pcl::PointXYZRGB>::Ptr ColorCloudPtr;
typedef pcl::PointCloud<pcl::PointXYZ> Cloud;
typedef pcl::PointCloud<pcl::PointXYZ>::ConstPtr ConstCloudPtr;
typedef pcl::PointCloud<pcl::PointXYZ>::Ptr CloudPtr;


typedef Eigen::Matrix<uint8_t,Eigen::Dynamic,1> VectorXb;
typedef Eigen::Matrix<uint8_t,Eigen::Dynamic,Eigen::Dynamic> MatrixXb;

pcl::PointCloud<pcl::PointXYZRGB>::Ptr readPCD(const std::string& pcdfile);

Eigen::MatrixXi xyz2uv(const Eigen::MatrixXf& xyz);

Eigen::MatrixXf toEigenMatrix(ColorCloudPtr);
Eigen::MatrixXf getDepthImage(ColorCloudPtr);
pcl::PointCloud<pcl::PointXYZ>::Ptr toPointCloud(const std::vector< std::vector<float> >&);

MatrixXb toBGR(ColorCloudPtr);
cv::Mat toCVMat(Eigen::MatrixXf);


bool pointIsFinite(const pcl::PointXYZRGB& pt);

ColorCloudPtr extractInds(ColorCloudPtr in, std::vector<int> inds);
CloudPtr stripColor(ColorCloudPtr in);
ColorCloudPtr addRandomColor(CloudPtr);
ColorCloudPtr addRandomColor(ColorCloudPtr);
ColorCloudPtr setColor(ColorCloudPtr in, uint8_t r, uint8_t g, uint8_t b);

inline ColorCloudPtr removeConst(ConstColorCloudPtr cloud) {
  return boost::const_pointer_cast< pcl::PointCloud<pcl::PointXYZRGB> >(cloud);
}

float planeDist(std::vector<float> abcd);

std::vector<int> arange(int lo, int hi, int step=1);
Eigen::Matrix3f  makePerpBasis(const Eigen::Vector3f& zdir);
ColorCloudPtr transformPointCloud1(ColorCloudPtr in, Eigen::Affine3f transform);
inline float sq(float x) {return x*x;}


#include <utility>
#include <algorithm>
typedef std::pair<int, double> argsort_pair;

inline bool argsort_comp(const argsort_pair& left, const argsort_pair& right) {
    return left.second < right.second;
}

template<typename Derived>
Eigen::VectorXi argsort(const Eigen::MatrixBase<Derived> &x) {
  Eigen::VectorXi indices(x.size());
    std::vector<argsort_pair> data(x.size());
    for(int i=0;i<x.size();i++) {
        data[i].first = i;
        data[i].second = x(i);
    }
    std::sort(data.begin(), data.end(), argsort_comp);
    for(int i=0;i<data.size();i++) {
        indices(data[i].first) = i;
    }    
    return indices;
}
