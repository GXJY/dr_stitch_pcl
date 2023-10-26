#ifndef PREPROCESS_H
#define PREPROCESS_H

// #include <pcl_conversions/pcl_conversions.h>
// #include <ros/ros.h>
// #include <sensor_msgs/PointCloud2.h>
#include <pcl/io/pcd_io.h>

#include <vector>

using namespace std;  // NOLINT

typedef pcl::PointXYZINormal PointType;
typedef pcl::PointCloud<PointType> PointCloudXYZI;

namespace rs128 {
struct EIGEN_ALIGN16 Point {
  PCL_ADD_POINT4D;
  float intensity;
  double time;  // float
  std::uint16_t ring;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
}  // namespace rs128
POINT_CLOUD_REGISTER_POINT_STRUCT(rs128::Point,
                                  (float, x, x)(float, y, y)(float, z, z)(
                                      float, intensity,
                                      intensity)(std::uint16_t, ring,
                                                 ring)(double, time, timestamp))

#endif