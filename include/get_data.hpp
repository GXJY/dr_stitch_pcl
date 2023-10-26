#ifndef GET_DATA_H_
#define GET_DATA_H_

#include <jsoncpp/json/json.h>
#include <pcl/io/pcd_io.h>

#include <Eigen/Core>
#include <deque>
#include <fstream>
#include <memory>
#include <mutex>

#include "prepcl.hpp"

typedef pcl::PointCloud<rs128::Point> PointCloudRS128;

namespace getdata {

class get_files {
 private:
  std::string DR_path;
  std::string PCD_path;
  std::string IMU_path;
  std::string WHEEL_path;

 public:
  void setfilepath(std::string DR_path_, std::string PCD_path_);
  void setfilepath(std::string DR_path_, std::string PCD_path_, std::string IMU_path_, std::string WHEEl_path_);
  
  void readDRJsonPaths(std::deque<std::string> &DRJsonQue_);
  void readPCDPaths(std::deque<std::string> &PCDQue_);
  void readIMUPaths(std::deque<std::string> &IMUJsonQue_);
  void readWHEELPaths(std::deque<std::string> &WHEELJsonQue_);

  bool haveReadAllDR;
  bool haveReadAllPCD;
};

class get_DR {
 private:
 public:
  struct PoseData {
    std::string dr_time_sec;
    std::string dr_time_nsec;
    std::string dr_path;
    std::string dr_name;
    Eigen::Vector3d dr_pose;
    Eigen::Vector3d dr_eulerAngle;
    Eigen::Quaterniond dr_quaternion;

    bool pose_used = false;
    bool one_pose_readed = false;
  };

  PoseData readOneDRFromJson(std::deque<std::string> DRJsonQue_);
};

class get_PCD {
 private:
 public:
  struct PcdData {
    std::string pcd_time_sec;
    std::string pcd_time_nsec;
    std::string pcd_path;
    std::string pcd_name;

    bool one_pcd_readed = false;

  };

  PcdData readOnePcd(std::deque<std::string> PCDQue_);
  uint64_t getTimeFromString(std::string ss);
  PointCloudRS128 read_rsCloud(std::deque<std::string> PCDQue_);
};

class get_IMU {
 private:
 public:
  struct ImuData {
    std::string imu_time_sec;
    std::string imu_time_nsec;
    std::string imu_path;
    std::string imu_name;
    double imu_angular_velocity_x;
    double imu_angular_velocity_y;
    double imu_angular_velocity_z;
    double imu_linear_acceleration_x;
    double imu_linear_acceleration_y;
    double imu_linear_acceleration_z;

    bool one_imu_readed = false;

  };

  ImuData readOneImuFromJson(std::deque<std::string> IMUJsonQue_);
};

class get_WHEEL {
 private:
 public:
  struct WheelData {
    std::string wheel_time_sec;
    std::string wheel_time_nsec;
    std::string wheel_path;
    std::string wheel_name;
    double wheel_FLWheelSpeed;
    double wheel_FRWheelSpeed;
    double wheel_RLWheelSpeed;
    double wheel_RRWheelSpeed;

    bool one_wheel_readed = false;

  };

  WheelData readOneWheelFromJson(std::deque<std::string> WHEELJsonQue_);
};

}  // namespace getdata

#endif