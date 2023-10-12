#ifndef GET_DATA_H_
#define GET_DATA_H_

#include <jsoncpp/json/json.h>
#include <pcl/io/pcd_io.h>

#include <Eigen/Core>
#include <deque>
#include <fstream>
#include <memory>
#include <mutex>

namespace getdata {
// extern std::deque<std::string> PCDQue;
// extern std::deque<std::string> DRJsonQue;

class get_files {
 private:
  std::string DR_path;
  std::string PCD_path;

 public:
  void setfilepath(std::string DR_path_, std::string PCD_path_);
  void readDRJsonPaths(std::deque<std::string> &DRJsonQue_);
  void readPCDPaths(std::deque<std::string> &PCDQue_);

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
};

}  // namespace getdata

#endif