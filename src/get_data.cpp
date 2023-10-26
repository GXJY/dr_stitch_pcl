#include "get_data.hpp"

#include <glog/logging.h>
#include <jsoncpp/json/json.h>

#include <experimental/filesystem>

namespace getdata {

void get_files::setfilepath(std::string DR_path_, std::string PCD_path_) {
  DR_path = DR_path_;
  PCD_path = PCD_path_;
  std::cout << "DR_path    " << DR_path << std::endl;
  std::cout << "PCD_path   " << PCD_path << std::endl;
}

void get_files::setfilepath(std::string DR_path_, std::string PCD_path_,
                            std::string IMU_path_, std::string WHEEL_path_) {
  DR_path = DR_path_;
  PCD_path = PCD_path_;
  IMU_path = IMU_path_;
  WHEEL_path = WHEEL_path_;
  std::cout << "DR_path    " << DR_path << std::endl;
  std::cout << "PCD_path   " << PCD_path << std::endl;
  std::cout << "IMU_path    " << IMU_path << std::endl;
  std::cout << "WHEEL_path   " << WHEEL_path << std::endl;
}

void get_files::readDRJsonPaths(std::deque<std::string> &DRJsonQue_) {
  std::cout << "start read dr from json" << std::endl;
  boost::filesystem::directory_iterator dr_end_itr;
  DRJsonQue_.clear();
  for (boost::filesystem::directory_iterator dr_iter(DR_path);
       dr_iter != dr_end_itr; ++dr_iter) {
    if (boost::filesystem::is_regular_file(dr_iter->path())) {
      std::string dr_fullpath =
          dr_iter->path().string();  //.filename().string();
      // std::cout << "dr_fullpath    " << dr_fullpath << std::endl;
      DRJsonQue_.push_back(dr_fullpath);
    }
  }
  std::sort(DRJsonQue_.begin(), DRJsonQue_.end());
  std::cout << "DRJsonQue_size    " << DRJsonQue_.size() << std::endl;
}

void get_files::readPCDPaths(std::deque<std::string> &PCDQue_) {
  boost::filesystem::directory_iterator pcd_end_itr;
  PCDQue_.clear();
  for (boost::filesystem::directory_iterator pcd_iter(PCD_path);
       pcd_iter != pcd_end_itr; ++pcd_iter) {
    if (boost::filesystem::is_regular_file(pcd_iter->path())) {
      std::string pcd_fullpath =
          pcd_iter->path().string();  //.filename().string();
      // std::cout << "pcd_fullpath   " << pcd_fullpath << std::endl;
      PCDQue_.push_back(pcd_fullpath);
    }
  }
  std::sort(PCDQue_.begin(), PCDQue_.end());
  std::cout << "PCDQue_size    " << PCDQue_.size() << std::endl;
}

void get_files::readIMUPaths(std::deque<std::string> &IMUJsonQue_) {
  boost::filesystem::directory_iterator imu_end_itr;
  IMUJsonQue_.clear();
  for (boost::filesystem::directory_iterator imu_iter(IMU_path);
       imu_iter != imu_end_itr; ++imu_iter) {
    if (boost::filesystem::is_regular_file(imu_iter->path())) {
      std::string imu_fullpath =
          imu_iter->path().string();  //.filename().string();
      // std::cout << "imu_fullpath   " << imu_fullpath << std::endl;
      IMUJsonQue_.push_back(imu_fullpath);
    }
  }
  std::sort(IMUJsonQue_.begin(), IMUJsonQue_.end());
  std::cout << "IMUQue_size    " << IMUJsonQue_.size() << std::endl;
}

void get_files::readWHEELPaths(std::deque<std::string> &WHEELJsonQue_) {
  boost::filesystem::directory_iterator wheel_end_itr;
  WHEELJsonQue_.clear();
  for (boost::filesystem::directory_iterator wheel_iter(IMU_path);
       wheel_iter != wheel_end_itr; ++wheel_iter) {
    if (boost::filesystem::is_regular_file(wheel_iter->path())) {
      std::string wheel_fullpath =
          wheel_iter->path().string();  //.filename().string();
      // std::cout << "wheel_fullpath   " << wheel_fullpath << std::endl;
      WHEELJsonQue_.push_back(wheel_fullpath);
    }
  }
  std::sort(WHEELJsonQue_.begin(), WHEELJsonQue_.end());
  std::cout << "WHEELQue_size    " << WHEELJsonQue_.size() << std::endl;
}

getdata::get_DR::PoseData get_DR::readOneDRFromJson(
    std::deque<std::string> DRJsonQue_) {
  // std::cout << "strat read one dr data" << std::endl;
  PoseData data;
  Json::Reader reader;
  Json::Value root;
  data.dr_path = DRJsonQue_.front();
  boost::filesystem::path drpath(data.dr_path);
  data.dr_name = drpath.filename().string();
  // std::cout << "dr_path   " << data.dr_path << std::endl;
  std::ifstream in(data.dr_path, std::ios::binary);
  if (!in.is_open()) {
    std::cout << "Error opening DR json file\n";
    return data;
  }

  if (reader.parse(in, root)) {
    data.dr_time_sec = root["header"]["stamp"]["sec"].asString();
    data.dr_time_nsec = root["header"]["stamp"]["nsec"].asString();
    if (data.dr_time_nsec.size() < 9) {
      data.dr_time_nsec = "0" + data.dr_time_nsec;
    }
    // std::cout << "dr dr_time_sec   " << data.dr_time_sec << std::endl;
    // std::cout << "dr dr_time_nsec   " << dr_time_nsec << std::endl;

    data.dr_pose =
        Eigen::Vector3d(root["pose"]["poseDR"]["position"]["x"].asDouble(),
                        root["pose"]["poseDR"]["position"]["y"].asDouble(),
                        root["pose"]["poseDR"]["position"]["z"].asDouble());
    // std::cout << "dr_pose: (" << data.dr_pose[0] << ", " << data.dr_pose[1]
    // << ", " << data.dr_pose[2] << ")" << std::endl;

    data.dr_quaternion = Eigen::Quaterniond(
        root["pose"]["poseDR"]["quaternion"]["w"].asDouble(),
        root["pose"]["poseDR"]["quaternion"]["x"].asDouble(),
        root["pose"]["poseDR"]["quaternion"]["y"].asDouble(),
        root["pose"]["poseDR"]["quaternion"]["z"].asDouble());
    // std::cout << "Quaternion: (" << data.dr_quaternion.w() << ", " <<
    // data.dr_quaternion.x() << ", " << data.dr_quaternion.y() << ", " <<
    // data.dr_quaternion.z() << ")" << std::endl;

    data.dr_eulerAngle =
        Eigen::Vector3d(root["pose"]["poseDR"]["eulerAngle"]["x"].asDouble(),
                        root["pose"]["poseDR"]["eulerAngle"]["y"].asDouble(),
                        root["pose"]["poseDR"]["eulerAngle"]["z"].asDouble());
    // std::cout << "dr_eulerAngle: (" << std::fixed << std::setprecision(15) <<
    // data.dr_eulerAngle[0] << ", " << data.dr_eulerAngle[1] << ", " <<
    // data.dr_eulerAngle[2] << ")" << std::endl;

    // std::cout << "dr pose readed" << std::endl;
    data.one_pose_readed = true;
    return data;
  } else {
    std::cout << "DR file failed" << std::endl;
  }
  in.close();

  return data;
}

getdata::get_PCD::PcdData get_PCD::readOnePcd(std::deque<std::string> PCDQue_) {
  PcdData data;
  data.pcd_path = PCDQue_.front();
  // std::cout << "pcd_path   " << data.pcd_path << std::endl;

  boost::filesystem::path pcdpath(data.pcd_path);
  data.pcd_name = pcdpath.filename().string();
  // std::cout << "pcd_name   " << data.pcd_name << std::endl;

  // PCD_name example: RS128_1695780620_244451072.pcd
  std::string delimiter = "_";
  std::string dotttt = ".";
  size_t pos = 0;
  std::string token;
  int count = 0;
  std::string name = data.pcd_name;
  while ((pos = name.find(delimiter)) !=
         std::string::npos) {  // 先根据"_"查找到"RS128"和sec_time
    token = name.substr(0, pos);
    name.erase(0, pos + delimiter.length());
    // std::cout << "Extracted Number " << count << ": " << token << std::endl;
    count++;
    if (count == 2) {
      data.pcd_time_sec = token;
      data.pcd_name = token;
      // std::cout << "pcd_name:    " << data.pcd_name << std::endl;
      while ((pos = name.find(dotttt)) !=
             std::string::npos) {  // 先根据"."查找到nsec_time
        token = name.substr(0, pos);
        name.erase(0, pos + dotttt.length());
        // std::cout << "Extracted Number " << count << ": " << token <<
        // std::endl;
        data.pcd_time_nsec = token;
        data.pcd_name += ".";
        data.pcd_name += token;
        data.pcd_name += ".pcd";
        // std::cout << "pcd_name:    " << data.pcd_name << std::endl;
      }
    }
  }
  // std::cout << "pcd_time_sec       " <<  data.pcd_time_sec << std::endl;
  // std::cout << "pcd_time_nsec      " <<  data.pcd_time_nsec << std::endl;

  data.one_pcd_readed = true;
  return data;
}

uint64_t getdata::get_PCD::getTimeFromString(std::string ss) {
  std::string file_name = ss;
  // Get the index of the last occurrence of "/"
  size_t last_slash_idx = file_name.rfind("/");
  // Extract the file name without extension
  std::string file_id;
  if (last_slash_idx == std::string::npos) {
    file_id = file_name.substr(0, file_name.rfind("."));
  } else {
    file_id = file_name.substr(last_slash_idx + 1,
                               file_name.rfind(".") - last_slash_idx - 1);
  }
  double time = std::stod(file_id);
  if (time < 1e11) {
    time *= 1e9;
    file_id = std::to_string(time);
  }

  return std::stoll(file_id);
}
PointCloudRS128 getdata::get_PCD::read_rsCloud(std::deque<std::string> PCDQue_){

  PointCloudRS128 cloud;
  if (PCDQue_.size() == 0) {
    // haveReadAllLidar = true;
    return cloud;
  }
  std::string path = PCDQue_.front();
  pcl::PCDReader reader;
  reader.read(path, cloud);
  cloud.header.stamp = getTimeFromString(path);
  PCDQue_.pop_front();
  return cloud;
}

getdata::get_IMU::ImuData get_IMU::readOneImuFromJson(
    std::deque<std::string> IMUJsonQue_) {
  // std::cout << "strat read one dr data" << std::endl;
  ImuData data;
  Json::Reader reader;
  Json::Value root;
  data.imu_path = IMUJsonQue_.front();
  boost::filesystem::path imupath(data.imu_path);
  data.imu_name = imupath.filename().string();
  // std::cout << "dr_path   " << data.dr_path << std::endl;
  std::ifstream in(data.imu_path, std::ios::binary);
  if (!in.is_open()) {
    std::cout << "Error opening IMU json file\n";
    return data;
  }

  if (reader.parse(in, root)) {
    data.imu_time_sec = root["header"]["stamp"]["sec"].asString();
    data.imu_time_nsec = root["header"]["stamp"]["nsec"].asString();
    if (data.imu_time_nsec.size() < 9) {
      data.imu_time_nsec = "0" + data.imu_time_nsec;
    }
    // std::cout << "dr dr_time_sec   " << data.dr_time_sec << std::endl;
    // std::cout << "dr dr_time_nsec   " << dr_time_nsec << std::endl;

    data.imu_angular_velocity_x =
        root["imu_info"]["angularVelocity"]["x"].asFloat() * M_PI / 180.0;
    data.imu_angular_velocity_y =
        root["imu_info"]["angularVelocity"]["y"].asFloat() * M_PI / 180.0;
    data.imu_angular_velocity_z =
        root["imu_info"]["angularVelocity"]["z"].asFloat() * M_PI / 180.0;

    data.imu_linear_acceleration_x =
        root["imu_info"]["acceleration"]["x"].asFloat() * 9.81;
    data.imu_linear_acceleration_y =
        root["imu_info"]["acceleration"]["y"].asFloat() * 9.81;
    data.imu_linear_acceleration_z =
        root["imu_info"]["acceleration"]["z"].asFloat() * 9.81;

    data.one_imu_readed = true;
    return data;
  } else {
    std::cout << "IMU file failed" << std::endl;
  }
  in.close();

  return data;
}

getdata::get_WHEEL::WheelData get_WHEEL::readOneWheelFromJson(
    std::deque<std::string> WHEELJsonQue_) {
  // std::cout << "strat read one dr data" << std::endl;
  WheelData data;
  Json::Reader reader;
  Json::Value root;
  data.wheel_path = WHEELJsonQue_.front();
  boost::filesystem::path wheelpath(data.wheel_path);
  data.wheel_name = wheelpath.filename().string();
  // std::cout << "dr_path   " << data.dr_path << std::endl;
  std::ifstream in(data.wheel_path, std::ios::binary);
  if (!in.is_open()) {
    std::cout << "Error opening WHEEL json file\n";
    return data;
  }

  if (reader.parse(in, root)) {
    data.wheel_time_sec = root["header"]["stamp"]["sec"].asString();
    data.wheel_time_nsec = root["header"]["stamp"]["nsec"].asString();
    if (data.wheel_time_nsec.size() < 9) {
      data.wheel_time_nsec = "0" + data.wheel_time_nsec;
    }
    // std::cout << "dr dr_time_sec   " << data.dr_time_sec << std::endl;
    // std::cout << "dr dr_time_nsec   " << dr_time_nsec << std::endl;

    data.wheel_FLWheelSpeed =
        root["wheel_info"]["ESC_FLWheelSpeed"].asDouble();
    data.wheel_FRWheelSpeed =
        root["wheel_info"]["ESC_FRWheelSpeed"].asDouble();
    data.wheel_RLWheelSpeed =
        root["wheel_info"]["ESC_RLWheelSpeed"].asDouble();
    data.wheel_RRWheelSpeed =
        root["wheel_info"]["ESC_RRWheelSpeed"].asDouble();
    

    data.one_wheel_readed = true;
    return data;
  } else {
    std::cout << "WHEEL file failed" << std::endl;
  }
  in.close();

  return data;
}

}  // namespace getdata