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

void get_files::readDRJsonPaths(std::deque<std::string> &DRJsonQue_) {
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
    // std::cout << "dr dr_time_nsec   " << data.dr_time_nsec << std::endl;

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
}  // namespace getdata