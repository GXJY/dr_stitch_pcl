#include <jsoncpp/json/json.h>
#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <algorithm>
#include <chrono>
#include <ctime>
#include <deque>
#include <fstream>
#include <iostream>
#include <memory>
#include <mutex>
#include <vector>

#include "get_data.hpp"

std::deque<std::string> PCDQue;         // 文件夹下所有PCD
std::deque<std::string> DRJsonQue;      // 文件夹下所有dr_json
std::deque<std::string> IMUJsonQue;     // 文件夹下所有imu_json
std::deque<std::string> WHEELJsonQue;   // 文件夹下所有wheel_json


std::deque<getdata::get_PCD::PcdData> PclQue;                 // 所有包含时间、路径、文件名的PCDdata
std::deque<getdata::get_DR::PoseData> PoseQue;                // 所有从json中读到的dr_pose
std::deque<getdata::get_DR::PoseData> matched_PoseQue;        // 以pcd时间为参照，匹配到时间最近的dr_pose
std::deque<getdata::get_IMU::ImuData> ImuQue;                 // 所有从json中读到的imu
std::deque<getdata::get_WHEEL::WheelData> WheelQue;           // 所有从json中读到的wheel


getdata::get_DR::PoseData fixed_pose;
getdata::get_DR::PoseData last_pose;
pcl::PointCloud<pcl::PointXYZI>::Ptr fixed_cloud(
      new pcl::PointCloud<pcl::PointXYZI>);

void rotation2rpy(Eigen::Matrix3d &rotation_) {
  Eigen::Vector3d rpy_angles = rotation_.eulerAngles(2, 1, 0);  //xyz-前左上 常规普通rpy顺序012；  xyz-上左前 210
  float roll_degrees = rpy_angles(2) * 180.0 / M_PI;            //xyz-前上左 rs雷达 ryp 021；     xyz-左上前  车体  pyr120
  float pitch_degrees = rpy_angles(0) * 180.0 / M_PI;
  float yaw_degrees = rpy_angles(1) * 180.0 / M_PI;
  float roll_rad = rpy_angles(2);
  float pitch_rad = rpy_angles(0);
  float yaw_rad = rpy_angles(1);
  std::cout << "roll_degrees:    " << roll_degrees << "    degree" << std::endl;
  std::cout << "pitch_degrees:   " << pitch_degrees << "   degree" << std::endl;
  std::cout << "yaw_degrees:     " << yaw_degrees << "     degree\n" << std::endl;
  std::cout << "roll_rad:        " << roll_rad << "        rad" << std::endl;
  std::cout << "pitch_rad:       " << pitch_rad << "       rad" << std::endl;
  std::cout << "yaw_rad:         " << yaw_rad << "         rad\n" << std::endl;

  return;
}

// 第一帧dr和pcd对齐sec
void first_time_match(getdata::get_DR &dr, getdata::get_PCD &pcd) {
  std::cout << "------------start first_time_match------------" << std::endl;
  std::cout << "------------before match------------" << std::endl;
  std::cout << "dr_time_sec          " << PoseQue.front().dr_time_sec
            << std::endl;
  std::cout << "dr_time_nsec          " << PoseQue.front().dr_time_nsec
            << std::endl;
  std::cout << "pcd_time_sec         " << PclQue.front().pcd_time_sec
            << std::endl;
  std::cout << "pcd_time_nsec         " << PclQue.front().pcd_time_nsec
            << std::endl;

  if (PoseQue.front().dr_time_sec < PclQue.front().pcd_time_sec) {  // dr比pcd早
    while (PoseQue.front().dr_time_sec != PclQue.front().pcd_time_sec) {
      // std::cout << "PoseQue.size         " << PoseQue.size()  << std::endl;
      PoseQue.pop_front();
      // std::cout << "PoseQue.size         " << PoseQue.size() << std::endl;
    }
  } else if (PoseQue.front().dr_time_sec >
             PclQue.front().pcd_time_sec) {  // pcd比dr早
    while (PoseQue.front().dr_time_sec != PclQue.front().pcd_time_sec) {
      // std::cout << "befor PclQue.size         " << PclQue.size()  <<
      // std::endl;
      PclQue.pop_front();
      // std::cout << "after PclQue.size         " << PclQue.size()  <<
      // std::endl;
    }
  }
  std::cout << "------------after match------------" << std::endl;
  std::cout << "dr_time_sec           " << PoseQue.front().dr_time_sec
            << std::endl;
  std::cout << "dr_time_nsec          " << PoseQue.front().dr_time_nsec
            << std::endl;
  std::cout << "pcd_time_sec          " << PclQue.front().pcd_time_sec
            << std::endl;
  std::cout << "pcd_time_nsec         " << PclQue.front().pcd_time_nsec
            << std::endl;

  std::cout << "PoseQue_size          " << PoseQue.size() << std::endl;
  std::cout << "PclQue                " << PclQue.size()  << std::endl;

  std::cout << "------------finish first time match------------" << std::endl;

  return;
}

// 最后一帧dr和pcd对齐sec
void last_time_match(getdata::get_DR &dr, getdata::get_PCD &pcd) {
  std::cout << "------------start last_time_match------------" << std::endl;
  std::cout << "------------before match------------" << std::endl;
  std::cout << "dr_time_sec          " << PoseQue.back().dr_time_sec
            << std::endl;
  std::cout << "dr_time_nsec          " << PoseQue.back().dr_time_nsec
            << std::endl;
  std::cout << "pcd_time_sec         " << PclQue.back().pcd_time_sec
            << std::endl;
  std::cout << "pcd_time_nsec         " << PclQue.back().pcd_time_nsec
            << std::endl;

  if (PoseQue.back().dr_time_sec < PclQue.back().pcd_time_sec) {  // dr比pcd早
    // std::cout << "dr_time_sec         " << PoseQue.back().dr_time_sec  <<
    // std::endl; std::cout << "pcl_time_sec         " <<
    // PclQue.back().dr_time_sec  << std::endl;
    while (PoseQue.back().dr_time_sec != PclQue.back().pcd_time_sec) {
      // std::cout << "PoseQue.size         " << PoseQue.size()  << std::endl;
      PclQue.pop_back();
      // std::cout << "PoseQue.size         " << PoseQue.size() << std::endl;
    }
  } else if (PoseQue.back().dr_time_sec >
             PclQue.back().pcd_time_sec) {  // pcd比dr早
    while (PoseQue.back().dr_time_sec != PclQue.back().pcd_time_sec) {
      // std::cout << "befor PclQue.size         " << PclQue.size()  <<
      // std::endl;
      PoseQue.pop_back();
      // std::cout << "after PclQue.size         " << PclQue.size()  <<
      // std::endl;
    }
  }
  std::cout << "------------after match------------" << std::endl;
  std::cout << "dr_time_sec          " << PoseQue.back().dr_time_sec
            << std::endl;
  std::cout << "dr_time_nsec          " << PoseQue.back().dr_time_nsec
            << std::endl;
  std::cout << "pcd_time_sec         " << PclQue.back().pcd_time_sec
            << std::endl;
  std::cout << "pcd_time_nsec         " << PclQue.back().pcd_time_nsec
            << std::endl;

  std::cout << "PoseQue_size          " << PoseQue.size() << std::endl;
  std::cout << "PclQue                " << PclQue.size()  << std::endl;

  std::cout << "------------finish last time match------------" << std::endl;

  return;
}

// 双指针没写好
// void match_files() {
//   std::cout << "------------start match files------------" << std::endl;
//   int left = 0;
//   int right = 0;
//   for (int i = 0; i < PclQue.size(); i++) {
//     // right = left;
//     while (right < PoseQue.size()) {
//       std::cout << "left     " << left << std::endl;
//       std::cout << "right    " << right << std::endl;
//       std::cout << "i        " << i << std::endl;
//       while (PoseQue[right].dr_time_sec == PclQue[i].pcd_time_sec) {  //
//       计算所有sec相同的dr帧与pcd的nsec时间误差，选出最近的dr帧
//         int dr_nsec = std::stoi(PoseQue[right].dr_time_nsec);
//         int pcd_nsec = std::stoi(PclQue[i].pcd_time_nsec);
//         int dis = std::abs(dr_nsec - pcd_nsec);
//         std::cout << "dr_nsec      " << dr_nsec << std::endl;
//         std::cout << "pcd_nsec     " << pcd_nsec << std::endl;
//         std::cout << "dis          " << dis << std::endl;
//         if (dis < min_dis) {
//           std::cout << "11111111111111111111111111dis miner " << std::endl;
//           min_dis = dis;
//           std::cout << "min_dis      " << min_dis << std::endl;
//           left = right;
//           std::cout << "left     " << left << std::endl;
//           std::cout << "right    " << right << std::endl;
//         }
//         right++;
//       }
//       matched_PoseQue.push_back(PoseQue[left]);
//       left = right;
//       break;
//     }
//   }
//   std::cout << "PclQue.size()     " << PclQue.size() << std::endl;
//   std::cout << "PoseQue.size()     " << PoseQue.size() << std::endl;
//   std::cout << "matched_PoseQue.size()     " << matched_PoseQue.size() <<
//   std::endl; std::cout << "------------finish match files------------" <<
//   std::endl; return;
// }
// 依据时间差匹配dr和pcd
void match_files() {
  matched_PoseQue.clear();
  std::cout << "------------start match files------------" << std::endl;
  for (int i = 0; i < PclQue.size(); i++) {
    int flag = 0;
    getdata::get_DR::PoseData temp_pose;
    long min_dis = 999999999;

    std::string s_pcl_time = PclQue[i].pcd_time_sec + PclQue[i].pcd_time_nsec;
    long l_pcl_time = std::stol(s_pcl_time);

    for (int j = 0; j < PoseQue.size(); j++) {
      std::string s_pose_time =
          PoseQue[j].dr_time_sec + PoseQue[j].dr_time_nsec;
      long l_pose_time = std::stol(s_pose_time);
      // std::cout << "l_pcl_time          " << l_pcl_time << std::endl;
      // std::cout << "l_pose_time         " << l_pose_time << std::endl;
      long dis = std::abs(l_pcl_time - l_pose_time);

      //            dr:  1695780588.944840908
      //            pcd: 1695780587.944042240
      if ((dis < min_dis) && (dis < 900000000) &&
          !PoseQue[j].pose_used) {  // pose仅匹配一次
        // std::cout << "used?     " << PoseQue[j].pose_used << std::endl;
        min_dis = dis;
        // std::cout << "dis         " << dis << std::endl;
        temp_pose = PoseQue[j];
        flag = j;
      }
    }

    if (!PoseQue[flag].pose_used) {
      // std::cout << "now_pcd       " << PclQue[i].pcd_name << std::endl;
      // std::cout << "temp_pose     " << temp_pose.dr_name << std::endl;

      matched_PoseQue.emplace_back(temp_pose);
      PoseQue[flag].pose_used = true;

      if (flag == (PoseQue.size())) {
        for (int k = i - 1; k < PclQue.size(); k++) {
          std::cout << "erase:       " << i << std::endl;
          auto iter = PclQue.begin() + k;
          PclQue.erase(iter);
        }
      }
    }
  }
  std::cout << "PclQue.size()            " << PclQue.size() << std::endl;
  std::cout << "matched_PoseQue.size()   " << matched_PoseQue.size()
            << std::endl;

  std::cout << "------------finish match files------------" << std::endl;
  return;
}

// 保存PclQue和matched_PoseQue
bool save_que_to_file() {
  if (PclQue.size() != matched_PoseQue.size()) {
    std::cout << "------------file match failed------------" << std::endl;
    return false;
  } else   {
    std::string que_file_path =
        "/media/pw/data1/cjy_data/10193954/matched_files.txt";
    std::ofstream results;
    results.open(que_file_path);  //, std::ios::out | std::ios::app);
    results << PclQue.size() << "\n";
    results << matched_PoseQue.size() << "\n\n";

    for (int i = 0; i < matched_PoseQue.size(); i++) {
      results << i << "\n";
      results << PclQue[i].pcd_name << "\n";
      results << matched_PoseQue[i].dr_name << "\n\n";
    }
    results.close();
  }
  std::cout << "------------file match saved------------" << std::endl;
  return true;
}

// void read_one_pcl_from_file(
//     std::shared_ptr<getData::DataOP> &data_loader,  // NOLINT
//     int mode) {
//   mtx_buffer.lock();
//   scan_count++;
//   double preprocess_start_time = omp_get_wtime();
  
//   if (p_pre->lidar_type == RS128) {
//     getData::PointCloudRS128 cloud;
//     cloud = data_loader->readRS128Cloud();
//     if (cloud.points.size() != 0) {
//       PointCloudXYZI::Ptr ptr(new PointCloudXYZI());
//       p_pre->process(cloud, ptr);
//       cloud.header.stamp =
//           p_pre->rs128_pointcloud_timestamp * 1e9;  // unit nanosecond
//       lidar_buffer.push_back(ptr);
//       double timestamp = p_pre->rs128_pointcloud_timestamp;  // unit second
//       if (timestamp < last_timestamp_lidar) {
//         LOG(WARNING) << "lidar loop back, clear buffer";
//         lidar_buffer.clear();
//       }
//       // ROS_INFO("get a lidar data");
//       time_buffer.push_back(timestamp);
//       last_timestamp_lidar = timestamp;
//     }
//   } 
// }

// void undistor() {
  
//   return;
// }


// 计算dr间的变换矩阵   n-->p
Eigen::Matrix4d calculate_pTn(getdata::get_DR::PoseData pre_pose_,
                                   getdata::get_DR::PoseData now_pose_,
                                   Eigen::Matrix4d &trans_pTn_) {
  // std::cout << "------------start claculate T------------" << std::endl;
  std::cout << "now_pose_dr_name     " << now_pose_.dr_name << std::endl;

  // std::cout << "------------claculate t------------" << std::endl;
  Eigen::Vector3d ptn = pre_pose_.dr_pose - now_pose_.dr_pose;
  std::cout << "T t  " << std::endl << ptn << std::endl;

  // std::cout << "------------claculate R by Q------------" << std::endl;
  now_pose_.dr_quaternion.normalize();

  Eigen::Matrix3d pRn = pre_pose_.dr_quaternion.conjugate().toRotationMatrix() * now_pose_.dr_quaternion.toRotationMatrix();
  // std::cout << "pRn  " << std::endl << pRn << std::endl;

  std::cout << "T R  " << std::endl << pRn << std::endl;
  // rotation2rpy(pRn);

  trans_pTn_.block<3, 3>(0, 0) = pRn;
  trans_pTn_.block<3, 1>(0, 3) = ptn;
  
  std::cout << "trans T  " << std::endl << trans_pTn_ << std::endl;

  std::cout << "------------finish claculate T------------" << std::endl;
  return trans_pTn_;
}

void get_fixed() {
  fixed_pose = matched_PoseQue.front();
  std::cout << "fixed pose time:     " << fixed_pose.dr_time_sec << "." << fixed_pose.dr_time_nsec << std::endl;
  fixed_pose.dr_quaternion.normalize();
  std::cout << "fixed Q: (" << fixed_pose.dr_quaternion.w() << ", "
            << fixed_pose.dr_quaternion.x() << ", "
            << fixed_pose.dr_quaternion.y() << ", "
            << fixed_pose.dr_quaternion.z() << ")" << std::endl;

  pcl::io::loadPCDFile<pcl::PointXYZI>(PclQue.front().pcd_path, *fixed_cloud);
  std::cout << "------------loaded fixed cloud------------" << std::endl;
  return;
}

void trans_L2f() {
  int add_num = 0;
  pcl::PointCloud<pcl::PointXYZI>::Ptr add_pcd(
      new pcl::PointCloud<pcl::PointXYZI>);
  *add_pcd += *fixed_cloud;

  pcl::PointCloud<pcl::PointXYZI>::Ptr pre_trans_cloud(
      new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr aft_trans_cloud(
      new pcl::PointCloud<pcl::PointXYZI>);

  double dis_thred = 0.2; // 相邻两帧点云间隔距离
  double trans_thred = 5; // 收尾两帧点云间隔距离

  Eigen::Matrix4d trans_fTn = Eigen::Matrix4d::Identity();  // 当前帧-->fixed 增量式
  Eigen::Matrix4d trans_lTn = Eigen::Matrix4d::Identity();  // 当前帧-->上一帧
  double f_tdiss_n = 0;  // 当前帧-->fixed 位移
  double l_tdiss_n = 0;  //当前帧-->上一帧 位移
  last_pose = fixed_pose;

  for (int i = 1; i < matched_PoseQue.size(); i++) { // 跳过计算第一帧固定帧
    trans_lTn = calculate_pTn(last_pose, matched_PoseQue[i], trans_lTn);
    double t_x = trans_lTn(0, 3);
    double t_y = trans_lTn(1, 3);
    double l_tdiss_n = std::sqrt(std::pow(t_x, 2) + std::pow(t_y, 2));

    if (l_tdiss_n < dis_thred) {
      continue;
    }

    f_tdiss_n += l_tdiss_n;
    trans_fTn *= trans_lTn;
    last_pose = matched_PoseQue[i];

    long dis_time_sec = std::stol(matched_PoseQue[i].dr_time_sec) -
                        std::stol(fixed_pose.dr_time_sec);
    std::cout << "dis_time_sec            " << dis_time_sec << std::endl;
    long dis_time_nsec = std::stol(matched_PoseQue[i].dr_time_nsec) -
                         std::stol(fixed_pose.dr_time_nsec);
    std::cout << "dis_time_nsec           " << dis_time_nsec << std::endl;

    pcl::io::loadPCDFile<pcl::PointXYZI>(PclQue[i].pcd_path, *pre_trans_cloud);
    std::cout << "------------loaded pre cloud------------" << std::endl;

    pcl::transformPointCloud(*pre_trans_cloud, *aft_trans_cloud,
                             trans_fTn);  // pre --tran--> aft

    std::cout << PclQue[i].pcd_name << std::endl;

    pcl::io::savePCDFileASCII("/media/pw/data1/cjy_data/dr/10193954/res/transed" + PclQue[i].pcd_name, *aft_trans_cloud);

    std::cout << "------------transed cloud------------" << std::endl;
    // pcl::io::savePCDFileASCII("/home/pw/Desktop/pcdpcd/after_trans.pcd" + i,
    // *aft_trans_cloud); std::cout << "saved transed cloud" << std::endl;

    *add_pcd += *aft_trans_cloud;
    add_num++;
    std::cout << "add num      " << add_num << std::endl;

    if (f_tdiss_n > trans_thred) {
      break;
    }
  }
  std::cout << "------------start save final pcd------------" << std::endl;
  pcl::io::savePCDFileASCII("/media/pw/data1/cjy_data/dr/10193954/res/add_pcd.pcd", *add_pcd);
}

void trans_pcd_L_T_I(Eigen::Matrix4f L_T_I_) {
  std::cout << PclQue.size() << std::endl;
  while(!PclQue.empty()) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr pre_trans_cloud(
      new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr aft_trans_cloud(
      new pcl::PointCloud<pcl::PointXYZI>);

    pcl::io::loadPCDFile<pcl::PointXYZI>(PclQue.front().pcd_path, *pre_trans_cloud);
    pcl::transformPointCloud(*pre_trans_cloud, *aft_trans_cloud, L_T_I_);  // pre --tran--> aft
    std::cout << PclQue.front().pcd_name << std::endl;
    pcl::io::savePCDFileASCII("/media/pw/data1/cjy_data/lio/1009/subpcd/output_pcd/aft" + PclQue.front().pcd_name, *aft_trans_cloud);
    std::cout << "finish         " << std::endl;
    PclQue.pop_front();

  }
  
  return;
}

void cal_T_rpy() {
  Eigen::Matrix3d old_extrin;
  old_extrin << -0.0031189598549765194, -0.99998180471893194, -0.0051635569610389634,  
                -0.073016140628518703, 0.0053775315812755698, -0.99731626145461383,
                0.99732588219555351,  -0.0027335663809816197, -0.073031584384438394;
  std::cout << "old_extrin" << std::endl;
  rotation2rpy(old_extrin);

  Eigen::Matrix3d t_1_extrin;
  t_1_extrin << -0.0038777428235728831, -0.99992504301525387, -0.011613417306567404,  
                -0.073007157572826789, 0.011865598569986322, -0.99726082972997143,
                0.99732387821295565,  -0.0030192584384561762, -0.073047696916132429;
  std::cout << "t_1_extrin" << std::endl;
  rotation2rpy(t_1_extrin);

  Eigen::Matrix3d t_2_extrin;
  t_2_extrin << -0.0045068694437556643, -0.99997649872838668, -0.0051662480324685409,  
                -0.072377905622873093, 0.0054789471972761664, -0.99736223104509769,
                0.99736709736458784,  -0.0041210591509347663, -0.072400897549761659;
  std::cout << "t_2_extrin" << std::endl;  
  rotation2rpy(t_2_extrin);

  Eigen::Matrix3d t_3_extrin;
  t_3_extrin << -0.0023537909675873984, -0.9999796814822649, -0.0059242122436430136,  
                -0.071758230084385921, 0.0060778576139409266, -0.99740353722150998,
                0.99741927797854213,  -0.0019225684517036401, -0.071771078069869362;
  std::cout << "t_3_extrin" << std::endl;
  rotation2rpy(t_3_extrin);
  
  return;
}

int main(int argc, char **argv) {
  std::string dr_path = "/media/pw/data1/cjy_data/dr/10193954/dr_json";
  std::string pcd_path = "/media/pw/data1/cjy_data/dr/10193954/rs_pcd";
  std::string imu_path = "/media/pw/data1/cjy_data/dr/10193954/rs_pcd";
  std::string wheel_path = "/media/pw/data1/cjy_data/dr/10193954/rs_pcd";

  // std::string dr_path = "/home/pw/cjy_home/data/drpose";
  // std::string pcd_path = "/home/pw/cjy_home/data/pcd";
  getdata::get_files g_files;
  // g_files.setfilepath(dr_path, pcd_path);
  g_files.setfilepath(dr_path, pcd_path, imu_path, wheel_path);
  g_files.readDRJsonPaths(DRJsonQue);
  g_files.readPCDPaths(PCDQue);
  g_files.readIMUPaths(IMUJsonQue);
  g_files.readWHEELPaths(WHEELJsonQue);

  cal_T_rpy();
  
  std::cout << "------------all files readed------------" << std::endl;

  getdata::get_DR g_dr;
  getdata::get_PCD g_pcd;
  getdata::get_IMU g_imu;
  getdata::get_WHEEL g_wheel;

  // std::cout << "DRJsonQue.empty       "  << DRJsonQue.empty() << std::endl;
  while (!DRJsonQue.empty()) {
    getdata::get_DR::PoseData one_pose;
    one_pose = g_dr.readOneDRFromJson(DRJsonQue);
    PoseQue.push_back(one_pose);
    if (one_pose.one_pose_readed) {
      DRJsonQue.pop_front();
    }
  }

  // std::cout << "PCDQue.empty       "  << PCDQue.empty() << std::endl;
  while (!PCDQue.empty()) {
    getdata::get_PCD::PcdData one_pcd;
    one_pcd = g_pcd.readOnePcd(PCDQue);
    PclQue.push_back(one_pcd);
    if (one_pcd.one_pcd_readed) {
      PCDQue.pop_front();
    }
  }

  while (!IMUJsonQue.empty()) {
    getdata::get_IMU::ImuData one_imu;
    one_imu = g_imu.readOneImuFromJson(IMUJsonQue);
    ImuQue.push_back(one_imu);
    if (one_imu.one_imu_readed) {
      IMUJsonQue.pop_front();
    }
  }

  while (!WHEELJsonQue.empty()) {
    getdata::get_WHEEL::WheelData one_wheel;
    one_wheel = g_wheel.readOneWheelFromJson(WHEELJsonQue);
    WheelQue.push_back(one_wheel);
    if (one_wheel.one_wheel_readed) {
      WHEELJsonQue.pop_front();
    }
  }


  first_time_match(g_dr, g_pcd);
  last_time_match(g_dr, g_pcd);
  std::cout << "PclQue.size()            " << PclQue.size() << std::endl;
  std::cout << "matched_PoseQue.size()   " << matched_PoseQue.size() << std::endl;
  match_files();
  if(save_que_to_file()) {
    // undistor();
    // get_fixed();
    // trans_L2f();
  }

  // ---------------------------------after lio  IMU --> Lidar

  // std::string dr_path = "/media/pw/data1/cjy_data/dr/10193954/dr_json";
  // std::string pcd_path = "/media/pw/data1/cjy_data/lio/1009/subpcd";
  // getdata::get_files g_files;
  // g_files.setfilepath(dr_path, pcd_path);
  // g_files.readPCDPaths(PCDQue);
  
  // getdata::get_PCD g_pcd;
  // while (!PCDQue.empty()) {
  //   getdata::get_PCD::PcdData one_pcd;
  //   one_pcd = g_pcd.readOnePcd(PCDQue);
  //   PclQue.push_back(one_pcd);
  //   if (one_pcd.one_pcd_readed) {
  //     PCDQue.pop_front();
  //   }
  // }

  // Eigen::Matrix4f L_T_I = Eigen::Matrix4f::Identity();
  // L_T_I <<1, 0, 0, -1.810,
  //         0, 1, 0, 0,
  //         0, 0, 1, -0.043,
  //         0, 0, 0, 1;

  // trans_pcd_L_T_I(L_T_I);
  
  
  
  std::cout << "------------finish all------------" << std::endl;

  return 0;
}