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

std::deque<std::string> PCDQue;     // 文件夹下所有PCD
std::deque<std::string> DRJsonQue;  // 文件夹下所有dr_json
std::deque<getdata::get_PCD::PcdData> PclQue;  // 所有包含时间、路径、文件名的PCDdata
std::deque<getdata::get_DR::PoseData> PoseQue;  // 所有从json中读到的dr_pose
std::deque<getdata::get_DR::PoseData>
    matched_PoseQue;  // 以pcd时间为参照，匹配到时间最近的dr_pose
getdata::get_DR::PoseData fixed_pose;
pcl::PointCloud<pcl::PointXYZI>::Ptr fixed_cloud(
      new pcl::PointCloud<pcl::PointXYZI>);

void rotation2rpy(Eigen::Matrix3d &rotation_) {
  Eigen::Vector3d rpy_angles = rotation_.eulerAngles(2, 1, 0);
  float roll_degrees = rpy_angles(2) * 180.0 / M_PI;
  float pitch_degrees = rpy_angles(1) * 180.0 / M_PI;
  float yaw_degrees = rpy_angles(0) * 180.0 / M_PI;
  std::cout << "rotation_" << std::endl;
  std::cout << "Roll (X-axis): " << roll_degrees << " degrees" << std::endl;
  std::cout << "Pitch (Y-axis): " << pitch_degrees << " degrees" << std::endl;
  std::cout << "Yaw (Z-axis): " << yaw_degrees << " degrees" << std::endl;
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
        "/media/pw/data1/cjy_data/09271017/matched_files.txt";
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

// 计算dr间的变换矩阵
Eigen::Matrix4d calculate_trans_dr(getdata::get_DR::PoseData fixed_pose_,
                                   getdata::get_DR::PoseData p_,
                                   Eigen::Matrix4d &trans_dr_) {
  // std::cout << "------------start claculate T------------" << std::endl;
  std::cout << "p_dr_name     " << p_.dr_name << std::endl;
  getdata::get_DR::PoseData pre_trans = p_;

  // std::cout << "------------claculate t------------" << std::endl;
  Eigen::Vector3d t_dr = p_.dr_pose - fixed_pose_.dr_pose;
  // std::cout << "T t  " << std::endl << t_dr << std::endl;

  // std::cout << "------------claculate R by Q------------" << std::endl;
  p_.dr_quaternion.normalize();

  Eigen::Matrix3d R_p_fixed_Q =
      p_.dr_quaternion.toRotationMatrix() *
      fixed_pose_.dr_quaternion.conjugate().toRotationMatrix();
  // std::cout << "R_p_fixed_Q  " << std::endl << R_p_fixed_Q << std::endl;

  // std::cout << "T R_Q  " << std::endl << R_p_fixed_Q << std::endl;
  // rotation2rpy(R_p_fixed_Q);

  trans_dr_.block<3, 3>(0, 0) = R_p_fixed_Q;
  trans_dr_.block<3, 1>(0, 3) = t_dr;
  // std::cout << "trans T  " << std::endl << trans_dr_ << std::endl;

  std::cout << "------------finish claculate T------------" << std::endl;
  return trans_dr_;
}

void test_calcu_R() {
  //       1695780589.016092777.json        1695780629.995780468.json
  // RS128_1695780589_044048128.pcd   RS128_1695780629_944593152.pcd

  // cal R
  Eigen::Quaterniond quaternion1(0.9978914856910706, 0.0, 0.0,
                                 0.0649048388004303);
  Eigen::Quaterniond quaternion2(0.9984201788902283, 0.0, 0.0,
                                 0.0561889186501503);
  quaternion1.normalize();
  quaternion2.normalize();
  Eigen::Matrix3d Q_rotation_matrix =
      quaternion2.toRotationMatrix() *
      quaternion1.conjugate().toRotationMatrix();
  std::cout << "Q_rotation_matrix  " << std::endl
            << Q_rotation_matrix << std::endl;
  rotation2rpy(Q_rotation_matrix);

  Eigen::Vector3d euler_angles1(0.6499999761581421, -0.029999999329447746,
                                7.443326950073242);
  Eigen::Vector3d euler_angles2(1.1799999475479126, 0.14000000059604645,
                                6.442643165588379);

  Eigen::Matrix3d rotation_matrix1;
  rotation_matrix1 =
      Eigen::AngleAxisd(euler_angles1[0], Eigen::Vector3d::UnitX()) *
      Eigen::AngleAxisd(euler_angles1[1], Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(euler_angles1[2], Eigen::Vector3d::UnitZ());

  Eigen::Matrix3d rotation_matrix2;
  rotation_matrix2 =
      Eigen::AngleAxisd(euler_angles2[0], Eigen::Vector3d::UnitX()) *
      Eigen::AngleAxisd(euler_angles2[1], Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(euler_angles2[2], Eigen::Vector3d::UnitZ());

  Eigen::Matrix3d E_rotation_matrix =
      rotation_matrix2 * rotation_matrix1.transpose();
  std::cout << "E_rotation_matrix  " << std::endl
            << E_rotation_matrix << std::endl;
  rotation2rpy(E_rotation_matrix);

  // cal t
  Eigen::Vector3d pose1(-206.11752291698429, -196.7775533712687, 0.0);
  Eigen::Vector3d pose2(-122.28197590525846, -185.79892610874197, 0.0);
  Eigen::Vector3d t = pose2 - pose1;
  std::cout << "t  " << std::endl << t << std::endl;

  // cal T
  Eigen::Matrix4d T_Q = Eigen::Matrix4d::Identity();
  Eigen::Matrix4d T_E = Eigen::Matrix4d::Identity();

  T_Q.block<3, 3>(0, 0) = Q_rotation_matrix;
  T_Q.block<3, 1>(0, 3) = t;

  T_E.block<3, 3>(0, 0) = E_rotation_matrix;
  T_E.block<3, 1>(0, 3) = t;

  // load pcd
  std::string pcd_name1 =
      "/media/pw/data/cjy_data/09271010/rs_pcd/RS128_1695780589_044048128.pcd";
  std::string pcd_name2 =
      "/media/pw/data/cjy_data/09271010/rs_pcd/RS128_1695780629_944593152.pcd";

  pcl::PointCloud<pcl::PointXYZI>::Ptr pcd1(
      new pcl::PointCloud<pcl::PointXYZI>);
  pcl::io::loadPCDFile<pcl::PointXYZI>(pcd_name1, *pcd1);

  pcl::PointCloud<pcl::PointXYZI>::Ptr pcd2(
      new pcl::PointCloud<pcl::PointXYZI>);
  pcl::io::loadPCDFile<pcl::PointXYZI>(pcd_name2, *pcd2);

  pcl::PointCloud<pcl::PointXYZI>::Ptr pcd2_trans_Q(
      new pcl::PointCloud<pcl::PointXYZI>);
  pcl::io::loadPCDFile<pcl::PointXYZI>(pcd_name2, *pcd2_trans_Q);

  pcl::PointCloud<pcl::PointXYZI>::Ptr pcd2_trans_E(
      new pcl::PointCloud<pcl::PointXYZI>);
  pcl::io::loadPCDFile<pcl::PointXYZI>(pcd_name2, *pcd2_trans_E);

  pcl::transformPointCloud(*pcd2, *pcd2_trans_Q, T_Q);
  pcl::io::savePCDFileASCII("/home/pw/Desktop/pcdpcd/pcd2_trans_Q.pcd",
                            *pcd2_trans_Q);

  pcl::transformPointCloud(*pcd2, *pcd2_trans_E, T_E);
  pcl::io::savePCDFileASCII("/home/pw/Desktop/pcdpcd/pcd2_trans_E.pcd",
                            *pcd2_trans_E);

  pcl::PointCloud<pcl::PointXYZI>::Ptr add_pcd(
      new pcl::PointCloud<pcl::PointXYZI>);

  *add_pcd += *pcd1;
  *add_pcd += *pcd2_trans_Q;
  pcl::io::savePCDFileASCII("/home/pw/Desktop/pcdpcd/add_pcd.pcd", *add_pcd);

  // result： Q is better

  return;
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
  double last_t = 0;
  pcl::PointCloud<pcl::PointXYZI>::Ptr pre_trans_cloud(
      new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr aft_trans_cloud(
      new pcl::PointCloud<pcl::PointXYZI>);

  double dis_thred = 0.1; // 相邻两帧点云间隔距离
  double trans_thred = 2; // 收尾两帧点云间隔距离

  for (int i = 0; i < matched_PoseQue.size(); i++) {
    Eigen::Matrix4d trans_dr = Eigen::Matrix4d::Identity();
    trans_dr = calculate_trans_dr(fixed_pose, matched_PoseQue[i], trans_dr);
    double t_x = trans_dr(0, 3);
    double t_y = trans_dr(1, 3);
    // double trans_t = std::sqrt((t_x * t_x) + (t_y * t_y));
    double trans_t = std::sqrt(std::pow(t_x, 2) + std::pow(t_y, 2));
    double dis_t = trans_t - last_t;
    // 0.05米拼一次
    if (dis_t < dis_thred) {
      continue;
    }
    // 10米之内
    if (trans_t > trans_thred) {
      break;
    }
    last_t = trans_t;

    long dis_time_sec = std::stol(matched_PoseQue[i].dr_time_sec) -
                        std::stol(fixed_pose.dr_time_sec);
    std::cout << "dis_time_sec            " << dis_time_sec << std::endl;
    long dis_time_nsec = std::stol(matched_PoseQue[i].dr_time_nsec) -
                         std::stol(fixed_pose.dr_time_nsec);
    std::cout << "dis_time_nsec           " << dis_time_nsec << std::endl;

    pcl::io::loadPCDFile<pcl::PointXYZI>(PclQue[i].pcd_path, *pre_trans_cloud);
    std::cout << "------------loaded pre cloud------------" << std::endl;

    pcl::transformPointCloud(*pre_trans_cloud, *aft_trans_cloud,
                             trans_dr);  // pre --tran--> aft
    std::cout << "------------transed cloud------------" << std::endl;
    // pcl::io::savePCDFileASCII("/home/pw/Desktop/pcdpcd/after_trans.pcd" + i,
    // *aft_trans_cloud); std::cout << "saved transed cloud" << std::endl;

    *add_pcd += *fixed_cloud;
    *add_pcd += *aft_trans_cloud;
    add_num++;
    std::cout << "add num      " << add_num << std::endl;
    // pcl::io::savePCDFileASCII("/home/pw/Desktop/pcdpcd/add_pcd.pcd",
    // *add_pcd);
  }
  std::cout << "------------start save final pcd------------" << std::endl;
  pcl::io::savePCDFileASCII("/media/pw/data1/cjy_data/09271017/res/add_pcd.pcd",
  *add_pcd);
}


int main(int argc, char **argv) {
  std::string dr_path = "/media/pw/data1/cjy_data/09271017/dr_json";
  std::string pcd_path = "/media/pw/data1/cjy_data/09271017/rs_pcd";

  // std::string dr_path = "/home/pw/cjy_home/data/drpose";
  // std::string pcd_path = "/home/pw/cjy_home/data/pcd";
  getdata::get_files g_files;
  g_files.setfilepath(dr_path, pcd_path);
  g_files.readDRJsonPaths(DRJsonQue);
  g_files.readPCDPaths(PCDQue);
  std::cout << "------------all files readed------------" << std::endl;

  getdata::get_DR g_dr;
  getdata::get_PCD g_pcd;

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

  first_time_match(g_dr, g_pcd);
  last_time_match(g_dr, g_pcd);
  std::cout << "PclQue.size()            " << PclQue.size() << std::endl;
  std::cout << "matched_PoseQue.size()   " << matched_PoseQue.size() << std::endl;
  match_files();
  if(save_que_to_file()) {
  // test_calcu_R();
  get_fixed();
  trans_L2f();
  }
  
  std::cout << "------------finish all------------" << std::endl;

  return 0;
}