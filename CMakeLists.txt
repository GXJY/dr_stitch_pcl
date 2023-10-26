cmake_minimum_required(VERSION 2.8.3)
set (CMAKE_CXX_STANDARD 14)
project(trans_pcd)

link_directories(/usr/local/lib)

set(jsoncpp_lib libjsoncpp.a)

find_package (PCL REQUIRED)
find_package (jsoncpp REQUIRED)

# 添加头文件目录
include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${JSON_INCLUDE_DIRS}
  )
  
  add_executable(trans_pcd
  src/trans_pcd.cpp
  src/get_data.cpp
  )
  
  target_link_libraries(trans_pcd
  ${PCL_LIBRARIES}
  jsoncpp
  )
  
  message("cmake_module_path: " ${CMAKE_MODULE_PATH})
