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



# "eulerAngle": {
#   "x": 0.6499999761581421,
#   "y": -0.03999999910593033,
#   "z": 7.356781482696533
# },
# "quaternion": {
#   "w": 0.997940182685852,
#   "x": 0.0,
#   "y": 0.0,
#   "z": 0.06415122002363205
# },
# "position": {
#   "x": -206.11752291698383,
#   "y": -196.77755337126885,
#   "z": 0.0
# },


# "eulerAngle": {
#   "x": 0.6499999761581421,
#   "y": -0.029999999329447746,
#   "z": 7.443437576293945
# },
# "quaternion": {
#   "w": 0.997891366481781,
#   "x": 0.0,
#   "y": 0.0,
#   "z": 0.0649057999253273
# },
# "position": {
#   "x": -206.1175229169843,
#   "y": -196.7775533712687,
#   "z": 0.0
# },
