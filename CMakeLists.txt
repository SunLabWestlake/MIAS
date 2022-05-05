#CMakeLists.txt
set(CV_INCLUDE "E:\\opencv401\\build\\include")
set(OPENCV_LIB "E:\\opencv401\\build\\x64\\vc15\\lib")

set(JSON_INCLUDE "E:\\git\\jsoncpp-master\\include\\json")
set(JSON_LIB "E:\\git\\jsoncpp-master\\build\\src\\lib_json\\Release")

set(PTHREAD_INCLUDE "D:\\pthread-Pre-built.2\\include")
set(PTHREAD_LIB "D:\\pthread-Pre-built.2\\lib\\x64")

set(FLIR_INCLUDE "D:\\Program Files\\FLIR Systems\\Spinnaker\\include")
set(FLIR_LIB "D:\\Program Files\\FLIR Systems\\Spinnaker\\lib64\\vs2015")

set(BASLER_INCLUDE "D:\\Program Files\\Basler\\pylon 6\\Development\\include")
set(BASLER_LIB "D:\\Program Files\\Basler\\pylon 6\\Development\\lib\\x64")

set(DAHENG_INCLUDE "D:\\Program Files\\Daheng Imaging\\GalaxySDK\\Samples\\VC SDK\\inc")
set(DAHENG_LIB "D:\\Program Files\\Daheng Imaging\\GalaxySDK\\Samples\\VC SDK\\lib\\x64")

set(MVS_INCLUDE "D:\\Program Files (x86)\\MVS\\Development\\Includes")
set(MVS_LIB "D:\\Program Files (x86)\\MVS\\Development\\Libraries\\win64")

cmake_minimum_required(VERSION 3.0)
project(MIAS CXX)
include_directories(${CV_INCLUDE};${JSON_INCLUDE};${PTHREAD_INCLUDE};${FLIR_INCLUDE};${BASLER_INCLUDE};${DAHENG_INCLUDE};${MVS_INCLUDE})
link_directories(MIAS ${OPENCV_LIB};${JSON_LIB};${PTHREAD_LIB};${FLIR_LIB};${BASLER_LIB};${DAHENG_LIB};${MVS_LIB})
add_executable(MIAS MIAS.cpp)
