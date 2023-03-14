CMAKE_MINIMUM_REQUIRED(VERSION 3.14)

project(PlyCal)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
add_definitions(-std=c++11)

# Optionally compiling part
option(BUILD_PlyCal_GUI "enables PlyCal viewer" true)
option(BUILD_PlyCal_TEST "enables PlyCal test" true)

# The library prefix
set(LIB_NAME plycal)
set(CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake)
set(EDLINE_SRC ${PROJECT_SOURCE_DIR}/thirdParty/edlines/EDLineDetector.cpp)

include(FindEigen3)
include(FindVTK)
include(FindQt)
include(FindOpenCV)
include(FindPCL)
include(FindBoost)
include(FindFlann)
include(FindGLOG)
include(FindCeres)

# Fix a compilation bug under ubuntu 16.04 (Xenial)
list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")
add_definitions(${PCL_DEFINITIONS})
add_definitions(-DWIN32_LEAN_AND_MEAN)
add_definitions(-DGLOG_NO_ABBREVIATED_SEVERITIES)
add_definitions(-D_ENABLE_EXTENDED_ALIGNED_STORAGE)

include_directories(${PCL_INCLUDE_DIRS})
include_directories(${FLANN_INCLUDE_DIR})
include_directories(${Boost_INCLUDE_DIRS})
include_directories(${OpenCV4_INCLUDE_DIR})
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${GFLAGS_INCLUDE_DIRS})
include_directories(${GLOG_INCLUDE_DIRS})
include_directories(${CERES_INCLUDE_DIRS})
include_directories(${PROJECT_SOURCE_DIR}/PlyCal)
include_directories(${PROJECT_SOURCE_DIR}/thirdParty)
include_directories(${PROJECT_SOURCE_DIR}/thirdParty/pnpsolver/src)

add_subdirectory(PlyCal)

if(BUILD_PlyCal_GUI)
 	add_subdirectory(PlyCal_qt)
endif(BUILD_PlyCal_GUI)

if(BUILD_PlyCal_TEST)
	add_subdirectory(PlyCal_test)
endif(BUILD_PlyCal_TEST)


