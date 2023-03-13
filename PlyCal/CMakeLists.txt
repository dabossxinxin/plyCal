
file(GLOB LIB_SOURCE "*.cpp")
file(GLOB LIB_HEADER "*.h")

source_group("Header" FILES ${LIB_HEADER})
source_group("Source" FILES ${LIB_SOURCE})

include_directories(${PCL_INCLUDE_DIRS})
include_directories(${FLANN_INCLUDE_DIR})
include_directories(${Boost_INCLUDE_DIRS})
include_directories(${OpenCV4_INCLUDE_DIR})
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${GFLAGS_INCLUDE_DIRS})
include_directories(${GLOG_INCLUDE_DIRS})
include_directories(${CERES_INCLUDE_DIRS})

link_directories(${PROJECT_SOURCE_DIR}/thirdParty/pnpsolver/lib)

add_library(${LIB_NAME} ${LIB_SOURCE} ${LIB_HEADER} ${EDLINE_SRC})

target_link_libraries(${LIB_NAME} debug pnpsolver optimized pnpsolver)
target_link_libraries(${LIB_NAME} debug ${GLOG_LIBRARIES} optimized ${GLOG_LIBRARIES})
target_link_libraries(${LIB_NAME} debug ${CERES_DEBUG_LIB} optimized ${CERES_RELEASE_LIB})
target_link_libraries(${LIB_NAME} debug ${OpenCV4_DEBUG_LIB} optimized ${OpenCV4_RELEASE_LIB})

foreach(_var ${Boost_LIBRARIES})
    target_link_libraries(${LIB_NAME} debug ${Boost_LIB_DIR}/${_var}-gd-x64-1_68.lib
	optimized ${Boost_LIB_DIR}/${_var}-x64-1_68.lib)
endforeach()

foreach(_var ${PCL_LIBRARIES})
    target_link_libraries(${LIB_NAME} debug ${PCL_LIB_DIR}/${_var}debug.lib
	optimized ${PCL_LIB_DIR}/${_var}release.lib)
endforeach()

foreach(_var ${FLANN_LIBRARIES})
    	target_link_libraries(${LIB_NAME} debug ${FLANN_LIB_DIR}/${_var}.lib
		optimized ${FLANN_LIB_DIR}/${_var}-gd.lib)
endforeach()

foreach(_var ${GFLAGS_LIBRARIES})
    target_link_libraries(${LIB_NAME} debug ${GFLAGS_LIB_DIR}/${_var}_debug.lib
	optimized ${GFLAGS_LIB_DIR}/${_var}.lib)
endforeach()