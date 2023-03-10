
FIND_PATH(FLANN_INCLUDE_DIR NAMES FLANN HINTS ${CMAKE_SOURCE_DIR}/../../SDK/PCL-1.9.1/3rdParty/FLANN/include)
FIND_PATH(FLANN_LIB_DIR NAMES flann.lib HINTS ${CMAKE_SOURCE_DIR}/../../SDK/PCL-1.9.1/3rdParty/FLANN/lib)

SET(FLANN_LIBRARIES
	flann
	flann_cpp
	flann_cpp_s
	flann_s
	)

SET(FLANN_INCLUDE_DIRS ${FLANN_INCLUDE_DIR}/include)

