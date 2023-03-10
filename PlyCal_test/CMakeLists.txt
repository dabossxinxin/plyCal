
add_executable(test_image_polygon test_image_polygon.cpp)
target_link_libraries(test_image_polygon ${LIB_NAME})

add_executable(test_image_polygons test_image_polygons.cpp)
target_link_libraries(test_image_polygons ${LIB_NAME})

add_executable(test_pointcloud_polygon test_pointcloud_polygon.cpp)
target_link_libraries(test_pointcloud_polygon ${LIB_NAME})


