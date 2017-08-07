cmake_minimum_required(VERSION 2.8.3)
project(v4r_segment_and_classify)

find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs std_msgs visualization_msgs pcl_conversions v4r_segmentation_msgs v4r_classifier_msgs v4r_object_perception_msgs v4r_segmentation v4r_object_classifier)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(V4R REQUIRED)

catkin_package(
   CATKIN_DEPENDS v4r_segmentation_msgs v4r_classifier_msgs v4r_object_classifier v4r_segmentation pcl_conversions
)

link_directories(${V4R_3RDPARTY_LIB_DIR_OPT})
include_directories(include ${catkin_INCLUDE_DIRS} ${V4R_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
add_executable(segement_and_classify_test src/test.cpp)
add_dependencies(segement_and_classify_test ${catkin_EXPORTED_TARGETS})
target_link_libraries(segement_and_classify_test ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${V4R_LIBS} ${OpenCV_LIBS})

install(TARGETS segement_and_classify_test
ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


install(DIRECTORY launch/
DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
FILES_MATCHING PATTERN "*.launch"
)																																																																																																																
