cmake_minimum_required(VERSION 2.8.3)
project(v4r_object_detection_and_tracking)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  v4r_object_tracker
  v4r_object_tracker_msgs
  pcl_conversions
  v4r_object_recognition_msgs
  roscpp
  rospy
  sensor_msgs
  v4r_object_recognition
)


find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(V4R REQUIRED)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${V4R_INCLUDE_DIRS} 
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)


add_executable(detect_and_track_object src/detect_and_track_object.cpp)
add_dependencies(detect_and_track_object ${catkin_EXPORTED_TARGETS})
target_link_libraries(detect_and_track_object ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${V4R_LIBS} ${OpenCV_LIBS})

install(TARGETS detect_and_track_object
ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


install(DIRECTORY launch/
DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
FILES_MATCHING PATTERN "*.launch"
)																																																																																																																
