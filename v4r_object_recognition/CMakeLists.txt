cmake_minimum_required(VERSION 2.8.3)
project(v4r_object_recognition)

find_package(catkin REQUIRED COMPONENTS roscpp rospy v4r_object_recognition_msgs
                                         genmsg cv_bridge sensor_msgs image_transport pcl_conversions)

find_package( PCL REQUIRED )
find_package( OpenCV REQUIRED )
find_package( V4R REQUIRED )

catkin_package(
   INCLUDE_DIRS include
   CATKIN_DEPENDS roscpp rospy v4r_object_recognition_msgs pcl_conversions
)


add_definitions(${PCL_DEFINITIONS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

include_directories(include
  ${PCL_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${V4R_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)
add_definitions(${PCL_DEFINITIONS})
link_directories(${V4R_3RDPARTY_LIB_DIR_OPT})

set(incs include/recognizer_ros.h)

add_executable(recognition_service src/recognizer_ros.cpp ${incs})
add_executable(test_object_recognition src/test_object_recognizer.cpp)

add_dependencies(recognition_service ${catkin_EXPORTED_TARGETS})
add_dependencies(test_object_recognition ${catkin_EXPORTED_TARGETS})

target_link_libraries(recognition_service
   ${catkin_LIBRARIES}
   ${PCL_LIBRARIES}
   ${OpenCV_LIBS}
   ${V4R_LIBS}
)

target_link_libraries(test_object_recognition
   ${catkin_LIBRARIES}
   ${PCL_LIBRARIES}
   ${V4R_LIBS}
   ${OpenCV_LIBS}
)


install(TARGETS recognition_service
ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS test_object_recognition
ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/
DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
FILES_MATCHING PATTERN "*.h"
PATTERN ".svn" EXCLUDE
)

install(DIRECTORY launch/
DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
FILES_MATCHING PATTERN "*.launch"
)	
