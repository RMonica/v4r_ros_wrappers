cmake_minimum_required(VERSION 2.8.3)
project(v4r_object_classification)

find_package(catkin REQUIRED COMPONENTS roscpp 
   sensor_msgs std_msgs visualization_msgs geometry_msgs
   v4r_segmentation_msgs v4r_object_classification_msgs pcl_conversions)

find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
find_package(V4R REQUIRED COMPONENTS v4r_ml v4r_segmentation v4r_recognition v4r_features v4r_io)

include_directories(${PCL_INCLUDE_DIRS})

catkin_package(
   INCLUDE_DIRS include
   CATKIN_DEPENDS v4r_object_classification_msgs pcl_conversions
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

link_directories(${V4R_3RDPARTY_LIB_DIR_OPT})
include_directories(include ${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} ${V4R_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

set(incs include/global_classifier.h)

add_executable(object_classifier_node src/global_classifier.cpp ${incs})
add_executable(object_classifier_demo_node src/test.cpp)

add_dependencies(object_classifier_demo_node ${catkin_EXPORTED_TARGETS})
add_dependencies(object_classifier_node ${catkin_EXPORTED_TARGETS})

target_link_libraries(object_classifier_node ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${V4R_LIBS} ${OpenCV_LIBS})
target_link_libraries(object_classifier_demo_node  ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${V4R_LIBS} ${OpenCV_LIBS})

install(TARGETS object_classifier_demo_node
ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS object_classifier_node
ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/
DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
FILES_MATCHING PATTERN "*.h"
PATTERN ".svn" EXCLUDE
)
