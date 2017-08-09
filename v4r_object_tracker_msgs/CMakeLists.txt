cmake_minimum_required(VERSION 2.8.3)
project(v4r_object_tracker_msgs)

find_package(catkin REQUIRED COMPONENTS sensor_msgs std_msgs geometry_msgs message_generation)

 add_service_files(
  FILES
   start_tracker.srv
   stop_tracker.srv
   cleanup.srv
   change_tracking_model.srv
   detect_and_track.srv
 )

generate_messages(
   DEPENDENCIES
   std_msgs
   sensor_msgs
   geometry_msgs
 )
 
catkin_package()

