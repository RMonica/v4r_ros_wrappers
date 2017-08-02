^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package temporal_filtering
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.1.5 (2017-07-26)
------------------
* + finished package.xml file for temporal_filtering
  + fixed warning for CMakeLists.txt
* update rosdeps
* remove version from cmake (to allow e.g. for OpenCV 3)
* Replaced temporal_filter_srv_definitions with standard ros srv definitions:
  -> stdstd_srvs/SetBool.srv
  --------------------------
  data = true    /enable
  data = false   /disable
* Outputs teomporal filtered cloud.
  Service call to turn temporal filtering on/off:
  -----------------------------------------------
  bool enable: ture/false
  Input topic:  /camera/depth_registered/points pointcloud from RGBD-Sensor
  Output topic: /temporal_filtered_cloud        the filtered pointcloud
* Contributors: Markus Suchi, Thomas Faeulhammer, Thomas Fäulhammer
