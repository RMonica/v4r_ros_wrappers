^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package object_gestalt_segmentation
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.1.5 (2017-07-26)
------------------
* + finished package.xml file for temporal_filtering
  + fixed warning for CMakeLists.txt
* remove version from cmake (to allow e.g. for OpenCV 3)
* Removed test pcd file from repository.
* This package contains a wrapper for the segmentation library in v4r, created by Ekaterina Potapova and Andreas Reichtsfeld.
  In addition it contains a testprogram 'test_object_gestalt_segmentation' which can read pointcloud files from a directory initiates the service call and optionale visualizes the output image as a ros-topic.
  The image can be viewed using the rospackage image_view.
  1. Starting the service:
  $ roslaunch object_gestalt_segmentation startup.launch
  2. Starting the test program:
  Start the test program using a directory containing pointcloud files:
  $ rosrun object_gestalt_segmentation test_object_gestalt_segmentation _input_method:=1 _directory:="/home/markus/datasets/test_object_gestalt_segmentation" _visualize:=true
* Contributors: Markus Suchi, Thomas Faeulhammer
