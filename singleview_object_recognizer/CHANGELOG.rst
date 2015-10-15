^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package singleview_object_recognizer
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* fixed namespace and include issues to fit V4R Version 1.0.11
* Contributors: Thomas Fäulhammer

0.0.5 (2015-09-07)
------------------
* default was *very* specific and shouldn't be
  This confused a lot of people as the component simply throws a `boost::filesystem` error when the directory doesn't exist. Here, we should not define a default, but force people to actually define the argument, as it is a required one.
* Contributors: Marc Hanheide

0.0.4 (2015-08-29)
------------------
* fixed string formatting bug
* Contributors: Marc Hanheide

0.0.3 (2015-08-29)
------------------
* disbale c++11 as PCL 1.7 cannot handle it
* Contributors: Marc Hanheide

0.0.1 (2015-08-29)
------------------
* added V4R ass dependency where needed and changed maintainers to STRANDS people
* included pcl segmenter
* cleaned up launch file, removed obsolete cpp file
* removed unnecessary dependencies
* fixed camera tracker, single- and multi-view recognition with new V4R
  added some ReadMe
  still need to check classification
* added cam tracker
* tmp commit
* fixed warning of no return value
* fixed some linking and namespace issues
* tmp
  Conflicts:
  dynamic_object_learning/CMakeLists.txt
  dynamic_object_learning/package.xml
* deleted tmp commits
* .
* initial commit
* Contributors: Marc Hanheide, Thomas Fäulhammer, mzillich
