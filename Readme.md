This repository contains ROS wrappers for the V4R library.

# Installation

## From Ubuntu Packages

Coming soon ...

## From Source

Make sure you install the V4R library first. See https://rgit.acin.tuwien.ac.at/v4r/v4r for instructions.

Then clone this repository and build it with `catkin_make`. You might get an error regarding V4RModules.cmake. This is easy to fix:
```
cd my_catkin_ws/build
ccmake ../src
```
Locate the option `V4R_DIR` and set it, according to where you build/installed V4R library, e.g.:
```
V4R_DIR   /home/somewhere/v4r/build
```
Then call catkin again, and all should now compile fine.

# Tutorial
A tutorial can be found [here](Tutorial.md).

# Troubleshooting

## OpenGL not working for SiftGPU

A GPU (best: NVIDIA) is required for many components. For this to work, the user running the software needs to be allowed to access the X server. The easiest (but *very* insecure in a not secured network) way to achieve this is to allow access for all users via `DISPLAY=:0 sudo xhost +`.
Alternatively, SIFT can be computed using the non-free library of OpenCV. In this case, link the v4r and v4r\_ros\_wrappers libraries to the right OpenCV library.
