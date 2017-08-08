#!/bin/bash

add_v4r_yaml() {
echo "yaml file:///`pwd`/rosdep/${CI_ROS_DISTRO}.yaml" | sudo tee /etc/ros/rosdep/sources.list.d/10-v4r.list
rosdep update > /dev/null
}

setup_apt() {
echo "deb [arch=amd64] https://rwiki.acin.tuwien.ac.at/apt/v4r-release trusty main" | sudo tee /etc/apt/sources.list.d/v4r.list
sudo apt-key adv --keyserver hkp://pgp.mit.edu:80 --recv-key 943EB54F
sudo apt-get update -qq > /dev/null
}
