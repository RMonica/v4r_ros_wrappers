#!/bin/bash

release_messages() {
for i in *msgs; do 
	create_debian_package "$i"
done
dpkg -i ros-*msgs*_amd64.deb > /dev/null
}

release_dependency_messages() {
dpkg -i ros-*_amd64.deb > /dev/null
move_debian_packages
}

release_dependency_packages() {
for i in v4r_object_tracker v4r_object_classification v4r_segmentation v4r_object_recognition; do 
	create_debian_package "$i"
done
dpkg -i ros-*_amd64.deb > /dev/null
move_debian_packages
}

release_ros_wrappers() {
for i in `ls -d */|grep -v 'rosdep\|images\|msgs\|v4r_ros_wrappers'`; do
	create_debian_package "$i"
done
dpkg -i ros-*_amd64.deb > /dev/null
move_debian_packages
}

release_meta_package() {
create_debian_package "v4r_ros_wrappers"
dpkg -i ros-*v4r*wrappers*_amd64.deb > /dev/null
move_debian_packages
}

create_debian_package() {
cd $1
if [ -f .done ]; then
    echo "Package $1 was already build. Skipping..."
else
    echo "generating debian package for $1"
    bloom-generate rosdebian --os-name ubuntu --os-version $UBUNTU_DISTRO --ros-distro $CI_ROS_DISTRO &&\
    sed -i 's/dh  $@/dh  $@ --parallel/' debian/rules
    debuild -rfakeroot -us -uc -b -j8 > /dev/null
    touch .done
fi
cd ..
}

move_debian_packages() {
mv *deb .build/
}

release_package() {
release_dependency_messages
release_messages
release_dependency_packages
release_ros_wrappers
release_meta_package
}

show_info() {
echo $UBUNTU_DISTRO
echo $CI_ROS_DISTRO
}
