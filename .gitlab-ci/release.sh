#!/bin/bash

release_messages() {
for i in *msgs *srvs; do 
	cd $i && \
	create_debian_package
	cd ..;
done
dpkg -i ros-*srv*_amd64.deb ros-*msgs*_amd64.deb
}

release_dependency_messages() {
for i in object_perception_msgs; do 
	cd $i && \
	create_debian_package
	cd ..;
done
dpkg -i ros-*_amd64.deb
move_debian_packages
}

release_dependency_packages() {
for i in object_tracker object_classifier segmentation singleview_object_recognizer; do 
	cd $i && \
	create_debian_package
	cd ..;
done
dpkg -i ros-*_amd64.deb
move_debian_packages
}

release_ros_wrappers() {
for i in `ls -d */|grep -v 'rosdep\|images\|msgs\|srvs\|v4r_ros_wrappers\|multiview'`; do
	cd $i && \
	create_debian_package
	cd ..;
done
dpkg -i ros-*_amd64.deb
move_debian_packages
}

release_meta_package() {
cd v4r_ros_wrappers && \
create_debian_package
cd ..
dpkg -i ros-*v4r*wrappers*_amd64.deb
move_debian_packages
}

create_debian_package() {
bloom-generate rosdebian --os-name ubuntu --os-version $UBUNTU_DISTRO --ros-distro $CI_ROS_DISTRO &&\
sed -i 's/dh  $@/dh  $@ --parallel/' debian/rules
debuild -rfakeroot -us -uc -b -j8
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
