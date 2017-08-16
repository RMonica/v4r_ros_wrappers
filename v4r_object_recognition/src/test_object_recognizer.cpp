/*
 * main.cpp
 *
 *  Created on: Feb 20, 2014
 *      Author: Thomas Faeulhammer
 */

#include <pcl/common/common.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/String.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <v4r/common/miscellaneous.h>
#include <v4r/common/pcl_opencv.h>
#include <v4r/io/filesystem.h>

#include "v4r_object_recognition_msgs/recognize.h"
#include "v4r_object_recognition_msgs/set_camera.h"

class ObjectRecognizerDemo
{
private:
    typedef pcl::PointXYZRGB PointT;
    ros::NodeHandle *n_;
    ros::ServiceClient obj_rec_client_, obj_rec_set_camera_client_;
    std::string directory_;
    std::string topic_;
    sensor_msgs::CameraInfo camera_info_;

    bool KINECT_OK_;
    int input_method_; // defines the test input (0... camera topic, 1... file)
    boost::shared_ptr<image_transport::ImageTransport> it_;
    image_transport::Publisher image_pub_;
    ros::Subscriber cam_info_sub_;
    bool camera_info_received_;

    void camera_info_cb(const sensor_msgs::CameraInfoPtr& msg)
    {
        if(!camera_info_received_)
        {
            camera_info_ = *msg;
            std::cout << "Got camera info." << std::endl;
            camera_info_received_ = true;
        }
    }

public:
    ObjectRecognizerDemo()
        :input_method_(0),
          camera_info_received_ (false)
    {}

    void callObjectRecognizerUsingCam(const sensor_msgs::PointCloud2::ConstPtr& msg)
    {
        std::cout << "Received point cloud.\n" << std::endl;

        v4r_object_recognition_msgs::set_camera set_cam_srv;
        set_cam_srv.request.cam = camera_info_;
        if (obj_rec_set_camera_client_.call(set_cam_srv))
        {
            std::cout << "Set camera info successfully!" << std::endl;
        }
        else
        {
            std::cout << "Failed to set camera info!" << std::endl;
        }

        v4r_object_recognition_msgs::recognize srv;
        srv.request.cloud = *msg;

        geometry_msgs::Vector3 t;
        geometry_msgs::Quaternion q;
        q.x = q.y = q.z = t.x = t.y = t.z = 0.;
        q.w = 1.;
        srv.request.transform.translation = t;
        srv.request.transform.rotation = q;

        if (obj_rec_client_.call(srv))
        {
            std::vector<std_msgs::String>  detected_ids = srv.response.ids;

            if(detected_ids.empty())
                std::cout << "I did not detected any object from the model database in the current scene." << std::endl;
            else {
                std::cout << "I detected: " << std::endl;
                for(const auto &id:detected_ids)
                    std::cout << "  " << id.data << std::endl;
            }
            std::cout << std::endl;
        }
        else
        {
            ROS_ERROR("Error calling recognition service. ");
            return;
        }
    }

    void checkCloudArrive (const sensor_msgs::PointCloud2::ConstPtr& msg)
    {
        KINECT_OK_ = true;
    }

    bool checkKinect ()
    {
        ros::Subscriber sub_pc = n_->subscribe (topic_, 1, &ObjectRecognizerDemo::checkCloudArrive, this);
        ros::Rate loop_rate (1);
        size_t kinect_trials_ = 0;

        std::string camera_info_topic = topic_;
        boost::replace_last( camera_info_topic, "/points", "/camera_info");
        std::cout << "Checking if there is an associated camera info topic on " << camera_info_topic << "." << std::endl;
        cam_info_sub_ = n_->subscribe(camera_info_topic, 1, &ObjectRecognizerDemo::camera_info_cb, this);

        while (!KINECT_OK_ && ros::ok () && kinect_trials_ < 30 && !camera_info_received_)
        {
            std::cout << "Checking kinect status..." << std::endl;
            ros::spinOnce ();
            loop_rate.sleep ();
            kinect_trials_++;
        }


        return KINECT_OK_;

//        bool retrain = false;
//        if(retrain) // this is necessary if the model database of the recognizer has changed
//        {
//            ros::ServiceClient retrainClient = n_->serviceClient<v4r_object_recognition_msgs::retrain_recognizer>("/recognition_service/mp_recognition_retrain");
//            v4r_object_recognition_msgs::retrain_recognizer srv;
//            for(size_t k=0; k < model_ids_to_be_loaded_.size(); k++)
//            {
//                std_msgs::String a;
//                a.data = model_ids_to_be_loaded_[k];
//                srv.request.load_ids.push_back(a);
//            }

//            if(retrainClient.call(srv))
//            {
//                std::cout << "called retrain succesfull" << std::endl;
//            }
//            else
//            {
//                ROS_ERROR("Failed to call /recognition_service/mp_recognition_retrain");
//                exit(-1);
//            }
//        }

    }

    bool callObjectRecognizerUsingFiles()
    {
        std::vector<std::string> test_cloud = v4r::io::getFilesInDirectory(directory_, ".*.pcd", false);
        for(size_t i=0; i < test_cloud.size(); i++)
        {
            pcl::PointCloud<PointT> cloud;
            pcl::io::loadPCDFile(directory_ + "/" + test_cloud[i], cloud);
            const Eigen::Quaternionf &q = cloud.sensor_orientation_;
            const Eigen::Vector4f &t = cloud.sensor_origin_;

            sensor_msgs::PointCloud2 cloud_ros;
            pcl::toROSMsg(cloud, cloud_ros);
            v4r_object_recognition_msgs::recognize srv_rec;
            srv_rec.request.cloud = cloud_ros;
            srv_rec.request.transform.translation.x = t(0);
            srv_rec.request.transform.translation.y = t(1);
            srv_rec.request.transform.translation.z = t(2);
            srv_rec.request.transform.rotation.x = q.x();
            srv_rec.request.transform.rotation.y = q.y();
            srv_rec.request.transform.rotation.z = q.z();
            srv_rec.request.transform.rotation.w = q.w();

            if (obj_rec_client_.call(srv_rec))
            {
                std::vector<std_msgs::String>  detected_ids = srv_rec.response.ids;

                if(detected_ids.empty())
                    std::cout << "I did not detected any object from the model database in the current scene." << std::endl;
                else
                {
                    std::cout << "I detected: " << std::endl;
                    for(const auto &id:detected_ids)
                        std::cout << "  " << id.data << std::endl;
                }
                std::cout << std::endl;
            }
            else
            {
                ROS_ERROR("Error calling recognition service. ");
                return false;
            }
        }
        return true;
    }

    bool initialize(int argc, char ** argv)
    {
        ros::init (argc, argv, "ObjectRecognizerDemo");
        n_ = new ros::NodeHandle ( "~" );

        obj_rec_client_ = n_->serviceClient<v4r_object_recognition_msgs::recognize>("/object_recognition/recognize");
        obj_rec_set_camera_client_ = n_->serviceClient<v4r_object_recognition_msgs::set_camera>("/object_recognition/set_camera");
        it_.reset(new image_transport::ImageTransport(*n_));
        image_pub_ = it_->advertise("/object_recognition/debug_image", 1, true);

        n_->getParam ( "input_method", input_method_ );

        if ( input_method_ == 0 )
        {
            if(!n_->getParam ( "topic", topic_ ))
            {
                topic_ = "/camera/depth_registered/points";
            }
            std::cout << "Trying to connect to camera on topic " <<
                         topic_ << ". You can change the topic with param topic or " <<
                         " test pcd files from a directory by using input_method=1 and specifying param directory. " << std::endl;

            if ( checkKinect() )
            {
                std::cout << "Camera (topic: " << topic_ << ") is up and running." << std::endl;
                ros::Subscriber sub_pc = n_->subscribe (topic_, 1, &ObjectRecognizerDemo::callObjectRecognizerUsingCam, this);
                ros::spin();
            }
            else
            {
                std::cerr << "Camera (topic: " << topic_ << ") is not working." << std::endl;
                return false;
            }
        }
        else //input_method==1
        {
            if(n_->getParam ( "test_dir", directory_ ) && directory_.length())
            {
                callObjectRecognizerUsingFiles();
            }
            else
            {
                std::cout << "No test directory (param test_dir) specified. " << std::endl;
                return false;
            }
        }
        return true;
    }
};

int
main (int argc, char ** argv)
{
    ObjectRecognizerDemo m;
    m.initialize(argc, argv);
    return 0;
}
