/*
 * main.cpp
 *
 *  Created on: Feb 20, 2014
 *      Author: Thomas Faeulhammer
 */

#include <pcl/common/common.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_conversions/pcl_conversions.h>
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "std_msgs/String.h"
#include "std_msgs/Int32MultiArray.h"
#include "v4r_object_classification_msgs/segment_and_classify.h"
#include "v4r_object_classification_msgs/classify.h"
#include "v4r_segmentation_msgs/segment.h"
#include "geometry_msgs/Point32.h"

class SOCDemo
{
private:
    typedef pcl::PointXYZ PointT;
    int kinect_trials_;
    int service_calls_;
    std::string topic_;
    bool KINECT_OK_;
    bool all_required_services_okay_;
    ros::NodeHandle *n_;
    bool visualize_output_;
    std::vector< std_msgs::Int32MultiArray> cluster_indices_ros_;
    std::vector< std::vector< std::string > > class_results_;
    std::vector< std::vector< float > > confidences_;

    std::vector< geometry_msgs::Point32> cluster_centroids_ros_;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr scene_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr sceneXYZ_;
    std::vector<std::string> text_3d_;
    boost::shared_ptr<pcl::visualization::PCLVisualizer> vis_;
    bool do_segmentation_;
    std::string table_id_;

    void
    checkCloudArrive (const sensor_msgs::PointCloud2::ConstPtr& msg)
    {
        KINECT_OK_ = true;
    }

    void
    checkKinect ()
    {
        ros::Subscriber sub_pc = n_->subscribe (topic_, 1, &SOCDemo::checkCloudArrive, this);
        ros::Rate loop_rate (1);
        kinect_trials_ = 0;
        while (!KINECT_OK_ && ros::ok ())
        {
            std::cout << "Checking kinect status..." << std::endl;
            ros::spinOnce ();
            loop_rate.sleep ();
            kinect_trials_++;
            if(kinect_trials_ >= 30)
            {
                std::cout << "Kinect is not working..." << std::endl;
                return;
            }
        }

        KINECT_OK_ = true;
        std::cout << "Kinect is up and running" << std::endl;
    }

    bool callSegService(const sensor_msgs::PointCloud2::ConstPtr& msg)
    {
        ros::ServiceClient segmentation_client = n_->serviceClient<v4r_segmentation_msgs::segment>("/object_segmenter_service/object_segmenter");
        v4r_segmentation_msgs::segment seg_srv;
        seg_srv.request.cloud = *msg;

        if (segmentation_client.call(seg_srv))
        {
            std::cout << "Number of clusters:" << static_cast<int>(seg_srv.response.clusters_indices.size()) << std::endl;
            cluster_indices_ros_ = seg_srv.response.clusters_indices;
        }
        else
        {
            ROS_ERROR("Failed to call segmentation service.");
            return false;
        }
        return true;
    }

    bool callClassifierService(const sensor_msgs::PointCloud2::ConstPtr& msg)
    {
        ros::ServiceClient classifierClient = n_->serviceClient<v4r_object_classification_msgs::classify>("/classifier_service/classify");
        v4r_object_classification_msgs::classify srv;
        srv.request.cloud = *msg;
        srv.request.clusters_indices = cluster_indices_ros_;
        if (classifierClient.call(srv))
        {
            size_t num_clusters = srv.response.category_ids.size();
            class_results_.resize( num_clusters );
            confidences_.resize( num_clusters );
            for(size_t i=0; i<num_clusters; i++)
            {
                const v4r_object_classification_msgs::StringArray &ids = srv.response.category_ids[i];

                size_t num_classes = ids.data.size();
                class_results_[i].resize( num_classes );
                confidences_[i].resize( num_classes );
                for(size_t j=0; j<num_classes; j++)
                {
                    class_results_[i][j] = ids.data[j];
                    confidences_[i][j] = srv.response.confidences[i].data[j];
                }
            }

            cluster_indices_ros_ = srv.response.clusters_indices;
            cluster_centroids_ros_ = srv.response.centroid;
        }
        else
        {
            ROS_ERROR("Failed to call classifier service.");
            return false;
        }
        return true;
    }

    bool callSegAndClassifierService(const sensor_msgs::PointCloud2::ConstPtr& msg)
    {
        ros::ServiceClient segAndClassifierClient = n_->serviceClient<v4r_object_classification_msgs::segment_and_classify>("/classifier_service/segment_and_classify");
        v4r_object_classification_msgs::segment_and_classify srv;
        srv.request.cloud = *msg;
        if (segAndClassifierClient.call(srv))
        {
            size_t num_clusters = srv.response.category_ids.size();
            class_results_.resize( num_clusters );
            confidences_.resize( num_clusters );
            for(size_t i=0; i<num_clusters; i++)
            {
                const v4r_object_classification_msgs::StringArray &ids = srv.response.category_ids[i];

                size_t num_classes = ids.data.size();
                class_results_[i].resize( num_classes );
                confidences_[i].resize( num_classes );
                for(size_t j=0; j<num_classes; j++)
                {
                    class_results_[i][j] = ids.data[j];
                    confidences_[i][j] = srv.response.confidences[i].data[j];
                }
            }

            cluster_indices_ros_ = srv.response.clusters_indices;
        }
        else
        {
            ROS_ERROR("Failed to call segmentation_and_classifier service.");
            return false;
        }
        return true;
    }

    void
    callService (const sensor_msgs::PointCloud2::ConstPtr& msg)
    {
      std::cout << "Received point cloud.\n" << std::endl;
        // if any service is not available, wait for 5 sec and check again
        if( all_required_services_okay_ || ( !all_required_services_okay_ && (service_calls_ % (1 * 5)) == 0))
        {
            std::cout << "going to call service..." << std::endl;

            sensor_msgs::PointCloud2::Ptr input_cloud (new sensor_msgs::PointCloud2() );
            *input_cloud = *msg;

            if(do_segmentation_)
            {
                bool segServiceOkay = callSegService(input_cloud);
                bool classifierServiceOkay = callClassifierService(input_cloud);
                all_required_services_okay_ = segServiceOkay & classifierServiceOkay;
            }
            else
            {
                all_required_services_okay_ = callSegAndClassifierService(input_cloud);
            }
            pcl::fromROSMsg(*input_cloud, *scene_);
            pcl::copyPointCloud(*scene_, *sceneXYZ_);

            if (visualize_output_ && all_required_services_okay_)
            {
                visualize_output();
            }
        }
        service_calls_++;

    }
public:
    SOCDemo()
    {
        scene_.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
        sceneXYZ_.reset(new pcl::PointCloud<pcl::PointXYZ>);
        KINECT_OK_ = false;
        topic_ = "/camera/depth_registered/points";
        kinect_trials_ = 5;
        do_segmentation_ = true;
        all_required_services_okay_ = false;
    }

    bool initialize(int argc, char ** argv)
    {
        ros::init (argc, argv, "classifier_demo");
        if (sizeof(int) != 4)
        {
            ROS_WARN("PC Architectur does not use 32bit for integer - check conflicts with pcl indices.");
        }
        n_ = new ros::NodeHandle ( "~" );

        if(!n_->getParam ( "topic", topic_ ))
            topic_ = "/camera/depth_registered/points";

        if(!n_->getParam ( "visualize_output", visualize_output_ ))
            visualize_output_ = false;

        if(!n_->getParam ( "table_id", table_id_ ))
            table_id_ = "";

        checkKinect();
        return KINECT_OK_;
    }

    void run()
    {
        ros::Subscriber sub_pc = n_->subscribe (topic_, 1, &SOCDemo::callService, this);
        ros::spin();
        /*ros::Rate loop_rate (5);
      while (ros::ok () && (service_calls_ < 5)) //only calls 5 times
      {
        ros::spinOnce ();
        loop_rate.sleep ();
      }*/
    }

    void visualize_output()
    {
        int detected_classes = 0;
        if(!vis_)
        {
            vis_.reset(new pcl::visualization::PCLVisualizer("classifier visualization"));
        }

        float text_scale = 0.010f;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr pClassifiedPCl (new pcl::PointCloud<pcl::PointXYZRGB>());
        pcl::copyPointCloud(*sceneXYZ_, *pClassifiedPCl);
        vis_->removeAllPointClouds();

        text_3d_.clear();
        vis_->removeAllShapes();
        vis_->addPointCloud(sceneXYZ_, "scene_cloud");

        for(size_t i=0; i < cluster_indices_ros_.size(); i++)
        {
            float r = std::rand() % 255;
            float g = std::rand() % 255;
            float b = std::rand() % 255;
            for(size_t kk=0; kk < cluster_indices_ros_[i].data.size(); kk++)
            {
                pClassifiedPCl->at(cluster_indices_ros_[i].data[kk]).r = r;
                pClassifiedPCl->at(cluster_indices_ros_[i].data[kk]).g = g;
                pClassifiedPCl->at(cluster_indices_ros_[i].data[kk]).b = b;
            }
            std::stringstream cluster_name;
            if( !class_results_[i].empty() )
            {
                cluster_name << "#" << i << ": " << class_results_[i][0];
                std::cout << "Cluster " << i << ": " << std::endl;
                for (size_t kk = 0; kk < class_results_[i].size(); kk++)
                {
                    std::cout << class_results_[i][kk] <<
                                 " [" << confidences_[i][kk] << "]" << std::endl;

                    std::stringstream prob_str;
                    prob_str.precision (2);
                    prob_str << class_results_[i][kk] << " [" << confidences_[i][kk] << "]";

                    std::stringstream cluster_text;
                    cluster_text << "cluster_" << i << "_" << kk << "_text";
                    text_3d_.push_back(cluster_text.str());
                    vis_->addText(prob_str.str(), 10+150*detected_classes, 10+kk*25,
                                  17, r/255.f, g/255.f, b/255.f, cluster_text.str());
                }
                pcl::PointXYZ pos;
                pos.x = cluster_centroids_ros_[i].x;
                pos.y = cluster_centroids_ros_[i].y;
                pos.z = cluster_centroids_ros_[i].z;
                text_3d_.push_back(cluster_name.str());
                vis_->addText3D(cluster_name.str(), pos, text_scale, 0.8*r/255.f, 0.8*g/255.f, 0.8*b/255.f, cluster_name.str(), 0);
                detected_classes++;
            }
            std::cout << std::endl;
        }
        pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb_handler (pClassifiedPCl);
        vis_->addPointCloud<pcl::PointXYZRGB> (pClassifiedPCl, rgb_handler, "classified_pcl");
        vis_->spin();
    }
};

int
main (int argc, char ** argv)
{
    SOCDemo m;
    m.initialize (argc, argv);
    m.run();
    return 0;
}
