#pragma once

#include <v4r/object_modelling/incremental_object_learning.h>

#include <ros/ros.h>
#include <geometry_msgs/Transform.h>

#include "v4r_incremental_object_learning_msgs/clear.h"
#include "v4r_incremental_object_learning_msgs/learn_object.h"
#include "v4r_incremental_object_learning_msgs/learn_object_inc.h"
#include "v4r_incremental_object_learning_msgs/save_model.h"
#include "v4r_incremental_object_learning_msgs/visualize.h"

namespace v4r
{
namespace object_modelling
{
class IOL_ROS : public IOL
{
private:
    boost::shared_ptr<ros::NodeHandle> n_;
    ros::ServiceServer clear_cached_model_,
                       learn_object_,
                       learn_object_inc_,
                       save_model_,
                       vis_model_,
                       write_images_to_disk_srv_;
    ros::Publisher vis_pc_pub_;

    bool visualize_intermediate_results_;

public:
    void initSIFT (int argc, char ** argv);

    bool clear_cached_model (v4r_incremental_object_learning_msgs::clear::Request & req,
                     v4r_incremental_object_learning_msgs::clear::Response & response);

    bool save_model (v4r_incremental_object_learning_msgs::save_model::Request & req,
                     v4r_incremental_object_learning_msgs::save_model::Response & response);

    bool visualizeROS(v4r_incremental_object_learning_msgs::visualize::Request & req,
                        v4r_incremental_object_learning_msgs::visualize::Response & response);

    bool learn_object (v4r_incremental_object_learning_msgs::learn_object::Request & req,
                       v4r_incremental_object_learning_msgs::learn_object::Response & response);

    bool learn_object_inc (v4r_incremental_object_learning_msgs::learn_object_inc::Request & req,
                           v4r_incremental_object_learning_msgs::learn_object_inc::Response & response);

    static Eigen::Matrix4f fromGMTransform(const geometry_msgs::Transform & gm_trans)
    {
        Eigen::Matrix4f trans = Eigen::Matrix4f::Identity();

        Eigen::Quaternionf q(gm_trans.rotation.w,
                             gm_trans.rotation.x,
                             gm_trans.rotation.y,
                             gm_trans.rotation.z);

        Eigen::Vector3f translation(gm_trans.translation.x,
                                    gm_trans.translation.y,
                                    gm_trans.translation.z);


        trans.block<3,3>(0,0) = q.toRotationMatrix();
        trans.block<3,1>(0,3) = translation;
        return trans;
    }
};


}
}
