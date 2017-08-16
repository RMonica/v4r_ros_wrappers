#include "v4r_object_recognition_msgs/recognize.h"
#include "v4r_object_recognition_msgs/retrain_recognizer.h"
#include "v4r_object_recognition_msgs/set_camera.h"

#include <image_transport/image_transport.h>
#include <v4r/apps/ObjectRecognizer.h>
#include <v4r/recognition/object_hypothesis.h>

namespace v4r
{
template<typename PointT>
class RecognizerROS
{
private:
    boost::shared_ptr<image_transport::ImageTransport> it_;
    image_transport::Publisher image_pub_;
    boost::shared_ptr<ros::NodeHandle> n_;
    ros::Publisher vis_pc_pub_;
    ros::ServiceServer recognize_, recognizer_set_camera_;

    std::vector< ObjectHypothesesGroup > object_hypotheses_; ///< recognized objects
    typename boost::shared_ptr<v4r::apps::ObjectRecognizer<PointT> > mrec_; ///< recognizer
    typename pcl::PointCloud<PointT>::Ptr scene_; ///< input cloud
    Camera::ConstPtr camera_; ///< camera (if cloud is not organized)

    bool respondSrvCall(v4r_object_recognition_msgs::recognize::Request &req, v4r_object_recognition_msgs::recognize::Response &response) const;

public:
    RecognizerROS()
    {}

    bool recognizeROS (v4r_object_recognition_msgs::recognize::Request & req,
                       v4r_object_recognition_msgs::recognize::Response & response);

    bool setCamera (v4r_object_recognition_msgs::set_camera::Request & req,
                       v4r_object_recognition_msgs::set_camera::Response & response);

    bool initialize (int argc, char ** argv);
};

}
