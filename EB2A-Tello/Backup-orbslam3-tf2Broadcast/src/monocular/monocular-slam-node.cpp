#include "monocular-slam-node.hpp"

#include<opencv2/core/core.hpp>




using std::placeholders::_1;

MonocularSlamNode::MonocularSlamNode(ORB_SLAM3::System* pSLAM)
:   Node("ORB_SLAM3_ROS2")
{
    m_SLAM = pSLAM;
    // std::cout << "slam changed" << std::endl;
    m_image_subscriber = this->create_subscription<ImageMsg>(
        "/drone1/image_raw",
        10,
        std::bind(&MonocularSlamNode::GrabImage, this, std::placeholders::_1));
    std::cout << "slam changed" << std::endl;

    // Create a tf broadcaster to broadcast the camera pose
    m_tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    
    // create a static tf broadcaster to broadcast the telloBase_link to camera_link
    m_static_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(*this);
}

MonocularSlamNode::~MonocularSlamNode()
{
    // Stop all threads
    m_SLAM->Shutdown();

    // Save camera trajectory
    m_SLAM->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
}

void MonocularSlamNode::GrabImage(const ImageMsg::SharedPtr msg)
{
    // Copy the ros image message to cv::Mat.
    try
    {
        m_cvImPtr = cv_bridge::toCvCopy(msg);
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    // std::cout<<"one frame has been sent"<<std::endl;
    // m_SLAM->TrackMonocular(m_cvImPtr->image, Utility::StampToSec(msg->header.stamp));
    // ------------------test------------------
    Sophus::SE3f Tcw = m_SLAM->TrackMonocular(m_cvImPtr->image, Utility::StampToSec(msg->header.stamp));
    this->BroadcastCameraTransform(Tcw);
}

void MonocularSlamNode::BroadcastCameraTransform(Sophus::SE3f Tcw)
{
    Sophus::SE3f Twc = Tcw.inverse();

    Eigen::Vector3f t = Twc.translation();
    Eigen::Quaternionf q(Twc.rotationMatrix());

    // Rotate the camera -90 degrees around the x axis, 90 degrees around the y axis
    Eigen::Quaternionf q_cam_rot = Eigen::AngleAxisf(-M_PI/2, Eigen::Vector3f::UnitX()) * Eigen::AngleAxisf(M_PI/2, Eigen::Vector3f::UnitY()) * Eigen::Quaternionf::Identity();

    // Rotate the translation vector
    t = q_cam_rot * t;

    q_cam_rot = q_cam_rot * q;

    // Create a transform from world to camera
    geometry_msgs::msg::TransformStamped transform_stamped;
    transform_stamped.header.stamp = rclcpp::Clock().now();
    transform_stamped.header.frame_id = "world";
    transform_stamped.child_frame_id = "telloCamera";
    transform_stamped.transform.translation.x = t.x();
    transform_stamped.transform.translation.y = t.y();
    transform_stamped.transform.translation.z = t.z();
    transform_stamped.transform.rotation.x = q_cam_rot.x();
    transform_stamped.transform.rotation.y = q_cam_rot.y();
    transform_stamped.transform.rotation.z = q_cam_rot.z();
    transform_stamped.transform.rotation.w = q_cam_rot.w();


    // Rotate the IMU -90 degrees around the y axis relative to the camera
    Eigen::Quaternionf q_imu_rot = Eigen::AngleAxisf(-M_PI/2, Eigen::Vector3f::UnitY()) * Eigen::Quaternionf::Identity();


    // Create a static transform from telloCamera to telloIMU
    geometry_msgs::msg::TransformStamped static_transform_stamped;
    static_transform_stamped.header.stamp = rclcpp::Clock().now();
    static_transform_stamped.header.frame_id = "telloCamera";
    static_transform_stamped.child_frame_id = "tellIMU";
    static_transform_stamped.transform.translation.x = 0.0;
    static_transform_stamped.transform.translation.y = -0.0028;
    static_transform_stamped.transform.translation.z = -0.043;
    static_transform_stamped.transform.rotation.x = q_imu_rot.x();
    static_transform_stamped.transform.rotation.y = q_imu_rot.y();
    static_transform_stamped.transform.rotation.z = q_imu_rot.z();
    static_transform_stamped.transform.rotation.w = q_imu_rot.w();

    // Send the transform
    m_tf_broadcaster_->sendTransform(transform_stamped);
    
    // Send the static transform
    m_static_tf_broadcaster_->sendTransform(static_transform_stamped);
}
