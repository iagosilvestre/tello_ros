#include "mono-inertial-slam-node.hpp"
#include<opencv2/core/core.hpp>




using std::placeholders::_1;

MonoInertialSlamNode::MonoInertialSlamNode(ORB_SLAM3::System* pSLAM)
:   Node("ORB_SLAM3_ROS2")
{
    m_SLAM = pSLAM;

    // set subscriber qos profile best effort rclcpp::SensorDataQoS()
    auto qos = rclcpp::SensorDataQoS();

    m_image_subscriber = this->create_subscription<ImageMsg>(
        "/tello/camera/image_synced",
        qos,
        std::bind(&MonoInertialSlamNode::GrabImage, this, std::placeholders::_1));
    std::cout << "slam changed" << std::endl;

    m_imu_subscriber = this->create_subscription<ImuMsg>(
        "/tello/imu/data_synced",
        qos,
        std::bind(&MonoInertialSlamNode::GrabImu, this, std::placeholders::_1));

    // Create a tf broadcaster to broadcast the camera pose
    m_tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    
    // create a static tf broadcaster to broadcast the telloBase_link to camera_link
    m_static_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(*this);

    syncThread = new std::thread(&MonoInertialSlamNode::SyncWithImu, this);
}

MonoInertialSlamNode::~MonoInertialSlamNode()
{
    // Stop all threads
    m_SLAM->Shutdown();

    // Save camera trajectory
    m_SLAM->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
}

void MonoInertialSlamNode::GrabImu(const ImuMsg::SharedPtr msg)
{
    mutexImuQueue.lock();
    imu_queue.push(msg);
    mutexImuQueue.unlock();
}

void MonoInertialSlamNode::GrabImage(const ImageMsg::SharedPtr msg)
{
    mutexImageQueue.lock();
    if (!image_queue.empty()){
        image_queue.pop();
    }
    image_queue.push(msg);
    mutexImageQueue.unlock();
}

cv::Mat MonoInertialSlamNode::GetImage(const ImageMsg::SharedPtr msg){
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptr;

    try
    {
        cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::MONO8);
    }
    catch (cv_bridge::Exception &e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    }

    if (cv_ptr->image.type() == 0)
    {
        return cv_ptr->image.clone();
    }
    else
    {
        std::cerr << "Error image type" << std::endl;
        return cv_ptr->image.clone();
    }  
}



void MonoInertialSlamNode::SyncWithImu()
{
    // std::cout << "imu synchronisation thread started" << std::endl;
    while (1)
    {
        cv::Mat img;
        double tImg = 0;
        // print size of imu and image queue
        // std::cout<<"image queue size: "<<image_queue.size()<<std::endl;
        // std::cout<<"imu queue size: "<<imu_queue.size()<<std::endl;
        if (!image_queue.empty() && !imu_queue.empty())
        {
            tImg = Utility::StampToSec(image_queue.front()->header.stamp);
            

        
            mutexImageQueue.lock();
            img = GetImage(image_queue.front());
            image_queue.pop();
            mutexImageQueue.unlock();

            vector<ORB_SLAM3::IMU::Point> vImuMeas;
            mutexImuQueue.lock();
            if (!imu_queue.empty())
            {
                // Load imu measurements from buffer
                vImuMeas.clear();
                while (!imu_queue.empty() && Utility::StampToSec(imu_queue.front()->header.stamp) - this->time_shift <= tImg)
                {   
                    // print imu time stamp and image time stamp
                    // std::cout<<"imu time stamp: "<<Utility::StampToSec(imu_queue.front()->header.stamp);
                    // std::cout<<"\timage time stamp: "<<tImg<<std::endl;

                    double t = Utility::StampToSec(imu_queue.front()->header.stamp);
                    cv::Point3f acc(imu_queue.front()->linear_acceleration.x, imu_queue.front()->linear_acceleration.y, imu_queue.front()->linear_acceleration.z);
                    cv::Point3f gyr(imu_queue.front()->angular_velocity.x, imu_queue.front()->angular_velocity.y, imu_queue.front()->angular_velocity.z);
                    vImuMeas.push_back(ORB_SLAM3::IMU::Point(acc, gyr, t));
                    imu_queue.pop();
                }
            }
            mutexImuQueue.unlock();
            // prinst length of imu measurements
            // std::cout<<"imu measurements length: "<<vImuMeas.size()<<std::endl;

            // std::cout<<"one frame has been sent"<<std::endl;
            Sophus::SE3f Tcw = m_SLAM->TrackMonocular(img, tImg, vImuMeas);
            // Sophus::SE3f Tcw = m_SLAM->TrackMonocular(img, tImg);
            this->BroadcastCameraTransform(Tcw);
            
        }
        std::chrono::milliseconds tSleep(1);
        std::this_thread::sleep_for(tSleep);
    }
}

void MonoInertialSlamNode::BroadcastCameraTransform(Sophus::SE3f Tcw)
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
