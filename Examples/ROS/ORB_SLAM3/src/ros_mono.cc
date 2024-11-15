#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <vector>
#include <queue>
#include <thread>
#include <mutex>

#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <opencv2/core/core.hpp>

#include "../../../../include/System.h"

#include <functional> // For std::function

using namespace std;

class ImageGrabber : public rclcpp::Node
{
public:
    ImageGrabber(ORB_SLAM3::System *pSLAM, const bool bClahe)
        : Node("image_grabber"), mpSLAM(pSLAM), mbClahe(bClahe)
    {
        img_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/image_raw",
            rclcpp::QoS(10),
            std::bind(&ImageGrabber::GrabImage, this, std::placeholders::_1));

        // Initialize the pose publisher
        // pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("camera_pose", 10);
    }

    void GrabImage(const sensor_msgs::msg::Image::SharedPtr img_msg)
    { // Convert ROS Image message to cv::Mat
        RCLCPP_INFO(this->get_logger(), "Grab image callback called!");
        cv::Mat im = GetImage(img_msg);

        RCLCPP_INFO(this->get_logger(), "GOT IMAGE");

        if (im.empty())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to convert image");
            return;
        }

        // Apply CLAHE if enabled
        if (mbClahe)
            mClahe->apply(im, im);

        // Get timestamp
        double tIm = img_msg->header.stamp.sec +
                     img_msg->header.stamp.nanosec * 1e-9;

        // Pass the image to the SLAM system and get the pose
        Sophus::SE3f pose = mpSLAM->TrackMonocular(im, tIm);

        RCLCPP_INFO(this->get_logger(), "Tracked");

        // Extract the translation (position) and rotation (orientation)
        Eigen::Vector3f position = pose.translation();
        Eigen::Quaternionf orientation = pose.unit_quaternion();

        // Create a PoseStamped message
        geometry_msgs::msg::PoseStamped pose_msg;
        pose_msg.header = img_msg->header; // Use the timestamp and frame ID from the image
        pose_msg.pose.position.x = position.x();
        pose_msg.pose.position.y = position.y();
        pose_msg.pose.position.z = position.z();

        pose_msg.pose.orientation.x = orientation.x();
        pose_msg.pose.orientation.y = orientation.y();
        pose_msg.pose.orientation.z = orientation.z();
        pose_msg.pose.orientation.w = orientation.w();

        // Publish the pose
        pose_publisher_->publish(pose_msg);
    }

    cv::Mat GetImage(const sensor_msgs::msg::Image::SharedPtr img_msg)
    {
        cv_bridge::CvImageConstPtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvShare(img_msg, "mono8");
        }
        catch (cv_bridge::Exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return cv::Mat();
        }

        if (cv_ptr->image.type() == CV_8UC1) // Ensure image is grayscale
        {
            return cv_ptr->image.clone();
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Image is not grayscale");
            return cv::Mat();
        }
    }

private:
    ORB_SLAM3::System *mpSLAM;
    const bool mbClahe;
    cv::Ptr<cv::CLAHE> mClahe = cv::createCLAHE(3.0, cv::Size(8, 8));

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_subscription_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    if (argc < 3 || argc > 4)
    {
        std::cerr << "\nUsage: run_monocular path_to_vocabulary path_to_settings [do_equalize]" << std::endl;
        return 1;
    }

    bool bEqual = false;
    if (argc == 4)
    {
        std::string sbEqual(argv[3]);
        if (sbEqual == "true")
            bEqual = true;
    }

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::MONOCULAR, true);

    auto image_grabber = std::make_shared<ImageGrabber>(&SLAM, bEqual);

    // Spin the node
    rclcpp::spin(image_grabber);

    // Shutdown the SLAM system
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    rclcpp::shutdown();

    return 0;
}