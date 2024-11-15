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
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <opencv2/core/core.hpp>

#include "../../../../include/System.h"
#include "../../../../include/ImuTypes.h"

#include <functional> // For std::function

using namespace std;

class ImuGrabber : public rclcpp::Node
{
public:
    ImuGrabber() : Node("imu_grabber")
    {
        imu_subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu", 1000, std::bind(&ImuGrabber::GrabImu, this, std::placeholders::_1));
    }

    void GrabImu(const sensor_msgs::msg::Imu::SharedPtr imu_msg)
    {
	// RCLCPP_INFO(this->get_logger(), "getting imu data");
        std::lock_guard<std::mutex> lock(mBufMutex);
        imuBuf.push(imu_msg);
    }

    queue<sensor_msgs::msg::Imu::SharedPtr> imuBuf;
    std::mutex mBufMutex;

private:
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
};

class ImageGrabber : public rclcpp::Node
{
public:
    ImageGrabber(ORB_SLAM3::System* pSLAM, ImuGrabber *pImuGb, const bool bClahe)
    : Node("image_grabber"), mpSLAM(pSLAM), mpImuGb(pImuGb), mbClahe(bClahe)
    {
        img_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/image_raw",
            100,
            std::function<void(sensor_msgs::msg::Image::SharedPtr)>(
                std::bind(&ImageGrabber::GrabImage, this, std::placeholders::_1)
            )
        );
    }

    void GrabImage(const sensor_msgs::msg::Image::SharedPtr& img_msg)
    {
        std::lock_guard<std::mutex> lock(mBufMutex);
        if (!img0Buf.empty())
            img0Buf.pop();
        img0Buf.push(img_msg);
    }

    cv::Mat GetImage(const sensor_msgs::msg::Image::SharedPtr& img_msg)
    {
        cv_bridge::CvImageConstPtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvShare(img_msg, "mono8");
        }
        catch (cv_bridge::Exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        }
  
        if(cv_ptr->image.type() == 0)
        {
            return cv_ptr->image.clone();
        }
        else
        {
            std::cout << "Error type" << std::endl;
            return cv_ptr->image.clone();
        }
    }

    void SyncWithImu()
    {
        while (rclcpp::ok())
        {
            cv::Mat im;
            double tIm = 0;
            if (!img0Buf.empty() && !mpImuGb->imuBuf.empty())
            {
                tIm = img0Buf.front()->header.stamp.sec +
                      img0Buf.front()->header.stamp.nanosec * 1e-9;
                if (tIm > mpImuGb->imuBuf.back()->header.stamp.sec +
                           mpImuGb->imuBuf.back()->header.stamp.nanosec * 1e-9)
                    continue;

                {
                    std::lock_guard<std::mutex> lock(this->mBufMutex);
                    im = GetImage(img0Buf.front());
                    img0Buf.pop();
                }

                vector<ORB_SLAM3::IMU::Point> vImuMeas;
                {
                    std::lock_guard<std::mutex> lock(mpImuGb->mBufMutex);
                    if (!mpImuGb->imuBuf.empty())
                    {
                        vImuMeas.clear();
                        while (!mpImuGb->imuBuf.empty() &&
                               mpImuGb->imuBuf.front()->header.stamp.sec +
                                       mpImuGb->imuBuf.front()->header.stamp.nanosec * 1e-9 <= tIm)
                        {
                            double t = mpImuGb->imuBuf.front()->header.stamp.sec +
                                       mpImuGb->imuBuf.front()->header.stamp.nanosec * 1e-9;
                            cv::Point3f acc(mpImuGb->imuBuf.front()->linear_acceleration.x, mpImuGb->imuBuf.front()->linear_acceleration.y, mpImuGb->imuBuf.front()->linear_acceleration.z);
                            cv::Point3f gyr(mpImuGb->imuBuf.front()->angular_velocity.x, mpImuGb->imuBuf.front()->angular_velocity.y, mpImuGb->imuBuf.front()->angular_velocity.z);
                            vImuMeas.push_back(ORB_SLAM3::IMU::Point(acc,gyr,t));
                            mpImuGb->imuBuf.pop();
                        }
                    }
                }
                if (mbClahe)
                    mClahe->apply(im, im);

                mpSLAM->TrackMonocular(im, tIm, vImuMeas);
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }

    queue<sensor_msgs::msg::Image::SharedPtr> img0Buf;
    std::mutex mBufMutex;

    ORB_SLAM3::System* mpSLAM;
    ImuGrabber *mpImuGb;

    const bool mbClahe;
    cv::Ptr<cv::CLAHE> mClahe = cv::createCLAHE(3.0, cv::Size(8, 8));

private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_subscription_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    if (argc < 3 || argc > 4)
    {
        std::cerr << std::endl << "Usage: run_monocular path_to_vocabulary path_to_settings [do_equalize]" << std::endl;
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
    ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::IMU_MONOCULAR, false);

    auto imu_grabber = std::make_shared<ImuGrabber>();
    auto image_grabber = std::make_shared<ImageGrabber>(&SLAM, imu_grabber.get(), bEqual);

    std::thread sync_thread(&ImageGrabber::SyncWithImu, image_grabber);

    // Create an executor and add both nodes
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(imu_grabber);
    executor.add_node(image_grabber);
    executor.spin();

    rclcpp::shutdown();
    sync_thread.join();

    return 0;
}

