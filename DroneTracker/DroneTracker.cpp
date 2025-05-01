#include "DroneTracker.hpp"

#include <_exp/OpenCV/Contour.hpp>

#include <Math/AngularConv.hpp>

#include <utility>

namespace fatsim
{
    DroneTracker::DroneTracker(
        std::vector<ImgRequest_t> imgRequests,
        bool                      captureExternalCamera,
        const std::string&        droneRouterSubAddress,
        const std::string&        unrealEnginePubAddress)
        :
        m_zmq_subscriber_(droneRouterSubAddress),
        m_zmq_publisher_(unrealEnginePubAddress),
        m_img_requests_(imgRequests),
        mc_from_external_camera_(captureExternalCamera)
    {
        m_airlib_client_.confirmConnection();

        std::println<>("Starting DroneTracker...");

        try
        {
            m_depth_camera_info_ = m_airlib_client_.simGetCameraInfo(imgRequests[0].camera_name, "", mc_from_external_camera_);

            std::println(
                "Depth Camera Info Acquired: Pos({:.2f},{:.2f},{:.2f}), FOV: {:.1f}",
                m_depth_camera_info_.pose.position.x(), m_depth_camera_info_.pose.position.y(), m_depth_camera_info_.pose.position.z(),
                m_depth_camera_info_.fov
            );
        }
        catch (const std::exception& ex)
        {
            std::println(stderr, "Failed to get camera info: {}", ex.what());
            std::println<>("DroneTracker could NOT start!");

            m_finished_ = true;

            throw;
        }
    }

    void DroneTracker::Run()
    {
        std::println<>("DroneTracker has started...");
        std::println<>("Capturing first frame...");
        CaptureFrame_();
        std::println<>("Captured first frame...");

        while (not m_finished_)
        {
            if (not ReceivedContinueMsg_())
            {
                goto wait;
            }

            try
            {
                CaptureFrame_();
            }
            catch (const std::exception& ex)
            {
                std::println<>("{}", ex.what());

                goto wait;
            }

            DetectAndPublishDronePosition_();

            if (cv::waitKey(1) == 27)
            {
                break;
            }
            else
            {
                continue;
            }

        wait:
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }

        cv::destroyAllWindows();
        std::println<>("DroneTracker has finished.");
    }

    auto DroneTracker::ReceivedContinueMsg_() -> bool
    {
        if (const auto& msg = m_zmq_subscriber_.Receive(); not msg.empty())
        {
            std::println<>("Message received: {}", msg);

            if (msg == "FATSIM_SIMULATION_ENDED")
            {
                m_finished_ = true;
            }
            else if (msg == "FATSIM_TRACKER_EXITING")
            {
                m_finished_ = true;
            }

            return not m_finished_ and (msg == "FATSIM_DRONE_MOVING" or msg == "FATSIM_DRONE_STOPPED");
        }

        return false;
    }
    auto DroneTracker::DroneDetected_() -> bool
    {
        if (m_largest_contour_idx_ not_eq -1)
        {
            if (const auto& drone_moments = cv::moments(m_contours_[m_largest_contour_idx_]); drone_moments.m00 > 0)
            {
                m_drone_center_.x = static_cast<int>(drone_moments.m10 / drone_moments.m00);
                m_drone_center_.y = static_cast<int>(drone_moments.m01 / drone_moments.m00);
                
                return true;
            }
        }

        return false;
    }

    void DroneTracker::CaptureFrame_()
    {
        const auto& responses = m_airlib_client_.simGetImages(m_img_requests_, "", mc_from_external_camera_);

        if (responses.size() < m_img_requests_.size())
        {
            throw std::runtime_error("Could NOT receive responses for all image requests!");
        }

        for (const auto& response : responses)
        {
            switch (response.image_type)
            {
            case msr::airlib::ImageCaptureBase::ImageType::Segmentation:
            {
                if (response.image_data_uint8.empty() or response.height == 0 or response.width == 0)
                {
                    m_segmentation_frame_.release();
                    std::println(stderr, "Failed to capture valid Segmentation data!");
                }
                else
                {
                    m_segmentation_frame_ = cv::Mat(response.height, response.width, CV_8UC3, static_cast<void*>(const_cast<uchar*>(response.image_data_uint8.data()))).clone();
                }
            }
                break;

            case msr::airlib::ImageCaptureBase::ImageType::DepthPerspective:
            {
                if (response.image_data_float.empty() or response.height == 0 or response.width == 0)
                {
                    m_depth_frame_.release();
                    std::println(stderr, "Failed to capture valid DepthPerspective data!");
                }
                else
                {
                    m_depth_frame_ = cv::Mat(response.height, response.width, CV_32FC1, static_cast<void*>(const_cast<float*>(response.image_data_float.data()))).clone();
                }
            }
                break;

            default:
                std::println(stderr, "Warning: Failed to capture either Segmentation or DepthPerspective (or both)!");
                break;
            }
        }
    }
    void DroneTracker::ProcessSegmentationImage_()
    {
        cv::inRange(m_segmentation_frame_, s_drone_bgr_values_, s_drone_bgr_values_, m_masked_segmentation_frame_);
        cv::erode(m_masked_segmentation_frame_,  m_masked_segmentation_frame_, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3)));
        cv::dilate(m_masked_segmentation_frame_, m_masked_segmentation_frame_, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(6, 6)));
        DisplayMaskedSegmentationFrame_();
        cv::findContours(m_masked_segmentation_frame_, m_contours_, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        m_largest_contour_idx_ = fatx::opencv::FindLargestContour_(m_contours_, 0.1);
    }

    void DroneTracker::MarkDrone_() const
    {
        cv::circle(m_segmentation_frame_, m_drone_center_, 15, cv::Scalar(0, 255, 0), 2);

        cv::line(m_segmentation_frame_, { m_drone_center_.x - 10, m_drone_center_.y      }, { m_drone_center_.x + 10, m_drone_center_.y }, cv::Scalar(0, 255, 0), 1);
        cv::line(m_segmentation_frame_, { m_drone_center_.x,      m_drone_center_.y - 10 }, { m_drone_center_.x, m_drone_center_.y + 10 }, cv::Scalar(0, 255, 0), 1);

        // cv::drawContours(m_segmentation_frame_, m_contours_, m_largest_contour_idx_, cv::Scalar(0, 0, 255), 2);
    }
    void DroneTracker::DisplaySegmentationFrame_() const
    {
        cv::imshow("Segmentation Frame", m_segmentation_frame_);
        std::println<>("displayed: Segmentation Frame");
    }
    void DroneTracker::DisplayDepthFrame_() const
    {
        cv::imshow("Depth Frame", m_depth_frame_);
        std::println<>("displayed: Depth Frame");
    }
    void DroneTracker::DisplayMaskedSegmentationFrame_() const
    {
        cv::imshow("Masked Segmentation Frame", m_masked_segmentation_frame_);
        std::println<>("displayed: Masked Segmentation Frame");
    }
    void DroneTracker::DetectAndPublishDronePosition_()
    {
        if (m_segmentation_frame_.empty())
        {
            m_zmq_publisher_.Publish("FATSIM_DRONE_NOT_FOUND");

            return;
        }

        ProcessSegmentationImage_();

        if (DroneDetected_())
        {
            MarkDrone_();

            auto depth_m = -1.0;

            if (m_drone_center_.y >= 0 and m_drone_center_.y < m_depth_frame_.rows and m_drone_center_.x >= 0 and m_drone_center_.x < m_depth_frame_.cols)
            {
                depth_m = m_depth_frame_.at<float>(m_drone_center_.y, m_drone_center_.x);
            }

            if (depth_m < 0.1F or depth_m > 5000.0F or std::isnan(depth_m) or std::isinf(depth_m))
            {
                m_zmq_publisher_.Publish("FATSIM_DRONE_NOT_FOUND (Invalid/OutOfRange Depth)");
                std::println("Invalid or Out-of-Range depth: {:.2f}", depth_m);

                goto show_and_reset;
            }
            else
            {
                //
                // World Coordinate Calculation
                //
                const auto& fovRad    = fatpound::math::DegToRad<>(m_depth_camera_info_.fov);
                const auto& imgWidth  = m_segmentation_frame_.cols;
                const auto& imgHeight = m_segmentation_frame_.rows;
                const auto& fx = imgWidth / (2.0F * std::tan(fovRad / 2.0F));
                const auto& fy = fx; // fx=fy ?
                const auto& cx = imgWidth  / 2.0F;
                const auto& cy = imgHeight / 2.0F;

                const auto& camX = depth_m;
                const auto& camY = (m_drone_center_.x - cx) * camX / fx;
                const auto& camZ = (m_drone_center_.y - cy) * camX / fy;
                const auto& posInCamFrame = msr::airlib::Vector3r(static_cast<float>(camX), static_cast<float>(camY), static_cast<float>(camZ));

                const auto& camOrientation = m_depth_camera_info_.pose.orientation;
                const auto& camPos         = m_depth_camera_info_.pose.position;
                const auto& rotatedPos     = camOrientation * posInCamFrame;
                const auto& droneWorldPos  = camPos + rotatedPos;

                const auto& pos = droneWorldPos;
                const auto& msg = std::format("WPOS:{:.2f},{:.2f},{:.2f}", pos.x(), pos.y(), pos.z());
                m_zmq_publisher_.Publish(msg);
                std::println("Published world position: {}", msg);
            }
        }
        else
        {
            m_zmq_publisher_.Publish("FATSIM_DRONE_NOT_FOUND");
            std::println<>("FATSIM_DRONE_NOT_FOUND");
        }


show_and_reset:
        // DisplayDepthFrame_();
        DisplaySegmentationFrame_();

        Reset_();
    }
    void DroneTracker::Reset_()
    {
        m_largest_contour_idx_ = -1;
        m_contours_.clear();
        m_segmentation_frame_.release();
        m_depth_frame_.release();
        m_masked_segmentation_frame_.release();
        m_drone_center_ = cv::Point(-1, -1);
    }
}