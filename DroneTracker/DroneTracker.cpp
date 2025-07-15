#include "DroneTracker.hpp"

#include <_exp/OpenCV/Contour.hpp>

#include <Math/Geometry/include/AngularConv.hpp>

#include <cmath>

#include <utility>
#include <chrono>
#include <array>
#include <format>
#include <print>
#include <stdexcept>
#include <thread>

namespace fatsim
{
    DroneTracker::DroneTracker(
        std::vector<ImgRequest_t> imgRequests,
        const bool                captureExternalCamera,
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

            std::println<>(
                "Depth Camera Info Acquired: Pos({:.2f},{:.2f},{:.2f}), FOV: {:.1f}",
                m_depth_camera_info_.pose.position.x(), m_depth_camera_info_.pose.position.y(), m_depth_camera_info_.pose.position.z(),
                m_depth_camera_info_.fov
            );
        }
        catch (const std::exception& ex)
        {
            std::println<>(stderr, "Failed to get camera info: {}", ex.what());
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
            if (const auto& drone_moments = cv::moments(m_contours_[static_cast<std::size_t>(m_largest_contour_idx_)]); drone_moments.m00 > 0)
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
            using enum msr::airlib::ImageCaptureBase::ImageType;

            switch (response.image_type)
            {
            case Segmentation:
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

            case DepthPerspective:
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
        // cv::erode(m_masked_segmentation_frame_,  m_masked_segmentation_frame_, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3)));
        // cv::dilate(m_masked_segmentation_frame_, m_masked_segmentation_frame_, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(6, 6)));
        ShowMaskedSegmentationFrame_();
        cv::findContours(m_masked_segmentation_frame_, m_contours_, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        m_largest_contour_idx_ = fatx::opencv::FindLargestContour(m_contours_, 0.1);
    }

    void DroneTracker::MarkDrone_() const
    {
        cv::circle(m_segmentation_frame_, m_drone_center_, 15, cv::Scalar(0, 255, 0), 2);

        cv::line(m_segmentation_frame_, { m_drone_center_.x - 10, m_drone_center_.y      }, { m_drone_center_.x + 10, m_drone_center_.y }, cv::Scalar(0, 255, 0), 1);
        cv::line(m_segmentation_frame_, { m_drone_center_.x,      m_drone_center_.y - 10 }, { m_drone_center_.x, m_drone_center_.y + 10 }, cv::Scalar(0, 255, 0), 1);

        // cv::drawContours(m_segmentation_frame_, m_contours_, m_largest_contour_idx_, cv::Scalar(0, 0, 255), 2);
    }
    void DroneTracker::ShowSegmentationFrame_() const
    {
        cv::imshow("Segmentation Frame", m_segmentation_frame_);
        std::println<>("displayed: Segmentation Frame");
    }
    void DroneTracker::ShowDepthFrame_() const
    {
        cv::imshow("Depth Frame", m_depth_frame_);
        std::println<>("displayed: Depth Frame");
    }
    void DroneTracker::ShowMaskedSegmentationFrame_() const
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

            auto depth_m = -1.0F;

            if (    m_drone_center_.y >= 0
                and m_drone_center_.y < m_depth_frame_.rows
                and m_drone_center_.x >= 0
                and m_drone_center_.x < m_depth_frame_.cols)
            {
                depth_m = m_depth_frame_.at<float>(m_drone_center_.y, m_drone_center_.x);
            }

            if (depth_m < 0.1F or depth_m > 5000.0F or std::isnan(depth_m) or std::isinf(depth_m))
            {
                m_zmq_publisher_.Publish("FATSIM_DRONE_NOT_FOUND (Invalid/OutOfRange Depth)");
                std::println<>("Invalid or Out-of-Range depth: {:.2f}", depth_m);

                goto display_detection;
            }
            else
            {
                using fatpound::math::geometry::DegToRad;
                using fatpound::math::geometry::RadToDeg;

                const auto& fov_rad_horizontal = DegToRad<>(m_depth_camera_info_.fov);
                const auto& imgWidth           = m_segmentation_frame_.cols;
                const auto& imgHeight          = m_segmentation_frame_.rows;
                const auto& aspect_ratio       = static_cast<float>(imgWidth) / static_cast<float>(imgHeight);
                const auto& fov_rad_vertical   = 2.0F * std::atan(std::tan(fov_rad_horizontal / 2.0F) / aspect_ratio);
                const auto& fx                 = static_cast<float>(imgWidth)  / (2.0F * std::tan(fov_rad_horizontal / 2.0F));
                const auto& fy                 = static_cast<float>(imgHeight) / (2.0F * std::tan(fov_rad_vertical / 2.0F));
                const auto& cx                 = static_cast<float>(imgWidth)  / 2.0F;
                const auto& cy                 = static_cast<float>(imgHeight) / 2.0F;

                // Pixel to Camera Coordinates (AirSim: +X Fwd, +Y Right, +Z Down)
                const auto& camX = depth_m;
                const auto& camY = (static_cast<float>(m_drone_center_.x) - cx) * camX / fx; // Horizontal Diff
                const auto& camZ = (static_cast<float>(m_drone_center_.y) - cy) * camX / fy; //   Vertical Diff

                // Yaw
                const auto& targetYaw_rad = std::atan2(static_cast<float>(camY), static_cast<float>(camX));
                const auto& targetYaw_deg = RadToDeg<>(targetYaw_rad);

                // Pitch
                const auto& targetPitch_rad = std::atan2(static_cast<float>(camZ), static_cast<float>(camX));

                const auto& basePitch_deg = RadToDeg<>(targetPitch_rad);

                auto pitch_boost_deg = 0.0f;

                if (camZ < 0.0f)
                {
                    const auto& dz_norm = (static_cast<float>(m_drone_center_.y) - cy) / static_cast<float>(imgHeight); // normalize [-1, +1]
                    const auto& steepness = 15.0f;
                    const auto& boost_factor = std::tanh(steepness * dz_norm);

                    pitch_boost_deg = boost_factor * 15.0f;
                }

                const auto& targetPitch_deg = basePitch_deg + pitch_boost_deg;

                const auto& msg = std::format<>("ANGLE:{:.2f},{:.2f}", targetYaw_deg, targetPitch_deg);
                m_zmq_publisher_.Publish(msg);
                std::println("Published angles: Yaw={:.2f}, Pitch={:.2f}", targetYaw_deg, targetPitch_deg);
            }
        }
        else
        {
            m_zmq_publisher_.Publish("FATSIM_DRONE_NOT_FOUND");
            std::println<>("FATSIM_DRONE_NOT_FOUND");
        }


    display_detection:
        // ShowDepthFrame_();
        ShowSegmentationFrame_();

        Reset_();
    }
    void DroneTracker::Reset_()
    {
        m_largest_contour_idx_ = -1;
        m_contours_.clear();
        m_drone_center_ = cv::Point(-1, -1);
    }
}