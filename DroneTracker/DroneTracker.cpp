#include "DroneTracker.hpp"

#include <utility>

namespace fatsim
{
    DroneTracker::DroneTracker(
        std::vector<ImgRequest_t> imgRequests,
        bool                      fromExternalCamera,
        const std::string&        routerAddress,
        const std::string&        unrealAddress)
        :
        m_img_requests_(std::move<>(imgRequests)),
        m_zmq_subscriber_(routerAddress),
        m_zmq_publisher_(unrealAddress),
        mc_from_external_camera_(fromExternalCamera)
    {
        m_drone_client_.confirmConnection();

        std::println<>("Starting DroneTracker...");
    }

    void DroneTracker::Run()
    {
        CaptureFrame_();

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
                std::println("{}", ex.what());

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
        std::println<>("DroneTracker finished.");
    }
    
    auto DroneTracker::GetDilatedThresholdImg_() -> cv::Mat
    {
        cv::Mat threshDilated;

        {
            cv::Mat thresh;

            cv::threshold(m_segmentation_frame_, thresh, 30, 255, cv::THRESH_BINARY);
            cv::dilate(thresh, threshDilated, cv::Mat(), cv::Point(-1, -1), 3);
        }

        return threshDilated;
    }

    auto DroneTracker::ReceivedContinueMsg_() -> bool
    {
        if (const auto& msg = m_zmq_subscriber_.Receive(); not msg.empty())
        {
            std::println<>("Message received: {}", msg);

            if (msg == "FATSIM_SIMULATION_ENDED")
            {
                m_finished_ = true;
                std::println<>("Received simulation end signal.");
            }
            else if (msg == "FATSIM_TRACKER_EXITING")
            {
                m_finished_ = true;
                std::println<>("Received simulation exit signal.");
            }

            return not m_finished_ and (msg == "FATSIM_DRONE_MOVING" or msg == "FATSIM_DRONE_STOPPED");
        }

        return false;
    }

    void DroneTracker::CaptureFrame_()
    {
        try
        {
            const auto& response = m_drone_client_.simGetImages(m_img_requests_, "SimpleFlight", mc_from_external_camera_);

            if (response.empty())
            {
                throw std::runtime_error("Image Response is EMPTY!");
            }

            if (response[0].image_data_uint8.empty() || response[0].height == 0 || response[0].width == 0)
            {
                m_segmentation_frame_.release();
                throw std::runtime_error("Failed to capture valid image data from AirSim.");
            }

            m_segmentation_frame_ = cv::Mat(response[0].height, response[0].width, CV_8UC3, static_cast<void*>(const_cast<uchar*>(response[0].image_data_uint8.data()))).clone();
        }
        catch (const cv::Exception&)
        {
            m_segmentation_frame_.release();
            throw std::runtime_error("OpenCV exception during Mat creation!");
        }
        catch (const std::exception&)
        {
            m_segmentation_frame_.release();
            throw std::runtime_error("OpenCV exception during Mat creation!");
        }
    }
    void DroneTracker::ApplyOpeningToMaskedFrame_()
    {
        const auto& kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));

        cv::erode(m_masked_frame_,  m_masked_frame_, kernel);
        cv::dilate(m_masked_frame_, m_masked_frame_, kernel);
    }
    void DroneTracker::FindLargestContour_()
    {
        double maxArea{};

        for (std::size_t i{}; i < m_contours_.size(); ++i)
        {
            const auto& area = cv::contourArea(m_contours_[i]);

            if (area > 15.0 and area > maxArea)
            {
                maxArea = area;
                m_largest_contour_idx_ = i;
            }
        }
    }
    void DroneTracker::DetectAndPublishDronePosition_()
    {
        if (m_segmentation_frame_.empty())
        {
            m_zmq_publisher_.Publish("FATSIM_DRONE_NOT_FOUND");

            return;
        }

        cv::inRange(m_segmentation_frame_, cv::Scalar(106, 31, 92), cv::Scalar(106, 31, 92), m_masked_frame_);
        ApplyOpeningToMaskedFrame_();
        cv::imshow("Masked Frame", m_masked_frame_);
        cv::findContours(m_masked_frame_, m_contours_, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        FindLargestContour_();

        bool foundDrone{};

        if (m_largest_contour_idx_ not_eq -1)
        {
            if (const auto& drone_moments = cv::moments(m_contours_[m_largest_contour_idx_]); drone_moments.m00 > 0)
            {
                m_drone_center_.x = static_cast<int>(drone_moments.m10 / drone_moments.m00);
                m_drone_center_.y = static_cast<int>(drone_moments.m01 / drone_moments.m00);
                foundDrone = true;
            }
        }

        if (foundDrone)
        {
            m_display_frame_ = m_segmentation_frame_.clone();

            cv::circle(m_display_frame_, m_drone_center_, 15, cv::Scalar(0, 255, 0), 2);

            cv::line(m_display_frame_, { m_drone_center_.x - 10, m_drone_center_.y      }, { m_drone_center_.x + 10, m_drone_center_.y      }, cv::Scalar(0, 255, 0), 1);
            cv::line(m_display_frame_, { m_drone_center_.x,      m_drone_center_.y - 10 }, { m_drone_center_.x,      m_drone_center_.y + 10 }, cv::Scalar(0, 255, 0), 1);

            // cv::drawContours(m_display_frame_, m_contours_, m_largest_contour_idx_, cv::Scalar(0, 0, 255), 2);

            const auto& offset_x = static_cast<float>(m_drone_center_.x - (m_segmentation_frame_.cols / 2));
            const auto& offset_y = static_cast<float>(m_drone_center_.y - (m_segmentation_frame_.rows / 2));

            const auto& msg = std::format("X:{:.1f},Y:{:.1f}", offset_x, offset_y);
            m_zmq_publisher_.Publish(msg);
            std::println<>("Published position: {}", msg);

            cv::imshow("Drone Detection", m_display_frame_);
        }
        else
        {
            m_zmq_publisher_.Publish("Publishing from DroneTracker: FATSIM_DRONE_NOT_FOUND");
            std::println<>("FATSIM_DRONE_NOT_FOUND");
        }

        m_largest_contour_idx_ = -1;
        m_contours_     = {};
        m_masked_frame_ = {};
        m_drone_center_ = cv::Point(-1, -1);
    }
}