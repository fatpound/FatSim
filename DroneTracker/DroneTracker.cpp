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
        m_rpc_client_.confirmConnection();

        std::println<>("Starting DroneTracker...");
    }

    void DroneTracker::Run()
    {
        m_prev_frame_ = CaptureFrame_();

        while (not m_finished_)
        {
            if (not ReceivedContinueMsg_())
            {
                goto wait;
            }

            m_current_frame_ = CaptureFrame_();
            DetectAndPublishDronePosition_();
            m_prev_frame_ = m_current_frame_.clone();

            if (cv::waitKey(1) == 27)
            {
                break;
            }
            else
            {
                continue;
            }

        wait:
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }

        cv::destroyAllWindows();
        std::println<>("DroneTracker finished.");
    }
    
    auto DroneTracker::CaptureFrame_() -> cv::Mat
    {
        const auto& response = m_rpc_client_.simGetImages(m_img_requests_, "SimpleFlight", mc_from_external_camera_);

        if (response.empty() or response[0].image_data_uint8.empty())
        {
            throw std::runtime_error("Failed to capture image from AirSim.");
        }

        const auto& imgData = response[0].image_data_uint8;

        cv::Mat img(cv::Size(response[0].width, response[0].height), CV_8UC3);
        std::memcpy(img.data, imgData.data(), imgData.size());

        return img.clone();
    }
    auto DroneTracker::GetFrameDifference_() -> cv::Mat
    {
        if (m_prev_frame_.empty() or m_current_frame_.empty() or (m_prev_frame_.size() not_eq m_current_frame_.size()))
        {
            return {};
        }

        cv::Mat diff;

        {
            cv::Mat prevGray;
            cv::Mat currentGray;

            cv::cvtColor(m_prev_frame_,       prevGray, cv::COLOR_BGR2GRAY);
            cv::cvtColor(m_current_frame_, currentGray, cv::COLOR_BGR2GRAY);

            cv::absdiff(prevGray, currentGray, diff);
        }
        

        return diff;
    }
    auto DroneTracker::GetDilatedThresholdImg_() -> cv::Mat
    {
        cv::Mat diff = GetFrameDifference_();
        
        if (diff.empty())
        {
            return {};
        }

        cv::Mat thresh;
        cv::Mat threshDilated;

        cv::threshold(diff, thresh, 30, 255, cv::THRESH_BINARY);
        cv::dilate(thresh, threshDilated, cv::Mat(), cv::Point(-1, -1), 3);

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
            }

            return not m_finished_ and (msg == "FATSIM_DRONE_MOVING" or msg == "FATSIM_DRONE_STOPPED");
        }

        return false;
    }

    void DroneTracker::DetectAndPublishDronePosition_()
    {
        cv::Mat detection_mask = GetDilatedThresholdImg_();

        if (detection_mask.empty())
        {
            m_zmq_publisher_.Publish("FATSIM_DRONE_NOT_FOUND");
            cv::imshow("Drone Detection", m_current_frame_);

            return;
        }

        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(detection_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        
        bool foundDrone{};
        cv::Point droneCenter(-1, -1);

        auto maxArea = 50.0;
        auto largestContourIndex = -1;

        for (int i = 0; i < contours.size(); ++i)
        {
            const auto& area = cv::contourArea(contours[i]);

            if (area > maxArea)
            {
                maxArea = area;
                largestContourIndex = i;
            }
        }

        if (largestContourIndex not_eq -1)
        {
            const auto& largestContour = contours[largestContourIndex];
            DrawRectangleAround_<10>(largestContour);
            const auto& box = cv::boundingRect(largestContour);
            droneCenter.x = box.x + box.width  / 2;
            droneCenter.y = box.y + box.height / 2;
            foundDrone = true;
        }

        if (foundDrone)
        {
            const auto& center_x = m_current_frame_.cols / 2;
            const auto& center_y = m_current_frame_.rows / 2;
            const auto& offset_x = static_cast<float>(droneCenter.x - center_x);
            const auto& offset_y = static_cast<float>(droneCenter.y - center_y);

            std::string msg = std::format("X:{:.1f},Y:{:.1f}", offset_x, offset_y);
            m_zmq_publisher_.Publish(msg);
            std::println<>("DroneTracker has Published: {}", msg);
        }
        else
        {
            m_zmq_publisher_.Publish("FATSIM_DRONE_NOT_FOUND");
            std::println<>("DroneTracker has Published: NOT_FOUND");
        }

        cv::imshow("Drone Detection", m_current_frame_);
    }
    void DroneTracker::ShowDetectedDrone_()
    {
        {
            std::vector<std::vector<cv::Point>> contours;
            cv::findContours(GetDilatedThresholdImg_(), contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

            for (const auto& contour : contours)
            {
                if (cv::contourArea(contour) > 50)
                {
                    DrawRectangleAround_<10>(contour);
                }
            }
        }

        cv::imshow("Drone Detection", m_current_frame_);
    }
}