#include "DroneTracker.hpp"

namespace fatsim
{
    DroneTracker::DroneTracker(std::vector<ImgRequest_t> imgRequests, bool fromExternalCamera)
        :
        m_img_requests_(std::move(imgRequests)),
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
            ShowDetectedDrone_();
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
    }

    auto DroneTracker::CaptureFrame_() -> cv::Mat
    {
        auto response = m_rpc_client_.simGetImages(m_img_requests_, "SimpleFlight", mc_from_external_camera_);

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
        cv::Mat prevGray;
        cv::Mat currentGray;
        cv::Mat diff;

        cv::cvtColor(m_prev_frame_,       prevGray, cv::COLOR_BGR2GRAY);
        cv::cvtColor(m_current_frame_, currentGray, cv::COLOR_BGR2GRAY);

        cv::absdiff(prevGray, currentGray, diff);

        return diff;
    }
    auto DroneTracker::GetDilatedThresholdImg_() -> cv::Mat
    {
        cv::Mat thresh;
        cv::Mat threshDilated;

        cv::threshold(GetFrameDifference_(), thresh, 25, 255, cv::THRESH_BINARY);
        cv::dilate(thresh, threshDilated, cv::Mat(), cv::Point(-1, -1), 2);

        return threshDilated;
    }

    auto DroneTracker::ReceivedContinueMsg_() -> bool
    {
        if (const auto& msg = m_zmq_subscriber_.Receive(); not msg.empty())
        {
            std::println<>("Message received: {0}", msg);

            if (msg == "FATSIM_SIMULATION_ENDED")
            {
                m_finished_ = true;
            }

            return not m_finished_ and (msg == "FATSIM_DRONE_MOVING" or msg == "FATSIM_DRONE_STOPPED");
        }

        return false;
    }

    void DroneTracker::ShowDetectedDrone_()
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

        cv::imshow("Drone Detection", m_current_frame_);
    }
}