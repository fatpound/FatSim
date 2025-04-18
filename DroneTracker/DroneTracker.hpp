#pragma once

#include <_exp/ZeroMQ/Subscriber.hpp>

#include <vehicles/multirotor/api/MultirotorRpcLibClient.hpp>

#include <opencv2/opencv.hpp>

#include <string>
#include <chrono>
#include <array>
#include <print>

namespace fatsim
{
    class DroneTracker final
    {
    public:
        using ImgRequest_t = msr::airlib::ImageCaptureBase::ImageRequest;


    public:
        DroneTracker(std::vector<ImgRequest_t> imgRequests, bool fromExternalCamera = false);

        DroneTracker()                        = delete;
        DroneTracker(const DroneTracker&)     = delete;
        DroneTracker(DroneTracker&&) noexcept = delete;

        auto operator = (const DroneTracker&)     -> DroneTracker& = delete;
        auto operator = (DroneTracker&&) noexcept -> DroneTracker& = delete;
        ~DroneTracker() noexcept                                   = default;


    public:
        void Run();


    protected:


    private:
        template <int Padding = 0>
        void DrawRectangleAround_(const std::vector<cv::Point> contour)
        {
            cv::Rect box = cv::boundingRect(contour);

            if constexpr (Padding not_eq 0)
            {
                box.x      = std::max<>(0, box.x - Padding);
                box.y      = std::max<>(0, box.y - Padding);
                box.width  = std::min<>(m_current_frame_.cols - box.x, box.width  + 2 * Padding);
                box.height = std::min<>(m_current_frame_.rows - box.y, box.height + 2 * Padding);
            }

            cv::rectangle(m_current_frame_, box, cv::Scalar(0, 255, 0), 2);
        }


    private:
        auto CaptureFrame_           () -> cv::Mat;
        auto GetFrameDifference_     () -> cv::Mat;
        auto GetDilatedThresholdImg_ () -> cv::Mat;

        auto ReceivedContinueMsg_    () -> bool;

        void ShowDetectedDrone_();


    private:
        msr::airlib::MultirotorRpcLibClient m_rpc_client_;

        std::vector<ImgRequest_t> m_img_requests_;

        fatx::zeromq::Subscriber m_zmq_subscriber_;
        
        const bool mc_from_external_camera_;

        cv::Mat m_prev_frame_;
        cv::Mat m_current_frame_;

        bool m_finished_{};
    };
}