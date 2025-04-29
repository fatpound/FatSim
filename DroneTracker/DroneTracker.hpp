#pragma once

#include <_exp/ZeroMQ/Subscriber.hpp>
#include <_exp/ZeroMQ/Publisher.hpp>

#include <vehicles/multirotor/api/MultirotorRpcLibClient.hpp>

#include <opencv2/opencv.hpp>

#include <string>
#include <chrono>
#include <array>
#include <print>
#include <format>

namespace fatsim
{
    class DroneTracker final
    {
    public:
        using ImgRequest_t = msr::airlib::ImageCaptureBase::ImageRequest;


    public:
        DroneTracker(
            std::vector<ImgRequest_t> imgRequests,
            bool                      fromExternalCamera = false,
            const std::string&        routerAddress = "tcp://localhost:5555",
            const std::string&        unrealAddress = "tcp://*:5556");

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
            if (m_current_frame_.empty() or contour.empty())
            {
                return;
            }

            cv::Rect box = cv::boundingRect(contour);

            if constexpr (Padding not_eq 0)
            {
                box.x = std::max<>(0, box.x - Padding);
                box.y = std::max<>(0, box.y - Padding);

                const auto& right  = std::min<>(m_current_frame_.cols, box.x + box.width  + Padding);
                const auto& bottom = std::min<>(m_current_frame_.rows, box.y + box.height + Padding);

                box.width  = right  - box.x;
                box.height = bottom - box.y;
            }
            else
            {
                box.width  = std::min(m_current_frame_.cols - box.x, box.width);
                box.height = std::min(m_current_frame_.rows - box.y, box.height);
            }

            if (box.width > 0 and box.height > 0)
            {
                cv::rectangle(m_current_frame_, box, cv::Scalar(0, 255, 0), 2);
            }
        }


    private:
        auto CaptureFrame_           () -> cv::Mat;
        auto GetFrameDifference_     () -> cv::Mat;
        auto GetDilatedThresholdImg_ () -> cv::Mat;

        auto ReceivedContinueMsg_    () -> bool;

        void DetectAndPublishDronePosition_();
        void ShowDetectedDrone_();


    private:
        msr::airlib::MultirotorRpcLibClient m_rpc_client_;

        std::vector<ImgRequest_t> m_img_requests_;

        fatx::zeromq::Subscriber m_zmq_subscriber_;
        fatx::zeromq::Publisher  m_zmq_publisher_;
        
        const bool mc_from_external_camera_;

        cv::Mat m_prev_frame_;
        cv::Mat m_current_frame_;

        bool m_finished_{};
    };
}