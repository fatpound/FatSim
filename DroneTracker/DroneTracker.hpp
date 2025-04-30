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
        /*
        *
        * An important note: first element of 'imgRequests' should be the Depth Perspective camera
        */
        DroneTracker(
            std::vector<ImgRequest_t> imgRequests,
            bool                      captureExternalCamera  = false,
            const std::string&        droneRouterSubAddress  = "tcp://localhost:5555",
            const std::string&        unrealEnginePubAddress = "tcp://localhost:5556");

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
            if (m_segmentation_frame_.empty() or contour.empty())
            {
                return;
            }

            cv::Rect box = cv::boundingRect(contour);

            if constexpr (Padding not_eq 0)
            {
                box.x = std::max<>(0, box.x - Padding);
                box.y = std::max<>(0, box.y - Padding);

                const auto& right  = std::min<>(m_segmentation_frame_.cols, box.x + box.width  + Padding);
                const auto& bottom = std::min<>(m_segmentation_frame_.rows, box.y + box.height + Padding);

                box.width  = right  - box.x;
                box.height = bottom - box.y;
            }
            else
            {
                box.width  = std::min(m_segmentation_frame_.cols - box.x, box.width);
                box.height = std::min(m_segmentation_frame_.rows - box.y, box.height);
            }

            if (box.width > 0 and box.height > 0)
            {
                cv::rectangle(m_segmentation_frame_, box, cv::Scalar(0, 255, 0), 2);
            }
        }


    private:
        auto ReceivedContinueMsg_() -> bool;
        auto DroneDetected_() -> bool;

        void CaptureFrame_();
        void ProcessSegmentationImage_();
        void ApplyOpeningToMaskedFrame_();
        void FindLargestContour_();
        void MarkDrone_() const;
        void ShowSegmentationFrame_() const;
        void ShowDepthFrame_() const;
        void ShowMaskedSegmentationFrame_() const;
        void DetectAndPublishDronePosition_();
        void Reset_();


    private:
        inline static const auto            s_drone_bgr_values_ = cv::Scalar(106, 31, 92); // for ID => 42


    private:
        msr::airlib::MultirotorRpcLibClient m_airlib_client_;
        msr::airlib::CameraInfo             m_depth_camera_info_;

        fatx::zeromq::Subscriber            m_zmq_subscriber_;
        fatx::zeromq::Publisher             m_zmq_publisher_;

        std::vector<ImgRequest_t>           m_img_requests_;
        std::vector<std::vector<cv::Point>> m_contours_;

        cv::Point                           m_drone_center_ = cv::Point(-1, -1);
        cv::Mat                             m_segmentation_frame_;
        cv::Mat                             m_depth_frame_;
        cv::Mat                             m_masked_segmentation_frame_;

        std::ptrdiff_t                      m_largest_contour_idx_{ -1 };

        const bool                          mc_from_external_camera_;
        bool                                m_finished_{};
    };
}