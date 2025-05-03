#pragma once

#include <_exp/ZeroMQ/Publisher.hpp>

#include <vehicles/multirotor/api/MultirotorRpcLibClient.hpp>

#include <string>
#include <chrono>
#include <thread>
#include <semaphore>
#include <print>

namespace fatsim
{
    class DroneRouter final
    {
        static constexpr auto scx_DroneSpeed_ = 6.0F;

    public:
        using Position_t = msr::airlib::Vector3r;


    public:
        DroneRouter(std::vector<Position_t> route, const std::string& trackerPubAddress = "tcp://localhost:5555", const std::size_t& loopCount = 2U);

        DroneRouter()                       = delete;
        DroneRouter(const DroneRouter&)     = delete;
        DroneRouter(DroneRouter&&) noexcept = delete;

        auto operator = (const DroneRouter&)     -> DroneRouter& = delete;
        auto operator = (DroneRouter&&) noexcept -> DroneRouter& = delete;
        ~DroneRouter() noexcept(false);


    protected:


    private:
        void SetDroneObjectID_(const int& id);
        void FollowRoute_();
        void SendZMQMessage_();
        void DetectCrash_();


    private:
        msr::airlib::MultirotorRpcLibClient m_airlib_client_;

        std::vector<Position_t>             m_route_;

        fatx::zeromq::Publisher             m_zmq_publisher_;

        const std::size_t                   mc_loop_count_;

        std::atomic_bool                    m_drone_is_moving_{};
        std::atomic_bool                    m_finished_{};
        std::atomic_bool                    m_emergency_stop_{};

        std::binary_semaphore               m_start_signal_{ 0 };
        std::binary_semaphore               m_finish_signal_{ 0 };
        std::binary_semaphore               m_zmq_start_signal_{ 0 };
        std::binary_semaphore               m_crash_detection_start_signal_{ 0 };

        std::jthread                        m_route_follower_;
        std::jthread                        m_msg_kernel_;
        std::jthread                        m_crash_detector_;
    };
}