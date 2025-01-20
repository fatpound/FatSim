#pragma once

#include "ZMQPublisher.hpp"

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
        static constexpr auto scx_DroneSpeed_ = 2.0f;

    public:
        using Position_t = msr::airlib::Vector3r;
        using Route_t = std::vector<Position_t>;


    public:
        DroneRouter(Route_t route);

        DroneRouter() = delete;
        DroneRouter(const DroneRouter&)     = delete;
        DroneRouter(DroneRouter&&) noexcept = delete;

        auto operator = (const DroneRouter&)     -> DroneRouter& = delete;
        auto operator = (DroneRouter&&) noexcept -> DroneRouter& = delete;
        ~DroneRouter() noexcept(false);


    public:
        void Run(const unsigned int loopCount = 0u);


    protected:


    private:
        void SetDroneObjectID_(const int id = 42);
        void FollowRoute_();
        void SendZMQMessage_();


    private:
        msr::airlib::MultirotorRpcLibClient m_rpc_client_;

        Route_t m_route_;

        ZMQPublisher m_zmq_publisher_;

        std::atomic_bool m_drone_is_moving_{};
        std::atomic_bool m_finished_{};

        std::binary_semaphore m_start_signal_{ 0 };

        std::jthread m_msg_kernel_;
    };
}