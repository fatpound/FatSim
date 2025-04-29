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
        DroneRouter(std::vector<Position_t> route, const std::string& trackerPubAddress = "tcp://localhost:5555");

        DroneRouter()                       = delete;
        DroneRouter(const DroneRouter&)     = delete;
        DroneRouter(DroneRouter&&) noexcept = delete;

        auto operator = (const DroneRouter&)     -> DroneRouter& = delete;
        auto operator = (DroneRouter&&) noexcept -> DroneRouter& = delete;
        ~DroneRouter() noexcept(false);


    public:
        void Run(const std::size_t& loop);


    protected:


    private:
        void SetDroneObjectID_(const int& id);
        void FollowRoute_();
        void SendZMQMessage_();


    private:
        msr::airlib::MultirotorRpcLibClient m_drone_client_;

        std::vector<Position_t>             m_route_;

        fatx::zeromq::Publisher             m_zmq_publisher_;

        std::atomic_bool                    m_drone_is_moving_{};
        std::atomic_bool                    m_finished_{};
        std::binary_semaphore               m_start_signal_{ 0 };
        std::jthread                        m_msg_kernel_;
    };
}