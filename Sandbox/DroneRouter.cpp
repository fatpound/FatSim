#include "DroneRouter.hpp"

namespace fatsim
{
    DroneRouter::DroneRouter(Route_t route)
        :
        m_route_(std::move(route)),
        m_msg_kernel_(&DroneRouter::SendZMQMessage_, this)
    {
        m_rpc_client_.confirmConnection();
        m_rpc_client_.enableApiControl(true);
        m_rpc_client_.armDisarm(true);

        SetDroneObjectID_();

        std::println<>("Drone Havalaniyor...");
        m_rpc_client_.takeoffAsync()->waitOnLastTask();
        std::println<>("Drone Havalandi!");
    }
    DroneRouter::~DroneRouter() noexcept(false)
    {
        m_rpc_client_.landAsync()->waitOnLastTask();

        m_rpc_client_.armDisarm(false);
        m_rpc_client_.enableApiControl(false);
    }

    void DroneRouter::Run(const unsigned int loopCount)
    {
        m_start_signal_.release();

        for (auto j = 0ull; loopCount ? (j < loopCount) : true; ++j)
        {
            FollowRoute_();
        }

        m_finished_ = true;
    }

    void DroneRouter::SetDroneObjectID_(const int id)
    {
        const auto droneName = std::string("SimpleFlight");

        if (const bool success = m_rpc_client_.simSetSegmentationObjectID(droneName, id, true))
        {
            std::println<>("{0} icin segmentasyon id'si ayarlandi...", droneName);
        }
        else
        {
            std::println<>("{0} icin segmentasyon id'si AYARLANAMADI!", droneName);
        }
    }
    void DroneRouter::FollowRoute_()
    {
        for (const auto& point : m_route_)
        {
            std::println<>("Drone'un gitmekte oldugu konum: {0} {1} {2}", point.x(), point.y(), point.z());

            m_drone_is_moving_ = true;
            m_rpc_client_.moveToPositionAsync(point.x(), point.y(), point.z(), scx_DroneSpeed_)->waitOnLastTask();
            m_drone_is_moving_ = false;
        }
    }
    void DroneRouter::SendZMQMessage_()
    {
        m_start_signal_.acquire();

        while (not m_finished_)
        {
            using std::literals::string_literals::operator ""s;
            using std::literals::chrono_literals::operator ""ms;

            m_zmq_publisher_.Publish("FATSIM_DRONE_"s + (m_drone_is_moving_ ? "MOVING" : "STOPPED"));

            std::this_thread::sleep_for(50ms);
        }
    }
}