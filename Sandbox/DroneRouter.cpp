#include "DroneRouter.hpp"

namespace fatsim
{
    DroneRouter::DroneRouter(std::vector<Position_t> route, const std::string& trackerPubAddress)
        :
        m_route_(route),
        m_zmq_publisher_(trackerPubAddress),
#pragma region (thread w/o C4355)
#pragma warning (push)
#pragma warning (disable : 4355)
        m_msg_kernel_(&DroneRouter::SendZMQMessage_, this)
#pragma warning (pop)
#pragma endregion
    {
        m_drone_client_.confirmConnection();
        m_drone_client_.enableApiControl(true);
        m_drone_client_.armDisarm(true);

        SetDroneObjectID_(42);

        std::println<>("Drone Havalaniyor...");
        m_drone_client_.takeoffAsync()->waitOnLastTask();
        std::println<>("Drone Havalandi!");
    }
    DroneRouter::~DroneRouter() noexcept(false)
    {
        m_drone_client_.landAsync()->waitOnLastTask();

        m_drone_client_.armDisarm(false);
        m_drone_client_.enableApiControl(false);
    }

    void DroneRouter::Run(const std::size_t& loop)
    {
        m_start_signal_.release();

        for (std::size_t j{}; j < loop; ++j)
        {
            FollowRoute_();
        }

        m_finished_ = true;
    }

    void DroneRouter::SetDroneObjectID_(const int& id)
    {
        if (const std::string& droneName = "SimpleFlight"; m_drone_client_.simSetSegmentationObjectID(droneName, id, true) == true)
        {
            std::println<>("Segmentation ID: {} set for {}", id, droneName);

            return;
        }

        throw std::runtime_error("COULD NOT set Segmentation ID!");
    }
    void DroneRouter::FollowRoute_()
    {
        for (const auto& point : m_route_)
        {
            const auto& x = point.x();
            const auto& y = point.y();
            const auto& z = point.z();
            
            std::println<>("Drone is going to : X={0} Y={1} Z={2}", x, y, z);

            m_drone_is_moving_ = true;
            m_drone_client_.moveToPositionAsync(x / 100.0F, y / 100.0F, -(z / 100.0F), scx_DroneSpeed_)->waitOnLastTask();
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

            const auto& msg = "FATSIM_DRONE_"s + (m_drone_is_moving_ ? "MOVING"s : "STOPPED"s);

            std::println<>("Publishing message: {}", msg);
            m_zmq_publisher_.Publish(msg);

            std::this_thread::sleep_for(200ms);
        }

        m_zmq_publisher_.Publish("FATSIM_SIMULATION_ENDED");
        std::println<>("Published: FATSIM_SIMULATION_ENDED");
    }
}