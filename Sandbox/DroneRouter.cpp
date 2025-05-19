#include "DroneRouter.hpp"

namespace fatsim
{
    DroneRouter::DroneRouter(std::vector<Position_t> route, const std::size_t& loopCount, const std::string& trackerPubAddress)
        :
        m_route_(route),
        m_zmq_publisher_(trackerPubAddress),
        mc_loop_count_(loopCount),
#pragma region (thread w/o C4355)
#pragma warning (push)
#pragma warning (disable : 4355)
        m_route_follower_(&DroneRouter::FollowRoute_, this),
        m_msg_kernel_(&DroneRouter::SendZMQMessage_,  this),
        m_crash_detector_(&DroneRouter::DetectCrash_, this)
#pragma warning (pop)
#pragma endregion
    {
        m_airlib_client_.confirmConnection();
        m_airlib_client_.enableApiControl(true);
        m_airlib_client_.armDisarm(true);
        
        SetDroneObjectID_(50);

        std::println<>("Drone is taking off...");
        m_airlib_client_.takeoffAsync();

        m_start_signal_.release();
        m_crash_detection_start_signal_.release();
    }
    DroneRouter::~DroneRouter() noexcept(false)
    {
        m_quit_signal_.acquire();

        if (not m_emergency_stop_)
        {
            m_airlib_client_.landAsync()->waitOnLastTask();
        }

        m_airlib_client_.armDisarm(false);
        m_airlib_client_.enableApiControl(false);

        std::println<>("FatSim has finished.");
    }

    void DroneRouter::SetDroneObjectID_(const int& id)
    {
        if (const std::string& droneName = "SimpleFlight"; m_airlib_client_.simSetSegmentationObjectID(droneName, id, true) == true)
        {
            std::println<>("Segmentation ID: {} set for {}", id, droneName);

            return;
        }

        throw std::runtime_error("COULD NOT set Segmentation ID!");
    }
    void DroneRouter::FollowRoute_()
    {
        m_start_signal_.acquire();

        m_airlib_client_.waitOnLastTask();

        m_zmq_start_signal_.release();

        if (m_emergency_stop_)
        {
            std::println<>("Emergency Stop signal received in Drone Router thread...");

            goto end;
        }

        std::println<>("Drone is airborne...");
        std::println<>("Routing drone...");

        for (std::size_t j{}; j < mc_loop_count_ and not m_emergency_stop_; ++j)
        {
            for (const auto& point : m_route_)
            {
                if (m_emergency_stop_)
                {
                    std::println<>("Emergency Stop signal received in Drone Router thread...");

                    goto end;
                }

                const auto& x = point.x() / 100.0F;
                const auto& y = point.y() / 100.0F;
                const auto& z = point.z() / 100.0F;

                std::println<>("Drone is going to : X={0} Y={1} Z={2}", x, y, z);

                m_drone_is_moving_ = true;
                m_airlib_client_.moveToPositionAsync(x, y, -z, scx_DroneSpeed_)->waitOnLastTask();
                m_drone_is_moving_ = false;
            }
        }

        if (m_emergency_stop_)
        {
            std::println<>("Emergency Stop signal received in Drone Router thread...");
        }

    end:
        m_finished_ = true;

        std::println<>("Drone Router thread is stopping...");
        m_finish_signal_.release();
    }
    void DroneRouter::SendZMQMessage_()
    {
        m_zmq_start_signal_.acquire();

        if (m_emergency_stop_)
        {
            std::println<>("Emergency Stop signal received in ZMQ Message Publisher thread...");

            goto end;
        }

        std::println<>("ZMQ Message Publisher thread is starting publishing...");

        while (not m_finished_ and not m_emergency_stop_)
        {
            using std::literals::chrono_literals::operator ""ms;

            const auto& msg = m_drone_is_moving_
                ? "FATSIM_DRONE_MOVING"
                : "FATSIM_DRONE_STOPPED"
                ;

            std::println<>("Publishing message: {}", msg);
            m_zmq_publisher_.Publish(msg);

            std::this_thread::sleep_for(100ms);
        }

    end:
        m_zmq_publisher_.Publish("FATSIM_SIMULATION_ENDED");
        std::println<>("Published: FATSIM_SIMULATION_ENDED");
        std::println<>("ZMQ Message Publisher thread is stopping...");

        m_finish_signal_.acquire();
        m_quit_signal_.release();
    }
    void DroneRouter::DetectCrash_()
    {
        m_crash_detection_start_signal_.acquire();

        std::println<>("Crash Detector thread is starting receiving collision info...");

        static_cast<void>(m_airlib_client_.simGetCollisionInfo());

        while (not m_finished_)
        {
            if (m_airlib_client_.simGetCollisionInfo().has_collided)
            {
                std::println<>("Collision detected. Stopping drone...");

                m_emergency_stop_ = true;
                m_finished_       = true;

                m_airlib_client_.cancelLastTask();

                break;
            }

            using std::literals::chrono_literals::operator ""ms;

            std::this_thread::sleep_for(10ms);
        }

        std::println<>("Crash Detector thread is stopping...");
    }
}