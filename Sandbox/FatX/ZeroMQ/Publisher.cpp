#include "Publisher.hpp"

namespace fatx::zeromq
{
    Publisher::Publisher(const std::string& addr)
        :
        m_context_(1),
        m_publisher_(m_context_, zmq::socket_type::pub)
    {
        m_publisher_.bind(addr);
    }

    auto Publisher::Publish(const std::string& msg) -> std::optional<unsigned long long>
    {
        zmq::message_t message(msg.begin(), msg.end());

        return m_publisher_.send(message, zmq::send_flags::none);
    }
}