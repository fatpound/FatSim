#include "Subscriber.hpp"

namespace fatx::zeromq
{
    Subscriber::Subscriber(const std::string& address)
        :
        m_context_(1),
        m_subscriber_(m_context_, ::zmq::socket_type::sub)
    {
        m_subscriber_.connect(address);
        m_subscriber_.set(::zmq::sockopt::subscribe, "");
    }

    auto Subscriber::Receive() -> std::string
    {
        ::zmq::message_t message;

        {
            [[maybe_unused]]
            const auto& retval = m_subscriber_.recv(message, ::zmq::recv_flags::none);
        }

        return message.to_string();
    }
}