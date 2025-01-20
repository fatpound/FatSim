#include "ZMQPublisher.hpp"

namespace fatsim
{
    ZMQPublisher::ZMQPublisher()
        :
        m_context_(::zmq_ctx_new()),
        m_publisher_(::zmq_socket(m_context_, ZMQ_PUB))
    {
        ::zmq_bind(m_publisher_, "tcp://*:5555");
    }
    ZMQPublisher::~ZMQPublisher() noexcept
    {
        ::zmq_close(m_publisher_);
        ::zmq_ctx_destroy(m_context_);
    }

    auto ZMQPublisher::Publish(const std::string& str) -> int
    {
        return ::zmq_send(m_publisher_, str.c_str(), str.size(), 0);
    }
}