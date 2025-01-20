#include "ZMQSubscriber.hpp"

namespace fatsim
{
    ZMQSubscriber::ZMQSubscriber()
        :
        m_context_(::zmq_ctx_new()),
        m_subscriber_(::zmq_socket(m_context_, ZMQ_SUB))
    {
        ::zmq_connect(m_subscriber_, "tcp://localhost:5555");
        ::zmq_setsockopt(m_subscriber_, ZMQ_SUBSCRIBE, "", 0);
    }
    ZMQSubscriber::~ZMQSubscriber() noexcept
    {
        ::zmq_close(m_subscriber_);
        ::zmq_ctx_destroy(m_context_);
    }

    auto ZMQSubscriber::Receive(Buffer_t& buffer) -> int
    {
        return ::zmq_recv(m_subscriber_, buffer.data(), buffer.size() - 1, 0);
    }
}