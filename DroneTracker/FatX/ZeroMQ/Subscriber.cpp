#include "Subscriber.hpp"

namespace fatx::zeromq
{
    Subscriber::Subscriber()
        :
        m_pContext_(::zmq_ctx_new()),
        m_pSubscriber_(::zmq_socket(m_pContext_, ZMQ_SUB))
    {
        ::zmq_connect(m_pSubscriber_, "tcp://localhost:5555");
        ::zmq_setsockopt(m_pSubscriber_, ZMQ_SUBSCRIBE, "", 0);
    }
    Subscriber::~Subscriber() noexcept
    {
        ::zmq_close(m_pSubscriber_);
        ::zmq_ctx_destroy(m_pContext_);
    }

    auto Subscriber::Receive(Buffer_t& buffer) -> int
    {
        return ::zmq_recv(m_pSubscriber_, buffer.data(), buffer.size() - 1, 0);
    }
}