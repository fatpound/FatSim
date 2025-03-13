#include "Publisher.hpp"

namespace fatx::zeromq
{
    Publisher::Publisher(const char* const pAddr) noexcept
        :
        m_pContext_(::zmq_ctx_new()),
        m_pPublisher_(::zmq_socket(m_pContext_, ZMQ_PUB))
    {
        ::zmq_bind(m_pPublisher_, pAddr);
    }
    Publisher::~Publisher() noexcept
    {
        ::zmq_close(m_pPublisher_);
        ::zmq_ctx_destroy(m_pContext_);
    }

    auto Publisher::Publish(const std::string& str) noexcept -> int
    {
        return ::zmq_send(m_pPublisher_, str.c_str(), str.size(), 0);
    }
}