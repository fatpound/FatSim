#pragma once

#define ZMQ_STATIC
#include <zmq.hpp>
#undef ZMQ_STATIC

#pragma comment(lib, "iphlpapi")

#include <string>
#include <optional>

namespace fatx::zeromq
{
    class Publisher final
    {
    public:
        Publisher(const std::string& address = "tcp://localhost:5555");

        Publisher(const Publisher&)     = delete;
        Publisher(Publisher&&) noexcept = delete;

        auto operator = (const Publisher&)     -> Publisher& = delete;
        auto operator = (Publisher&&) noexcept -> Publisher& = delete;
        ~Publisher() noexcept                                = default;


    public:
        auto Publish(const std::string& str) -> std::optional<unsigned long long>;


    protected:


    private:
        ::zmq::context_t m_context_;
        ::zmq::socket_t  m_publisher_;
    };
}