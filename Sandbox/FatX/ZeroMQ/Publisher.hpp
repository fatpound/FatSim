#pragma once

#define ZMQ_STATIC
#include <zmq.h>
#undef ZMQ_STATIC

#pragma comment(lib, "iphlpapi")

#include <string>

namespace fatx::zeromq
{
    class Publisher final
    {
    public:
        Publisher();
        Publisher(const Publisher&)     = delete;
        Publisher(Publisher&&) noexcept = delete;

        auto operator = (const Publisher&)     -> Publisher& = delete;
        auto operator = (Publisher&&) noexcept -> Publisher& = delete;
        ~Publisher() noexcept;


    public:
        auto Publish(const std::string& str) -> int;


    protected:


    private:
        void* m_pContext_{};
        void* m_pPublisher_{};
    };
}