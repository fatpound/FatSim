#pragma once

#define ZMQ_STATIC

#include <zmq.h>

#pragma comment(lib, "iphlpapi") // Win32 IP Helper API

#include <string>

namespace fatsim
{
    class ZMQPublisher final
    {
    public:
        ZMQPublisher();
        ZMQPublisher(const ZMQPublisher&)     = delete;
        ZMQPublisher(ZMQPublisher&&) noexcept = delete;

        auto operator = (const ZMQPublisher&)     -> ZMQPublisher& = delete;
        auto operator = (ZMQPublisher&&) noexcept -> ZMQPublisher& = delete;
        ~ZMQPublisher() noexcept;


    public:
        auto Publish(const std::string& str) -> int;


    protected:


    private:
        void* m_context_{};
        void* m_publisher_{};
    };
}