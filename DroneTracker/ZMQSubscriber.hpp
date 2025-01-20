#pragma once

#define ZMQ_STATIC

#include <zmq.h>

#pragma comment(lib, "iphlpapi") // Win32 IP Helper API

#include <array>

namespace fatsim
{
    class ZMQSubscriber final
    {
    public:
        static constexpr auto BufferSize = 256;

        using Buffer_t = std::array<char, BufferSize>;


    public:
        ZMQSubscriber();
        ZMQSubscriber(const ZMQSubscriber&)     = delete;
        ZMQSubscriber(ZMQSubscriber&&) noexcept = delete;

        auto operator = (const ZMQSubscriber&)     -> ZMQSubscriber& = delete;
        auto operator = (ZMQSubscriber&&) noexcept -> ZMQSubscriber& = delete;
        ~ZMQSubscriber() noexcept;


    public:
        auto Receive(Buffer_t& buffer) -> int;


    protected:


    private:
        void* m_context_{};
        void* m_subscriber_{};
    };
}