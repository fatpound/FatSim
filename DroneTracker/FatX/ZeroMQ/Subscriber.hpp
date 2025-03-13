#pragma once

#define ZMQ_STATIC
#include <zmq.h>
#undef ZMQ_STATIC

#pragma comment(lib, "iphlpapi")

#include <array>

namespace fatx::zeromq
{
    class Subscriber final
    {
    public:
        static constexpr auto scx_BufferSize = 256;


    public:
        using Buffer_t = std::array<char, scx_BufferSize>;


    public:
        Subscriber();
        Subscriber(const Subscriber&)     = delete;
        Subscriber(Subscriber&&) noexcept = delete;

        auto operator = (const Subscriber&)     -> Subscriber& = delete;
        auto operator = (Subscriber&&) noexcept -> Subscriber& = delete;
        ~Subscriber() noexcept;


    public:
        auto Receive(Buffer_t& buffer) noexcept -> int;


    protected:


    private:
        void* m_pContext_{};
        void* m_pSubscriber_{};
    };
}