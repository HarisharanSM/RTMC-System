#pragma once

#include <array>
#include <atomic>
#include <cstddef>

namespace RTMCCollision {

template <typename T, std::size_t Capacity>
class cSpscMailbox {
    static_assert(Capacity > 1, "SPSC mailbox needs at least two slots");

public:
    bool Push(const T& value) {
        const std::size_t tail = m_Tail.load(std::memory_order_relaxed);
        const std::size_t next = Increment(tail);
        if (next == m_Head.load(std::memory_order_acquire)) return false;
        m_Slots[tail] = value;
        m_Tail.store(next, std::memory_order_release);
        return true;
    }

    bool Pop(T& value) {
        const std::size_t head = m_Head.load(std::memory_order_relaxed);
        if (head == m_Tail.load(std::memory_order_acquire)) return false;
        value = m_Slots[head];
        m_Head.store(Increment(head), std::memory_order_release);
        return true;
    }

    void Drain() {
        T ignored{};
        while (Pop(ignored)) {}
    }

private:
    static constexpr std::size_t Increment(std::size_t value) {
        return (value + 1) % Capacity;
    }

    std::array<T, Capacity> m_Slots{};
    alignas(64) std::atomic<std::size_t> m_Head{0};
    alignas(64) std::atomic<std::size_t> m_Tail{0};
};

} // namespace RTMCCollision
