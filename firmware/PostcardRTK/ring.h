// ring.h - lock-free single-producer / single-consumer byte ring.
// One core writes, the other reads. Portable (std::atomic) so it is unit-tested on the PC.
#pragma once
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <cstring>

template <size_t N> class SpscRing
{
    static_assert((N & (N - 1)) == 0, "N must be a power of two");

  public:
    SpscRing() = default;
    SpscRing(const SpscRing &) = delete;
    SpscRing &operator=(const SpscRing &) = delete;
    ~SpscRing() { delete[] _buf; }
    // Producer side. Writes all or nothing; returns false (and counts a drop) when full.
    bool write(const uint8_t *data, size_t len)
    {
        size_t head = _head.load(std::memory_order_relaxed);
        size_t tail = _tail.load(std::memory_order_acquire);
        if (len > N - (head - tail))
        {
            _dropped.fetch_add(1, std::memory_order_relaxed);
            return false;
        }
        size_t idx = head & (N - 1);
        size_t first = (len < N - idx) ? len : N - idx;
        memcpy(&_buf[idx], data, first);
        memcpy(&_buf[0], data + first, len - first);
        _head.store(head + len, std::memory_order_release);
        return true;
    }

    // Consumer side. Returns the number of bytes copied (0..max).
    size_t read(uint8_t *out, size_t max)
    {
        size_t tail = _tail.load(std::memory_order_relaxed);
        size_t head = _head.load(std::memory_order_acquire);
        size_t avail = head - tail;
        size_t len = (avail < max) ? avail : max;
        size_t idx = tail & (N - 1);
        size_t first = (len < N - idx) ? len : N - idx;
        memcpy(out, &_buf[idx], first);
        memcpy(out + first, &_buf[0], len - first);
        _tail.store(tail + len, std::memory_order_release);
        return len;
    }

    size_t available() const
    {
        return _head.load(std::memory_order_acquire) - _tail.load(std::memory_order_acquire);
    }
    size_t space() const { return N - available(); }
    uint32_t dropped() const { return _dropped.load(std::memory_order_relaxed); }

  private:
    // On the heap (in PSRAM on the ESP32 for the large rings): static RAM is scarce next to Wi-Fi and Bluetooth
    uint8_t *const _buf = new uint8_t[N];
    std::atomic<size_t> _head{0};
    std::atomic<size_t> _tail{0};
    std::atomic<uint32_t> _dropped{0};
};
