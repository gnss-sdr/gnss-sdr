/*!
 * \file shm_ring.h
 * \brief Single-Producer/Single-Consumer shared-memory ring carrying the raw
 * EVK1029 byte stream between EVK1029_CtrlApp (producer) and this consumer
 * (SharedMemoryStreamReader) -- see EVK1029_shared_memory_input_spec(2).md
 * for the overall design this implements.
 *
 * Verbatim copy of EVK1029_CtrlApp's own evk1029/shm_ring.hpp: the two repos
 * build separately, but both mmap the same region, so the struct layout and
 * kMagic/kVersion must match -- Attach() refuses a mismatched version.
 *
 * Wire format: [Header][Block 0][Block 1]...[Block N-1], each block an
 * 8-byte payload-length prefix (real USB chunks aren't always exactly
 * block_size) followed by up to (block_size - 8) data bytes.
 *
 * write_seq/read_seq (one cache line apart) are the lock-free source of
 * truth; data_event/space_event are separate 32-bit futex words since
 * futex() needs 32-bit and the sequence counters are 64-bit -- a waiting
 * side snapshots the event word, re-checks the real condition (lost-wakeup
 * guard), then futex_waits on the snapshot. Never FUTEX_PRIVATE_FLAG: waited
 * on across processes, not threads.
 *
 * The *_waiters counters only let a side skip the futex_wake() syscall when
 * nobody is blocked; they can go stale without losing a wakeup, since
 * FUTEX_WAIT's own atomic expected-value check (not this counter) is what
 * actually prevents one -- a stale counter only ever causes an extra wake.
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2026  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */

#ifndef GNSS_SDR_SHM_RING_H
#define GNSS_SDR_SHM_RING_H

#include <algorithm>
#include <atomic>
#include <cstdint>
#include <cstring>
#include <fcntl.h>
#include <linux/futex.h>
#include <string>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/syscall.h>
#include <unistd.h>
#include <vector>

namespace shm_ring
{

constexpr uint32_t kMagic = 0x45563130u;         // "EV10"
constexpr uint32_t kVersion = 2u;                // v2: added data_waiters/space_waiters (see file comment)
constexpr std::size_t kHeaderRegionSize = 4096;  // one page; data blocks start right after

// Producer-owned stream lifecycle (Header::state); a temporarily empty ring
// is not EOF on its own -- only Stopped/EndOfStream is (see ReadBlock()).
enum class StreamState : uint32_t
{
    Initializing = 0,
    Running = 1,
    EndOfStream = 2,
    Stopped = 3,
    Error = 4,
};

struct alignas(64) Header
{
    uint32_t magic = 0;
    uint32_t version = 0;
    uint32_t block_size = 0;  // includes the 8-byte length prefix
    uint32_t block_count = 0;

    alignas(64) std::atomic<uint64_t> write_seq{0};
    alignas(64) std::atomic<uint64_t> read_seq{0};

    alignas(64) std::atomic<uint32_t> data_event{0};
    std::atomic<uint32_t> data_waiters{0};
    std::atomic<uint32_t> space_event{0};
    std::atomic<uint32_t> space_waiters{0};
    std::atomic<uint32_t> state{0};  // StreamState

    std::atomic<uint64_t> bytes_written{0};
    std::atomic<uint64_t> bytes_read{0};
};
static_assert(sizeof(Header) <= kHeaderRegionSize, "Header grew past its reserved page");

struct MappedRing
{
    int fd = -1;
    void* base = nullptr;
    std::size_t size = 0;
    Header* header = nullptr;
    bool owner = false;  // true for the creator (producer) -- Destroy() shm_unlink()s only then
    std::string name;
};

inline std::size_t TotalRegionSize(uint32_t block_size, uint32_t block_count)
{
    return kHeaderRegionSize + static_cast<std::size_t>(block_size) * static_cast<std::size_t>(block_count);
}

inline uint8_t* BlockPtr(void* base, uint32_t index, uint32_t block_size)
{
    return reinterpret_cast<uint8_t*>(base) + kHeaderRegionSize +
           static_cast<std::size_t>(index) * static_cast<std::size_t>(block_size);
}

inline long RawFutex(std::atomic<uint32_t>* word, int op, uint32_t val)
{
    return syscall(SYS_futex, reinterpret_cast<uint32_t*>(word), op, val, nullptr, nullptr, 0);
}

inline void FutexWait(std::atomic<uint32_t>& word, uint32_t expected)
{
    // Return value ignored: the caller's own loop re-checks the real
    // condition regardless of why this returned (EAGAIN/EINTR/spurious).
    RawFutex(&word, FUTEX_WAIT, expected);
}

inline void FutexWakeAll(std::atomic<uint32_t>& word)
{
    RawFutex(&word, FUTEX_WAKE, static_cast<uint32_t>(0x7fffffff));
}

// Increments waiters before the futex syscall, so a concurrent
// WakeIfWaiting() is guaranteed to see it (see the file comment).
inline void WaitOn(std::atomic<uint32_t>& word, uint32_t expected, std::atomic<uint32_t>& waiters)
{
    waiters.fetch_add(1, std::memory_order_acq_rel);
    FutexWait(word, expected);
    waiters.fetch_sub(1, std::memory_order_acq_rel);
}

inline void WakeIfWaiting(std::atomic<uint32_t>& word, const std::atomic<uint32_t>& waiters)
{
    if (waiters.load(std::memory_order_acquire) > 0)
        FutexWakeAll(word);
}

// ---- Producer (creator) side ----------------------------------------------

// payload_block_size excludes the 8-byte length prefix added on top.
inline bool Create(MappedRing& ring, const std::string& name, uint32_t payload_block_size, uint32_t block_count)
{
    const uint32_t block_size = payload_block_size + static_cast<uint32_t>(sizeof(uint64_t));
    const std::size_t total = TotalRegionSize(block_size, block_count);

    shm_unlink(name.c_str());  // best-effort: drop a stale segment left by a previous crash
    const int fd = shm_open(name.c_str(), O_CREAT | O_EXCL | O_RDWR, 0600);
    if (fd < 0)
        return false;

    if (ftruncate(fd, static_cast<off_t>(total)) != 0)
        {
            close(fd);
            shm_unlink(name.c_str());
            return false;
        }

    void* base = mmap(nullptr, total, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    if (base == MAP_FAILED)
        {
            close(fd);
            shm_unlink(name.c_str());
            return false;
        }

    auto* h = new (base) Header();  // placement-new: properly constructs the atomics in-place
    h->block_size = block_size;
    h->block_count = block_count;
    h->state.store(static_cast<uint32_t>(StreamState::Running), std::memory_order_relaxed);
    h->version = kVersion;
    h->magic = kMagic;  // written last: nothing attaches until Create() has already returned

    ring.fd = fd;
    ring.base = base;
    ring.size = total;
    ring.header = h;
    ring.owner = true;
    ring.name = name;
    return true;
}

// ---- Consumer (attacher) side ---------------------------------------------

inline bool Attach(MappedRing& ring, const std::string& name)
{
    const int fd = shm_open(name.c_str(), O_RDWR, 0600);
    if (fd < 0)
        return false;

    struct stat st{};
    if (fstat(fd, &st) != 0 || st.st_size <= 0)
        {
            close(fd);
            return false;
        }

    void* base = mmap(nullptr, static_cast<std::size_t>(st.st_size), PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    if (base == MAP_FAILED)
        {
            close(fd);
            return false;
        }

    auto* h = reinterpret_cast<Header*>(base);
    if (h->magic != kMagic || h->version != kVersion)
        {
            munmap(base, static_cast<std::size_t>(st.st_size));
            close(fd);
            return false;
        }

    const std::size_t expected = TotalRegionSize(h->block_size, h->block_count);
    if (static_cast<std::size_t>(st.st_size) != expected)
        {
            munmap(base, static_cast<std::size_t>(st.st_size));
            close(fd);
            return false;
        }

    ring.fd = fd;
    ring.base = base;
    ring.size = static_cast<std::size_t>(st.st_size);
    ring.header = h;
    ring.owner = false;
    ring.name = name;
    return true;
}

inline void Destroy(MappedRing& ring)
{
    if (ring.base)
        munmap(ring.base, ring.size);
    if (ring.fd >= 0)
        close(ring.fd);
    if (ring.owner && !ring.name.empty())
        shm_unlink(ring.name.c_str());
    ring = MappedRing{};
}

// Publishes a terminal state and wakes a blocked consumer; does not touch
// space_event (see WriteBlock()'s abortFlag for the mirror-image case).
inline void SignalShutdown(MappedRing& ring, StreamState finalState)
{
    Header& h = *ring.header;
    h.state.store(static_cast<uint32_t>(finalState), std::memory_order_release);
    h.data_event.fetch_add(1, std::memory_order_release);
    FutexWakeAll(h.data_event);
}

// Blocks while full; abortFlag lets a caller break out (e.g. a gone consumer).
inline bool WriteBlock(MappedRing& ring, const uint8_t* data, std::size_t len, const std::atomic<bool>* abortFlag)
{
    Header& h = *ring.header;
    const uint32_t payloadCap = h.block_size - static_cast<uint32_t>(sizeof(uint64_t));
    const uint64_t toWrite = std::min(len, static_cast<std::size_t>(payloadCap));

    for (;;)
        {
            if (abortFlag && abortFlag->load(std::memory_order_acquire))
                return false;

            const uint64_t w = h.write_seq.load(std::memory_order_relaxed);
            const uint64_t r = h.read_seq.load(std::memory_order_acquire);
            if (w - r < h.block_count)
                break;

            const uint32_t expected = h.space_event.load(std::memory_order_acquire);
            // Re-check after snapshotting to close the lost-wakeup race.
            if (h.write_seq.load(std::memory_order_relaxed) - h.read_seq.load(std::memory_order_acquire) < h.block_count)
                break;
            WaitOn(h.space_event, expected, h.space_waiters);
        }

    const uint64_t w = h.write_seq.load(std::memory_order_relaxed);
    const uint32_t index = static_cast<uint32_t>(w % h.block_count);
    uint8_t* blk = BlockPtr(ring.base, index, h.block_size);
    std::memcpy(blk, &toWrite, sizeof(uint64_t));
    std::memcpy(blk + sizeof(uint64_t), data, static_cast<std::size_t>(toWrite));

    h.bytes_written.fetch_add(toWrite, std::memory_order_relaxed);
    h.write_seq.store(w + 1, std::memory_order_release);
    h.data_event.fetch_add(1, std::memory_order_release);
    WakeIfWaiting(h.data_event, h.data_waiters);
    return true;
}

// Blocks while empty and the stream hasn't ended; false once empty AND
// Stopped/EndOfStream or abortFlag.
inline bool ReadBlock(MappedRing& ring, std::vector<uint8_t>& out, const std::atomic<bool>* abortFlag = nullptr)
{
    Header& h = *ring.header;
    for (;;)
        {
            const uint64_t w = h.write_seq.load(std::memory_order_acquire);
            const uint64_t r = h.read_seq.load(std::memory_order_relaxed);
            if (w != r)
                break;

            const auto state = static_cast<StreamState>(h.state.load(std::memory_order_acquire));
            if (state == StreamState::Stopped || state == StreamState::EndOfStream)
                return false;
            if (abortFlag && abortFlag->load(std::memory_order_acquire))
                return false;

            const uint32_t expected = h.data_event.load(std::memory_order_acquire);
            if (h.write_seq.load(std::memory_order_acquire) != h.read_seq.load(std::memory_order_relaxed))
                break;
            WaitOn(h.data_event, expected, h.data_waiters);
        }

    const uint64_t r = h.read_seq.load(std::memory_order_relaxed);
    const uint32_t index = static_cast<uint32_t>(r % h.block_count);
    const uint8_t* blk = BlockPtr(ring.base, index, h.block_size);
    uint64_t len = 0;
    std::memcpy(&len, blk, sizeof(uint64_t));
    out.resize(static_cast<std::size_t>(len));
    std::memcpy(out.data(), blk + sizeof(uint64_t), static_cast<std::size_t>(len));

    h.bytes_read.fetch_add(len, std::memory_order_relaxed);
    h.read_seq.store(r + 1, std::memory_order_release);
    h.space_event.fetch_add(1, std::memory_order_release);
    WakeIfWaiting(h.space_event, h.space_waiters);
    return true;
}

}  // namespace shm_ring

#endif  // GNSS_SDR_SHM_RING_H
