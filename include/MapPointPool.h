/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef MAPPOINTPOOL_H
#define MAPPOINTPOOL_H

#include <cassert>
#include <cstddef>
#include <cstdlib>
#include <mutex>
#include <vector>

namespace ORB_SLAM3
{

class MapPoint;

// 64-byte alignment: satisfies Eigen SIMD requirements (max 32 bytes for AVX-512)
// and eliminates false sharing between adjacent slots accessed concurrently from
// the Tracking and LocalMapping threads.
static constexpr std::size_t kMapPointAlignment = 64;

// Fixed chunk size: 4096 slots (2^12). Amortises chunk-allocation overhead across
// large triangulation batches. Keeps (chunk_idx, slot_idx) bookkeeping trivial.
static constexpr std::size_t kMapPointChunkSize = 4096;

class MapPointPool
{
public:
    MapPointPool();
    ~MapPointPool() { DestroyAll(); }

    MapPointPool(const MapPointPool&) = delete;
    MapPointPool& operator=(const MapPointPool&) = delete;

    // Acquire: claim the next sequential slot in the current chunk (allocating a
    // new chunk when the current one is full), then placement-construct a MapPoint
    // with the forwarded arguments.
    //
    // pMP->mnPoolChunkIdx and pMP->mnPoolSlotIdx are set after construction so
    // Release() can identify the owning chunk in O(1).
    //
    // mPoolMutex is released BEFORE the MapPoint constructor runs. The ctor
    // locks mpMap->mMutexPointCreation for nNextId assignment; holding mPoolMutex
    // across that call would create a deadlock cycle between the Tracking and
    // LocalMapping threads. The slot is safe: its index is incremented while the
    // lock is still held, so no other thread can claim the same slot.
    template <typename... Args>
    MapPoint* Acquire(Args&&... args);

    // Release: read chunk/slot indices from pMP, call its destructor, then
    // decrement the chunk's live count. If live reaches 0 (and the chunk is not
    // still the active bump-allocation target — see the isActiveChunk guard in
    // MapPointPool.cc) the chunk's aligned memory block is freed to the OS; the
    // Chunk entry remains as a tombstone so chunk indices are never reused and
    // the (chunk_idx, slot_idx) mapping stays consistent for the lifetime of the
    // pool.
    //
    // NOTE: Map::EraseMapPoint() deliberately does NOT call this immediately —
    // raw MapPoint* are cached without synchronization across threads (Tracking's
    // Frame::mvpMapPoints, Map::mvpReferenceMapPoints, etc.), so destroying a
    // point the moment it's marked bad would be a use-after-free for any thread
    // still holding a stale pointer to it. Points are currently reclaimed only in
    // bulk via DestroyAll() when their owning Map is cleared/destroyed. Release()
    // remains available (and covered by MapPointPoolTests) for a future
    // synchronized/epoch-based reclamation scheme.
    void Release(MapPoint* pMP);

    // Destroy all live MapPoints and free all chunk memory. NOT thread-safe —
    // the caller must guarantee that no concurrent Acquire/Release calls are in
    // flight. Called from Map::~Map() and Map::clear().
    void DestroyAll();

    std::size_t NumChunks() const;
    std::size_t TotalAllocated() const;

private:
    struct Chunk
    {
        void* memory{nullptr};       // aligned alloc; nullptr once freed
        std::size_t next{0};         // next slot index to allocate (0..kMapPointChunkSize)
        std::size_t live{0};         // live (not-yet-released) MapPoints in this chunk
        std::vector<bool> occupied;  // per-slot liveness; set in Acquire, cleared in Release
    };

    // Appends a new chunk to mChunks. Must be called under mPoolMutex.
    void AllocateNewChunk();

    // Returns a pointer to slot slot_idx within chunk c.
    void* SlotPtr(const Chunk& c, std::size_t slot_idx) const;

    const std::size_t mSlotSize;  // sizeof(MapPoint) rounded up to kMapPointAlignment
    mutable std::mutex mPoolMutex;
    std::vector<Chunk> mChunks;  // grows monotonically; indices are never reused
};

// ---------------------------------------------------------------------------
// Template implementation (must live in the header)
// ---------------------------------------------------------------------------
template <typename... Args>
MapPoint* MapPointPool::Acquire(Args&&... args)
{
    std::size_t chunk_idx, slot_idx;
    void* slot;

    {
        std::unique_lock<std::mutex> lock(mPoolMutex);

        if (mChunks.empty() || mChunks.back().next == kMapPointChunkSize)
            AllocateNewChunk();

        // Release() must never free the chunk it's still bump-allocating from
        // (see the isActiveChunk guard in MapPointPool::Release). This assert
        // documents and enforces that invariant at the one place a violation
        // would otherwise silently placement-new into freed memory.
        assert(mChunks.back().memory != nullptr && "Active chunk was freed while still accepting allocations");

        chunk_idx = mChunks.size() - 1;
        slot_idx = mChunks.back().next++;
        mChunks.back().live++;
        mChunks.back().occupied[slot_idx] = true;
        slot = SlotPtr(mChunks[chunk_idx], slot_idx);
    }
    // Pool lock is now released. Construct the MapPoint in the reserved slot.
    // Placement new bypasses operator new entirely; our pre-aligned storage
    // satisfies all Eigen SIMD alignment requirements.
    MapPoint* pMP = ::new (slot) MapPoint(std::forward<Args>(args)...);

    // Record pool coordinates after construction (mnId is now assigned).
    pMP->mnPoolChunkIdx = chunk_idx;
    pMP->mnPoolSlotIdx = slot_idx;
    return pMP;
}

}  // namespace ORB_SLAM3

#endif  // MAPPOINTPOOL_H
