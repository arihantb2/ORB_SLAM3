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

#include "MapPointPool.h"
#include "MapPoint.h"

#include <cassert>
#include <cstdlib>
#include <stdexcept>

namespace ORB_SLAM3
{

MapPointPool::MapPointPool()
    : mSlotSize(((sizeof(MapPoint) + kMapPointAlignment - 1) / kMapPointAlignment) * kMapPointAlignment)
{
    // Verify that our free-slot bookkeeping fits inside a MapPoint-sized slot
    // (this is trivially true for any non-trivial class, but kept as documentation).
    static_assert(sizeof(void*) <= sizeof(MapPoint), "Slot must be large enough to hold a pointer for bookkeeping");
}

void MapPointPool::AllocateNewChunk()
{
    // Must be called under mPoolMutex.
    //
    // std::aligned_alloc requires size to be a multiple of alignment.
    // kMapPointChunkSize * mSlotSize is a multiple of mSlotSize which is itself
    // a multiple of kMapPointAlignment, so the requirement is always satisfied.
    const std::size_t alloc_size = kMapPointChunkSize * mSlotSize;
    assert(alloc_size % kMapPointAlignment == 0);

    void* mem = std::aligned_alloc(kMapPointAlignment, alloc_size);
    if (!mem)
        throw std::bad_alloc();

    Chunk chunk;
    chunk.memory = mem;
    chunk.next = 0;
    chunk.live = 0;
    chunk.occupied.assign(kMapPointChunkSize, false);
    mChunks.push_back(std::move(chunk));
}

void* MapPointPool::SlotPtr(const Chunk& c, std::size_t slot_idx) const
{
    return static_cast<char*>(c.memory) + slot_idx * mSlotSize;
}

void MapPointPool::Release(MapPoint* pMP)
{
    // Read coordinates before the destructor runs (the destructor may clear
    // member variables, though in practice mnPoolChunkIdx/Slot are plain ints).
    const std::size_t chunk_idx = pMP->mnPoolChunkIdx;
    const std::size_t slot_idx = pMP->mnPoolSlotIdx;

    // Destruct outside the pool lock: the destructor may acquire per-MapPoint
    // mutexes and map locks; holding mPoolMutex across that would risk deadlock.
    pMP->~MapPoint();

    std::unique_lock<std::mutex> lock(mPoolMutex);

    assert(chunk_idx < mChunks.size());
    Chunk& chunk = mChunks[chunk_idx];
    assert(chunk.memory != nullptr && "Releasing into an already-freed chunk");
    assert(slot_idx < chunk.next);
    assert(chunk.occupied[slot_idx] && "Double-release detected");

    chunk.occupied[slot_idx] = false;
    --chunk.live;

    if (chunk.live == 0)
    {
        // All MapPoints in this chunk have been retired — free the memory block.
        // The Chunk entry remains as a tombstone (memory == nullptr) so that
        // chunk indices are never reused and the (chunk_idx, slot_idx) mapping
        // stays consistent.
        std::free(chunk.memory);
        chunk.memory = nullptr;
    }
}

void MapPointPool::DestroyAll()
{
    // No lock — caller guarantees exclusivity (called from ~Map / Map::clear
    // where no concurrent Acquire/Release can be in flight).
    for (Chunk& chunk : mChunks)
    {
        if (chunk.memory == nullptr)
            continue;  // already freed

        for (std::size_t i = 0; i < chunk.next; ++i)
        {
            if (chunk.occupied[i])
            {
                MapPoint* pMP = static_cast<MapPoint*>(SlotPtr(chunk, i));
                pMP->~MapPoint();
                chunk.occupied[i] = false;
            }
        }
        std::free(chunk.memory);
        chunk.memory = nullptr;
        chunk.live = 0;
    }
    mChunks.clear();
}

std::size_t MapPointPool::NumChunks() const
{
    std::unique_lock<std::mutex> lock(mPoolMutex);
    return mChunks.size();
}

std::size_t MapPointPool::TotalAllocated() const
{
    std::unique_lock<std::mutex> lock(mPoolMutex);
    std::size_t total = 0;
    for (const Chunk& c : mChunks)
        total += c.next;
    return total;
}

}  // namespace ORB_SLAM3
