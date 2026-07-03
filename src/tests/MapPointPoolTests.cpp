// Regression tests for MapPointPool chunk lifecycle and Map's MapPoint
// reclamation policy.
//
// Background: MapPointPool::Release() used to free a chunk's aligned memory
// block as soon as its live count reached zero, even when that chunk was
// still MapPointPool::Acquire()'s bump-allocation target (i.e. its `next`
// index had not yet reached kMapPointChunkSize). The next Acquire() would
// then placement-new a MapPoint into freed memory. Separately, Map::
// EraseMapPoint() called Release() (and therefore the MapPoint destructor)
// synchronously, even though raw MapPoint* are cached without
// synchronization across the Tracking and LocalMapping threads (e.g. in
// Frame::mvpMapPoints) — a classic use-after-free.

#include "Map.h"
#include "MapPoint.h"
#include "MapPointPool.h"
#include "KeyFrame.h"

#include <gtest/gtest.h>
#include <Eigen/Core>

#include <memory>
#include <unordered_set>
#include <vector>

using namespace ORB_SLAM3;

namespace
{

// KeyFrame's default constructor leaves mnId uninitialized (it's only set by
// the Frame-based constructor); MapPoint's constructor reads pRefKF->mnId, so
// tests must set it explicitly to avoid reading uninitialized memory.
KeyFrame makeMinimalKeyFrame(unsigned long id)
{
    KeyFrame kf;
    kf.mnId = id;
    return kf;
}

}  // namespace

// ============================================================================
// MapPointPool — chunk lifecycle
// ============================================================================

TEST(MapPointPool, AcquireReleaseCycleDoesNotCorruptSubsequentAllocation)
{
    // Drain a chunk to live == 0 while it is still the active (not-full) chunk,
    // then verify the next Acquire() lands in valid, freshly-allocated memory
    // rather than the block that was (under the bug) already freed.
    MapPointPool pool;
    Map map;
    KeyFrame kf = makeMinimalKeyFrame(0);

    std::vector<MapPoint*> points;
    const int kBatch = 16;  // well under kMapPointChunkSize, so the chunk stays "active"
    for (int i = 0; i < kBatch; ++i)
    {
        points.push_back(pool.Acquire(Eigen::Vector3f(static_cast<float>(i), 0.f, 0.f), &kf, &map));
    }
    ASSERT_EQ(pool.NumChunks(), 1u);
    ASSERT_EQ(pool.TotalAllocated(), static_cast<std::size_t>(kBatch));

    for (MapPoint* pMP : points)
    {
        pool.Release(pMP);
    }

    // Under the bug, this Acquire() would placement-new into memory already
    // freed by the loop above. Exercise the returned object fully (not just
    // check it's non-null) so a heap corruption/ASan failure would surface.
    MapPoint* pAfter = pool.Acquire(Eigen::Vector3f(99.f, 98.f, 97.f), &kf, &map);
    ASSERT_NE(pAfter, nullptr);
    const Eigen::Vector3f pos = pAfter->GetWorldPos();
    EXPECT_FLOAT_EQ(pos.x(), 99.f);
    EXPECT_FLOAT_EQ(pos.y(), 98.f);
    EXPECT_FLOAT_EQ(pos.z(), 97.f);

    pool.Release(pAfter);
}

TEST(MapPointPool, ChunkBoundaryAllocationRemainsValid)
{
    // Exercise Acquire() crossing from a full chunk into a freshly allocated
    // one: every point must be distinct, valid memory, and releasing
    // everything afterwards (including the fully drained first chunk) must
    // not corrupt a subsequent allocation.
    MapPointPool pool;
    Map map;
    KeyFrame kf = makeMinimalKeyFrame(0);

    std::vector<MapPoint*> points;
    points.reserve(kMapPointChunkSize + 1);
    for (std::size_t i = 0; i < kMapPointChunkSize + 1; ++i)
    {
        points.push_back(pool.Acquire(Eigen::Vector3f(static_cast<float>(i), 0.f, 0.f), &kf, &map));
    }
    ASSERT_EQ(pool.NumChunks(), 2u);

    const std::unordered_set<MapPoint*> uniquePtrs(points.begin(), points.end());
    EXPECT_EQ(uniquePtrs.size(), points.size()) << "Acquire returned a duplicate address";

    for (std::size_t i = 0; i < points.size(); ++i)
    {
        EXPECT_FLOAT_EQ(points[i]->GetWorldPos().x(), static_cast<float>(i));
    }

    for (MapPoint* pMP : points)
    {
        pool.Release(pMP);
    }

    MapPoint* pAfter = pool.Acquire(Eigen::Vector3f(-1.f, -1.f, -1.f), &kf, &map);
    ASSERT_NE(pAfter, nullptr);
    EXPECT_FLOAT_EQ(pAfter->GetWorldPos().x(), -1.f);
    pool.Release(pAfter);
}

// ============================================================================
// Map::EraseMapPoint — deferred reclamation (no UAF for stale raw pointers)
// ============================================================================

TEST(MapLifetime, ErasedMapPointRemainsValidUntilMapDestruction)
{
    auto map = std::make_unique<Map>();
    KeyFrame kf = makeMinimalKeyFrame(0);

    MapPoint* pMP = map->CreateMapPoint(Eigen::Vector3f(1.f, 2.f, 3.f), &kf);
    ASSERT_NE(pMP, nullptr);
    map->AddMapPoint(pMP);
    ASSERT_EQ(map->MapPointsInMap(), 1u);

    // SetBadFlag() has no observations to clean up on a freshly created point,
    // so this exercises exactly what LocalMapping::MapPointCulling does: mark
    // bad, then Map::EraseMapPoint().
    pMP->SetBadFlag();

    EXPECT_EQ(map->MapPointsInMap(), 0u) << "erased point must be removed from the live set";

    // The critical assertion: a stale raw pointer obtained before the erase
    // (the same pattern Tracking::CheckReplacedInLastFrame uses on
    // mLastFrame.mvpMapPoints) must still be safe to call methods on. Prior to
    // the fix, EraseMapPoint() destructed pMP synchronously here, so these
    // calls would run on a destroyed object.
    EXPECT_TRUE(pMP->isBad());
    EXPECT_NO_THROW(pMP->GetWorldPos());

    // Points are only actually reclaimed in bulk when the owning Map is
    // cleared or destroyed.
    map.reset();
}

TEST(MapLifetime, ClearDestroysErasedAndLiveMapPoints)
{
    // Map::clear() must still reclaim points that were erased (and therefore
    // deferred, not freed, by EraseMapPoint) as well as points still live in
    // mspMapPoints -- both are only resident in the pool, not in mspMapPoints,
    // once erased.
    Map map;
    KeyFrame kf = makeMinimalKeyFrame(0);

    MapPoint* pErased = map.CreateMapPoint(Eigen::Vector3f(1.f, 0.f, 0.f), &kf);
    map.AddMapPoint(pErased);
    MapPoint* pLive = map.CreateMapPoint(Eigen::Vector3f(2.f, 0.f, 0.f), &kf);
    map.AddMapPoint(pLive);

    pErased->SetBadFlag();
    ASSERT_EQ(map.MapPointsInMap(), 1u);

    // Must not crash or leak: DestroyAll() (invoked by clear()) walks the pool
    // directly, so it must find and destroy pErased even though it is no
    // longer in mspMapPoints.
    map.clear();
    EXPECT_EQ(map.MapPointsInMap(), 0u);
}
