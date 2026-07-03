#include <vo/nav_prediction_buffer.h>

#include <cassert>
#include <cmath>
#include <iostream>
#include <utility>

using visual_odometry::NavInterpolationResult;
using visual_odometry::NavPredictionBuffer;
using visual_odometry::PoseStamped;

namespace
{
PoseStamped make_pose(double t, float x)
{
    Eigen::Matrix4f m = Eigen::Matrix4f::Identity();
    m(0, 3) = x;
    return std::make_pair(t, m);
}
}  // namespace

static void test_prune_before_retains_lower_bracket_for_pending_frame()
{
    // Regression test: prune_before() used to erase every sample strictly
    // before the prune timestamp, including the lower interpolation bracket a
    // still-pending frame needs. This reproduces the scenario in
    // ImageDispatchSync::invoke_callbacks: frames at t=5,6 dispatch against
    // the [0,10] bracket, then prune_before(5) runs (the minimum dispatched
    // timestamp) -- a frame at t=7 must still be interpolatable afterwards.
    NavPredictionBuffer buf;
    buf.push(make_pose(0.0, 0.0f));
    buf.push(make_pose(10.0, 10.0f));
    buf.push(make_pose(20.0, 20.0f));

    buf.prune_before(5.0);

    NavInterpolationResult result;
    const bool ok = buf.try_get_interpolated(7.0, result);
    assert(ok && "frame at t=7 should still bracket between the retained t=0 and t=10 samples");
    assert(std::abs(result.pose(0, 3) - 7.0f) < 1e-5f);
}

static void test_prune_before_still_drops_genuinely_stale_samples()
{
    // Sanity check that the fix doesn't regress pruning altogether: samples
    // strictly older than the retained lower bracket must still be dropped.
    NavPredictionBuffer buf;
    buf.push(make_pose(0.0, 0.0f));
    buf.push(make_pose(10.0, 10.0f));
    buf.push(make_pose(20.0, 20.0f));
    buf.push(make_pose(30.0, 30.0f));
    buf.push(make_pose(40.0, 40.0f));

    buf.prune_before(25.0);

    double oldest = -1.0;
    const bool ok = buf.get_oldest_timestamp(oldest);
    assert(ok);
    assert(std::abs(oldest - 20.0) < 1e-9);
}

static void test_interpolate_at_oldest_sample_timestamp()
{
    // Regression test: querying exactly at buffer_.front()'s timestamp used to
    // fail because lower_bound() lands on begin(), which the "query is before
    // the buffer" guard rejected even though the sample exists there.
    NavPredictionBuffer buf;
    buf.push(make_pose(0.0, 0.0f));
    buf.push(make_pose(10.0, 10.0f));

    NavInterpolationResult result;
    const bool ok = buf.try_get_interpolated(0.0, result);
    assert(ok && "query exactly at the oldest sample's timestamp should succeed");
    assert(std::abs(result.pose(0, 3) - 0.0f) < 1e-5f);
}

static void test_interpolate_before_and_after_buffer_still_fails()
{
    NavPredictionBuffer buf;
    buf.push(make_pose(0.0, 0.0f));
    buf.push(make_pose(10.0, 10.0f));

    NavInterpolationResult result;
    assert(!buf.try_get_interpolated(-1.0, result));
    assert(!buf.try_get_interpolated(10.0001, result));
}

int main()
{
    test_prune_before_retains_lower_bracket_for_pending_frame();
    test_prune_before_still_drops_genuinely_stale_samples();
    test_interpolate_at_oldest_sample_timestamp();
    test_interpolate_before_and_after_buffer_still_fails();

    std::cout << "All NavPredictionBuffer tests passed." << std::endl;
    return 0;
}
