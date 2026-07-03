#pragma once

#include <vo/dispatch_types.h>
#include <vo/nav_prediction_buffer.h>

#include <algorithm>
#include <cmath>
#include <deque>
#include <functional>
#include <limits>
#include <mutex>
#include <vector>

namespace visual_odometry
{

class ImageDispatchSync
{
public:
    using MonoCallback = std::function<void(const PendingMonoFrame&, const DispatchContext&)>;
    using StereoCallback = std::function<void(const PendingStereoFrame&, const DispatchContext&)>;

    void set_nav(NavPredictionBuffer* nav) { nav_ = nav; }

    void set_mono_callback(MonoCallback cb) { mono_callback_ = std::move(cb); }
    void set_stereo_callback(StereoCallback cb) { stereo_callback_ = std::move(cb); }

    void push_frame(PendingMonoFrame frame)
    {
        DrainResult to_process;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            insert_sorted(pending_mono_, std::move(frame),
                          [](const PendingMonoFrame& a, double t) { return a.timestamp < t; });
            to_process = drain_locked();
        }
        // invoke_callbacks() takes a const& (see below), so nothing is
        // actually moved here -- call it plainly so the code doesn't imply a
        // transfer that isn't happening.
        invoke_callbacks(to_process);
    }

    void push_frame(PendingStereoFrame frame)
    {
        DrainResult to_process;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            insert_sorted(pending_stereo_, std::move(frame),
                          [](const PendingStereoFrame& a, double t) { return a.timestamp < t; });
            to_process = drain_locked();
        }
        invoke_callbacks(to_process);
    }

    void on_nav_updated()
    {
        DrainResult to_process;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            to_process = drain_locked();
        }
        invoke_callbacks(to_process);
    }

    size_t pending_mono_size() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return pending_mono_.size();
    }

    size_t pending_stereo_size() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return pending_stereo_.size();
    }

private:
    using DrainResult = std::pair<std::vector<std::pair<PendingMonoFrame, DispatchContext>>,
                                  std::vector<std::pair<PendingStereoFrame, DispatchContext>>>;

    template <typename FrameT, typename Compare>
    static void insert_sorted(std::deque<FrameT>& queue, FrameT frame, Compare cmp)
    {
        const double t = frame.timestamp;
        auto it = std::lower_bound(queue.begin(), queue.end(), t, cmp);
        queue.insert(it, std::move(frame));
    }

    DrainResult drain_locked()
    {
        DrainResult result;
        if (!nav_ || (!mono_callback_ && !stereo_callback_))
        {
            return result;
        }

        double first_nav_t = 0.0;
        if (!nav_->get_oldest_timestamp(first_nav_t))
        {
            return result;
        }

        while (!pending_mono_.empty() && pending_mono_.front().timestamp < first_nav_t)
        {
            pending_mono_.pop_front();
        }
        while (!pending_stereo_.empty() && pending_stereo_.front().timestamp < first_nav_t)
        {
            pending_stereo_.pop_front();
        }

        while (true)
        {
            const bool has_mono = !pending_mono_.empty();
            const bool has_stereo = !pending_stereo_.empty();
            if (!has_mono && !has_stereo)
            {
                break;
            }

            const bool take_mono =
                has_mono && (!has_stereo || pending_mono_.front().timestamp <= pending_stereo_.front().timestamp);
            const double t = take_mono ? pending_mono_.front().timestamp : pending_stereo_.front().timestamp;

            DispatchContext ctx;
            if (!nav_->try_get_interpolated(t, ctx.nav))
            {
                break;
            }

            if (take_mono)
            {
                result.first.emplace_back(std::move(pending_mono_.front()), ctx);
                pending_mono_.pop_front();
            }
            else
            {
                result.second.emplace_back(std::move(pending_stereo_.front()), ctx);
                pending_stereo_.pop_front();
            }
        }
        return result;
    }

    void invoke_callbacks(const DrainResult& to_process)
    {
        for (auto& p : to_process.first)
        {
            if (mono_callback_)
            {
                mono_callback_(p.first, p.second);
            }
        }
        for (auto& p : to_process.second)
        {
            if (stereo_callback_)
            {
                stereo_callback_(p.first, p.second);
            }
        }

        if (nav_)
        {
            double min_ts = std::numeric_limits<double>::infinity();
            for (const auto& p : to_process.first)
            {
                min_ts = std::min(min_ts, p.first.timestamp);
            }
            for (const auto& p : to_process.second)
            {
                min_ts = std::min(min_ts, p.first.timestamp);
            }
            if (std::isfinite(min_ts))
            {
                nav_->prune_before(min_ts);
            }
        }
    }

    NavPredictionBuffer* nav_ = nullptr;
    std::deque<PendingMonoFrame> pending_mono_;
    std::deque<PendingStereoFrame> pending_stereo_;
    MonoCallback mono_callback_;
    StereoCallback stereo_callback_;
    mutable std::mutex mutex_;
};

}  // namespace visual_odometry
