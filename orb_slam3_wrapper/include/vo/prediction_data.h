#pragma once

#include <algorithm>
#include <cstddef>
#include <mutex>
#include <utility>
#include <vector>

namespace visual_odometry
{
/**
 * Time-ordered buffer of messages of type T with interpolation at query time.
 * Timestamp and interpolation are provided by the caller (e.g. lambdas).
 * No knowledge of images, callbacks, or other sensors.
 */
template <typename T, typename InterpResult, typename TimestampExtractor, typename Interpolator>
class PredictionData
{
public:
    explicit PredictionData(TimestampExtractor get_timestamp, Interpolator interpolate)
        : get_timestamp_(std::move(get_timestamp)), interpolate_(std::move(interpolate))
    {
    }

    void push(T msg)
    {
        const double timestamp = get_timestamp_(msg);
        std::lock_guard<std::mutex> lock(mutex_);
        if (buffer_.empty() || timestamp >= buffer_.back().first)
        {
            buffer_.emplace_back(timestamp, std::move(msg));
        }
        else
        {
            auto it = std::lower_bound(buffer_.begin(), buffer_.end(), timestamp,
                                       [](const std::pair<double, T>& a, double t) { return a.first < t; });
            buffer_.insert(it, std::make_pair(timestamp, std::move(msg)));
        }
    }

    bool try_get_interpolated(double timestamp, InterpResult& out_result) const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (buffer_.size() < 2)
        {
            return false;
        }
        auto upper_it = std::lower_bound(buffer_.begin(), buffer_.end(), timestamp,
                                         [](const std::pair<double, T>& a, double t) { return a.first < t; });
        if (upper_it == buffer_.begin() || upper_it == buffer_.end())
        {
            return false;
        }
        const auto& lower = *(upper_it - 1);
        const auto& upper = *upper_it;
        const double t0 = lower.first;
        const double t1 = upper.first;
        const double dt = t1 - t0;
        if (dt <= 0.0)
        {
            return false;
        }
        const double alpha = (timestamp - t0) / dt;
        out_result = interpolate_(lower.second, upper.second, alpha, dt);
        return true;
    }

    void prune_before(double timestamp)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        auto it = std::lower_bound(buffer_.begin(), buffer_.end(), timestamp,
                                   [](const std::pair<double, T>& a, double t) { return a.first < t; });
        if (it != buffer_.begin())
        {
            buffer_.erase(buffer_.begin(), it);
        }
    }

    /** Returns true if buffer has at least one message and sets out to oldest timestamp. */
    bool get_oldest_timestamp(double& out) const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (buffer_.empty())
        {
            return false;
        }
        out = buffer_.front().first;
        return true;
    }

    size_t size() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return buffer_.size();
    }

private:
    TimestampExtractor get_timestamp_;
    Interpolator interpolate_;
    mutable std::mutex mutex_;
    std::vector<std::pair<double, T>> buffer_;
};

}  // namespace visual_odometry
