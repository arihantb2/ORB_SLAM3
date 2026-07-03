#pragma once

#include <vo/dispatch_types.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <algorithm>
#include <mutex>
#include <vector>

namespace visual_odometry
{

class NavPredictionBuffer
{
public:
    void push(PoseStamped pose)
    {
        const double ts = pose.first;
        std::lock_guard<std::mutex> lock(mutex_);
        if (buffer_.empty() || ts >= buffer_.back().first)
        {
            buffer_.push_back(std::move(pose));
        }
        else
        {
            auto it = std::lower_bound(buffer_.begin(), buffer_.end(), ts,
                                       [](const PoseStamped& a, double t) { return a.first < t; });
            buffer_.insert(it, std::move(pose));
        }
    }

    bool try_get_interpolated(double timestamp, NavInterpolationResult& out) const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (buffer_.size() < 2)
        {
            return false;
        }
        auto upper_it = std::lower_bound(buffer_.begin(), buffer_.end(), timestamp,
                                         [](const PoseStamped& a, double t) { return a.first < t; });
        // A query exactly at the oldest sample's timestamp lower_bounds to
        // begin() (since begin()->first >= timestamp), which the "before the
        // buffer" check below would otherwise reject even though the sample
        // exists. Treat it as the lower bracket of [begin(), begin()+1).
        if (upper_it == buffer_.begin() && upper_it != buffer_.end() && upper_it->first == timestamp)
        {
            ++upper_it;
        }
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
        out = interpolate(lower, upper, alpha, dt);
        return true;
    }

    void prune_before(double timestamp)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        auto it = std::lower_bound(buffer_.begin(), buffer_.end(), timestamp,
                                   [](const PoseStamped& a, double t) { return a.first < t; });
        if (it != buffer_.begin())
        {
            // Keep one sample at or before `timestamp`: it is the lower bracket
            // that a future try_get_interpolated() query in (that sample's
            // timestamp, timestamp] still needs. Erasing it (as this used to do)
            // could leave the buffer's oldest sample newer than `timestamp`,
            // making try_get_interpolated() and get_oldest_timestamp() report the
            // nav horizon as having jumped forward, silently dropping any pending
            // frame that timestamp could otherwise have bracketed.
            --it;
            buffer_.erase(buffer_.begin(), it);
        }
    }

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
    static NavInterpolationResult interpolate(const PoseStamped& lower, const PoseStamped& upper, double alpha,
                                              double dt)
    {
        NavInterpolationResult result;
        result.bracketing_dt = dt;
        const Eigen::Vector3f p0 = lower.second.block<3, 1>(0, 3);
        const Eigen::Vector3f p1 = upper.second.block<3, 1>(0, 3);
        const Eigen::Vector3f p = (1.0f - static_cast<float>(alpha)) * p0 + static_cast<float>(alpha) * p1;
        Eigen::Quaternionf q0(lower.second.block<3, 3>(0, 0));
        Eigen::Quaternionf q1(upper.second.block<3, 3>(0, 0));
        q0.normalize();
        q1.normalize();
        const Eigen::Quaternionf q = q0.slerp(static_cast<float>(alpha), q1).normalized();
        result.pose = Eigen::Matrix4f::Identity();
        result.pose.block<3, 3>(0, 0) = q.toRotationMatrix();
        result.pose.block<3, 1>(0, 3) = p;
        return result;
    }

    mutable std::mutex mutex_;
    std::vector<PoseStamped> buffer_;
};

}  // namespace visual_odometry
