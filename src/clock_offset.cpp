// clock_offset.cpp  (v1.0.0)
//
// See clock_offset.h for the full rationale and stated limitations.

#include "clock_offset.h"

#include <deque>
#include <utility>

namespace {

// 30s chosen as a middle ground: long enough that a real sample close to
// the true offset floor is likely to appear in the window, short enough
// that the estimate re-centers reasonably quickly after an offset jump.
// Not tuned against real data beyond the investigation in
// controller-bridge-motor-inference memory - a placeholder like this
// project's other unvalidated constants (THETA_LIMIT, ANCHOR_DISTANCE, ...).
constexpr double kWindowSeconds = 30.0;

// (this-process-time, frame_transfer_time) pairs, oldest first. Frame rate
// here is modest (tens of Hz), so even a 30s window holds at most a few
// thousand entries - a plain linear scan for the minimum on each call is
// microseconds, negligible next to the ~20ms control-loop budget, and
// stays obviously correct rather than reaching for a monotonic-deque
// O(1) minimum tracker this workload doesn't need.
std::deque<std::pair<double, double>> g_samples;

}  // namespace

void clock_offset_reset(void) {
    g_samples.clear();
}

double clock_offset_correct(double frame_transfer_time, double now) {
    g_samples.emplace_back(now, frame_transfer_time);
    while (!g_samples.empty() && now - g_samples.front().first > kWindowSeconds) {
        g_samples.pop_front();
    }

    double rolling_min = g_samples.front().second;
    for (const auto& sample : g_samples) {
        if (sample.second < rolling_min) {
            rolling_min = sample.second;
        }
    }

    return frame_transfer_time - rolling_min;
}
