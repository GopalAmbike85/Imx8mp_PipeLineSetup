// controller_pipeline.hpp  (v1.0.0)
//
// Portable (non-TFLite) pipeline math for the controller test harness:
// joystick (x, y) -> (theta, phi) -> (u, v) feature space -> motion-based
// history window -> int8 quantize/dequantize -> cascaded low-pass filter.
//
// Copied verbatim from the cnr-model-test-harness repo's
// controller_test_harness/controller_pipeline.hpp (self-tested there
// against a pure-Python reference across all 1151 rows of
// controller_data_log.csv - see that repo's README.md) for reuse by
// motor_inference.cpp in this project. Keep the two copies in sync if
// either is changed - there is no shared library between the two repos.
//
// Header-only and free of any TensorFlow Lite / NPU dependency so it can
// be compiled and unit-tested on any host, independent of TFLite-specific
// code.
//
// Numerical note: NumPy's np.round() (used throughout the Python reference
// for quantization) rounds half-way values to the nearest EVEN integer
// ("banker's rounding"), not away from zero like std::round()/std::lround().
// RoundHalfToEven() below replicates that exactly so C++ quantization
// matches the Python reference bit-for-bit, not just "close enough".

#ifndef CONTROLLER_TEST_HARNESS_CONTROLLER_PIPELINE_HPP_
#define CONTROLLER_TEST_HARNESS_CONTROLLER_PIPELINE_HPP_

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <deque>
#include <vector>

namespace cnr_pipeline {

inline constexpr double kPi = 3.14159265358979323846;

inline double DegToRad(double deg) { return deg * (kPi / 180.0); }
inline double RadToDeg(double rad) { return rad * (180.0 / kPi); }

// Matches NumPy's np.round(): round half-way values to the nearest even
// integer, rather than away from zero.
inline int32_t RoundHalfToEven(double v) {
    double floor_v = std::floor(v);
    double frac = v - floor_v;
    int32_t fl = static_cast<int32_t>(floor_v);
    if (frac < 0.5) return fl;
    if (frac > 0.5) return fl + 1;
    return (fl % 2 == 0) ? fl : fl + 1;  // exactly .5 -> round to even
}

// q = round(x / scale) + zero_point, clipped to the int8 range.
inline int8_t Quantize(float x, float scale, int zero_point) {
    int32_t q = RoundHalfToEven(static_cast<double>(x) / scale) + zero_point;
    q = std::max(-128, std::min(127, q));
    return static_cast<int8_t>(q);
}

inline float Dequantize(int8_t q, float scale, int zero_point) {
    return scale * (static_cast<float>(q) - static_cast<float>(zero_point));
}

// Converts joystick (x, y), each assumed in [-1.0, 1.0], to (theta, phi).
// theta_limit: max bend angle (radians) at full joystick deflection.
inline void XyToAngles(double x, double y, double theta_limit,
                        double& theta, double& phi) {
    phi = std::atan2(y, x);
    double radius = std::sqrt(x * x + y * y);
    radius = std::clamp(radius, 0.0, 1.0);
    theta = radius * theta_limit;
}

// Transforms (theta, phi) into the model's (u, v) feature space (degrees).
inline void FeatureTransform(double theta, double phi, double& u, double& v) {
    double theta_deg = RadToDeg(theta);
    u = std::cos(phi) * theta_deg;
    v = std::sin(phi) * theta_deg;
}

// Maintains a (seq_len, 2) window of [u, v] anchors sampled by cumulative
// motion distance, matching Python's HistoryWindowBuilder:
//   - A new anchor is stored only once the accumulated (u, v)-space arc
//     length since the last stored anchor exceeds anchor_distance.
//   - The current sample is always appended as the final window element,
//     and is only considered for anchor storage AFTER the window is built,
//     so it can never appear twice in the same window.
//   - Before seq_len - 1 anchors have accumulated, the window is padded by
//     repeating the first real [u, v] sample ("repeat_first" pad mode -
//     the only mode this port implements, matching the Python reference's
//     default).
class HistoryWindowBuilder {
   public:
    HistoryWindowBuilder(int seq_len, double anchor_distance)
        : seq_len_(seq_len), anchor_distance_(anchor_distance) {}

    std::vector<std::array<float, 2>> Build(double u, double v) {
        std::array<float, 2> cur = {static_cast<float>(u),
                                     static_cast<float>(v)};
        if (!has_first_) {
            first_sample_ = cur;
            prev_sample_ = cur;
            has_first_ = true;
        }

        const int needed = seq_len_ - 1;
        std::vector<std::array<float, 2>> window;
        window.reserve(seq_len_);

        if (static_cast<int>(hist_.size()) < needed) {
            const int pad_count = needed - static_cast<int>(hist_.size());
            for (int i = 0; i < pad_count; ++i) window.push_back(first_sample_);
            for (const auto& a : hist_) window.push_back(a);
        } else {
            // hist_ already holds at most `needed` entries (enforced below),
            // so this branch just takes all of them in order.
            for (const auto& a : hist_) window.push_back(a);
        }
        window.push_back(cur);

        double dx = cur[0] - prev_sample_[0];
        double dy = cur[1] - prev_sample_[1];
        accum_dist_ += std::sqrt(dx * dx + dy * dy);
        prev_sample_ = cur;
        if (accum_dist_ >= anchor_distance_) {
            hist_.push_back(cur);
            if (static_cast<int>(hist_.size()) > needed) hist_.pop_front();
            accum_dist_ = 0.0;
        }

        return window;  // (seq_len, 2)
    }

    // Clears all accumulated motion history and re-arms first-sample
    // padding, as if no samples had ever been seen. Not present in the
    // original harness (which only ever processes one continuous log) -
    // added here because a live controller session can disconnect and
    // reconnect, and a stale trajectory from a previous session should not
    // leak into a new one.
    void Reset() {
        hist_.clear();
        has_first_ = false;
        accum_dist_ = 0.0;
    }

   private:
    int seq_len_;
    double anchor_distance_;
    std::deque<std::array<float, 2>> hist_;
    std::array<float, 2> first_sample_{};
    std::array<float, 2> prev_sample_{};
    double accum_dist_ = 0.0;
    bool has_first_ = false;
};

// N-stage cascaded exponential moving average low-pass filter, matching
// Python's CascadedEMA (fixed cutoff-frequency form only, since that is
// what the harness uses).
class CascadedEMA {
   public:
    CascadedEMA(int dim, double ts, double fc, int stages)
        : dim_(dim), stages_(stages) {
        double a = 1.0 - std::exp(-2.0 * kPi * fc * ts);
        alpha_.assign(stages_, a);
        y_.assign(stages_, std::vector<double>(dim_, 0.0));
    }

    void Reset(const std::vector<double>& x0) {
        for (auto& yi : y_) yi = x0;
    }

    std::vector<double> Step(const std::vector<double>& x) {
        std::vector<double> s = x;
        for (int i = 0; i < stages_; ++i) {
            for (int d = 0; d < dim_; ++d) {
                y_[i][d] += alpha_[i] * (s[d] - y_[i][d]);
            }
            s = y_[i];
        }
        return s;
    }

   private:
    int dim_;
    int stages_;
    std::vector<double> alpha_;
    std::vector<std::vector<double>> y_;
};

}  // namespace cnr_pipeline

#endif  // CONTROLLER_TEST_HARNESS_CONTROLLER_PIPELINE_HPP_
