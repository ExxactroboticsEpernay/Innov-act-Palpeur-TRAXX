#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>

namespace palpeur {

class MedianFilter {
 public:
  static constexpr std::size_t kMaxWindow = 21;

  explicit MedianFilter(std::size_t window_size = 5) {
    setWindowSize(window_size);
  }

  void setWindowSize(std::size_t window_size) {
    window_size = std::max<std::size_t>(1, std::min(window_size, kMaxWindow));
    if (window_size % 2 == 0) --window_size;
    window_size_ = std::max<std::size_t>(1, window_size);
    reset();
  }

  float update(float value) {
    if (!std::isfinite(value)) return last_output_;
    values_[next_index_] = value;
    next_index_ = (next_index_ + 1) % window_size_;
    if (sample_count_ < window_size_) ++sample_count_;

    auto sorted = values_;
    std::sort(sorted.begin(), sorted.begin() + sample_count_);
    if (sample_count_ % 2 == 0) {
      last_output_ = (sorted[sample_count_ / 2 - 1] +
                      sorted[sample_count_ / 2]) / 2.0f;
    } else {
      last_output_ = sorted[sample_count_ / 2];
    }
    return last_output_;
  }

  void reset() {
    values_.fill(0.0f);
    next_index_ = 0;
    sample_count_ = 0;
    last_output_ = 0.0f;
  }

  std::size_t sampleCount() const { return sample_count_; }

 private:
  std::array<float, kMaxWindow> values_{};
  std::size_t window_size_ = 5;
  std::size_t next_index_ = 0;
  std::size_t sample_count_ = 0;
  float last_output_ = 0.0f;
};

class ExponentialFilter {
 public:
  explicit ExponentialFilter(float alpha = 0.25f) { setAlpha(alpha); }

  void setAlpha(float alpha) {
    alpha_ = std::isfinite(alpha) ? std::max(0.0f, std::min(alpha, 1.0f)) : 0.25f;
    reset();
  }

  float update(float value) {
    if (!std::isfinite(value)) return output_;
    if (!initialized_) {
      output_ = value;
      initialized_ = true;
    } else {
      output_ += alpha_ * (value - output_);
    }
    return output_;
  }

  void reset() {
    output_ = 0.0f;
    initialized_ = false;
  }

 private:
  float alpha_ = 0.25f;
  float output_ = 0.0f;
  bool initialized_ = false;
};

class SlewRateLimiter {
 public:
  explicit SlewRateLimiter(float maximum_rate_per_second = 60.0f)
      : maximum_rate_per_second_(
            std::isfinite(maximum_rate_per_second) && maximum_rate_per_second > 0.0f
                ? maximum_rate_per_second
                : 60.0f) {}

  float update(float value, float delta_seconds) {
    if (!std::isfinite(value)) return output_;
    if (!initialized_ || !std::isfinite(delta_seconds) || delta_seconds <= 0.0f) {
      output_ = value;
      initialized_ = true;
      return output_;
    }
    const float maximum_delta = maximum_rate_per_second_ * delta_seconds;
    const float delta = value - output_;
    output_ += std::max(-maximum_delta, std::min(delta, maximum_delta));
    return output_;
  }

  void reset(float value = 0.0f) {
    output_ = value;
    initialized_ = false;
  }

 private:
  float maximum_rate_per_second_;
  float output_ = 0.0f;
  bool initialized_ = false;
};

}  // namespace palpeur