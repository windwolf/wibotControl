//
// Created by zhouj on 2023/8/23.
//

#include "RelativeEncoder.hpp"

namespace wibot::control {

void RelativeEncoder::updatePositionValue(uint32_t value) {
    int32_t delta = value - lastValue_;
    delta_        = (uint32_t)std::abs(delta) < config.wrapRange / 2 ? delta : -delta;
    position_ += delta_;
    speed_     = filteredSpeed_.filter(delta_ / config.sampleTime);
    lastValue_ = value;
}

Result RelativeEncoder::apply_config() {
    filteredSpeed_.config.sample_time = config.sampleTime;
    // According to the formula BW*RiseTime=0.35,
    filteredSpeed_.config.cutoff_freq = 0.35f / (config.maxRange * 0.8f / config.maxSpeed);
    filteredSpeed_.reset();
    return Result::OK;
}

}  // namespace wibot::control
