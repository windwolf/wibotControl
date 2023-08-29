//
// Created by zhouj on 2023/8/28.
//

#include "softtrigger.hpp"

namespace wibot::control {

SoftTrigger::SoftTrigger(SoftTriggerConfig cfg, bool initialValue)
    : lastValue_(initialValue), eventFlag_(false) {
    config = cfg;
}
void SoftTrigger::update(bool value) {
    eventFlag_ = (config.mode == SoftTriggerMode::RiseEdge && !lastValue_ && value) ||
                 (config.mode == SoftTriggerMode::FallEdge && lastValue_ && !value) ||
                 (config.mode == SoftTriggerMode::HighLevel && value) ||
                 (config.mode == SoftTriggerMode::LowLevel && !value);
    lastValue_ = value;
}

bool SoftTrigger::get() {
    return eventFlag_;
}

void SoftTrigger::clear() {
    eventFlag_ = false;
}

}  // namespace wibot::control
