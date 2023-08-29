//
// Created by zhouj on 2023/8/28.
//

#ifndef WIBOTCONTROL_GENERATOR_SOFTTRIGGER_HPP_
#define WIBOTCONTROL_GENERATOR_SOFTTRIGGER_HPP_

#include "base.hpp"

namespace wibot::control {

enum class SoftTriggerMode : uint8_t {
    RiseEdge,
    FallEdge,
    HighLevel,
    LowLevel,
};

struct SoftTriggerConfig {
    SoftTriggerMode mode;
};

class SoftTrigger : public Configurable<SoftTriggerConfig> {
   public:
    SoftTrigger(SoftTriggerConfig config, bool initialValue);
    void update(bool value);
    bool get();
    void clear();

   private:
    bool lastValue_;
    bool eventFlag_;
};
}  // namespace wibot::control

#endif  //WIBOTCONTROL_GENERATOR_SOFTTRIGGER_HPP_
