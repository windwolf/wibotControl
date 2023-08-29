//
// Created by zhouj on 2023/8/23.
//

#ifndef WIBOTDEVICE_DEVICE_QUADENCODER_HPP_
#define WIBOTDEVICE_DEVICE_QUADENCODER_HPP_

#include "base.hpp"
#include "lp.hpp"

namespace wibot::control {

struct RelativeEncoderConfig {
    uint32_t wrapRange;
    float    maxSpeed;
    float    maxRange;
    float    sampleTime;
    // float    lpFilterFc;
};

class RelativeEncoder : public Configurable<RelativeEncoderConfig> {
   public:
    void  updatePositionValue(uint32_t value);
    float getPosition() const {
        return position_;
    };
    float getSpeed() const {
        return speed_;
    };

   private:
    uint32_t lastValue_;

    float delta_;

    float position_;
    float speed_;

    FirstOrderLowPassFilter filteredSpeed_;

   public:
    Result apply_config() override;
};

}  // namespace wibot::control

#endif  //WIBOTDEVICE_DEVICE_QUADENCODER_HPP_
