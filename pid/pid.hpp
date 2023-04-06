#ifndef __WWCONTROL_PID_HPP__
#define __WWCONTROL_PID_HPP__

#include "framework/framework.hpp"

namespace wibot::control {
enum class PidControllerMode {
    Serial,
    Parallel,
};
struct PidControllerConfig {
    PidControllerMode mode;
    /* Controller gains */
    float Kp;
    float Ki;
    float Kd;

    /* Derivative low-pass filter time constant */
    float tau;

    /* Output limits */
    bool output_limit_enable;
    float output_limit_max;
    float output_limit_min;

    /* Integrator limits */
    bool integrator_limit_enable;
    float integrator_limit_max;
    float integrator_limit_min;

    /* Sample time (in seconds) */
    float sample_time;
};

class PidController : public Configurable<PidControllerConfig> {
   public:
    PidController();
    Result apply_config() override;
    void reset();
    float update(float setpoint, float measurement);

   protected:
   private:
    float update_serial(float setpoint, float measurement);
    float update_parallel(float setpoint, float measurement);

    /* Controller "memory" */
    float _integrator;
    float _prevError;       /* Required for _integrator */
    float _differentiator;
    float _prevMeasurement; /* Required for _differentiator */
};

}  // namespace wibot::control

#endif  // __WWCONTROL_PID_HPP__
