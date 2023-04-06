#include "pid.hpp"

namespace wibot::control {

void PidController::reset() {
    /* Clear controller variables */
    _integrator = 0.0f;
    _prevError  = 0.0f;

    _differentiator  = 0.0f;
    _prevMeasurement = 0.0f;
};

float PidController::update(float setpoint, float measurement) {
    switch (config.mode) {
        case PidControllerMode::Serial:
            return update_serial(setpoint, measurement);
        case PidControllerMode::Parallel:
            return update_parallel(setpoint, measurement);
        default:
            return 0.0f;
    }
};

/**
 * @brief y = Kp + Ki/s + Kd*s/(s*tua+1)
 * p_k = Kp*e_k;
 * i_k = Ki*T/2*(e_k+e_k1) + i_k1;
 * d_k = 2*Kd/(2*tau+T)*(e_k-e_k1) + (2*tau-T)/(2*tau+T)*d_k1;
 * o_k = p_k + i_k + d_k;
 * @param setpoint
 * @param measurement
 * @return float
 */
float PidController::update_parallel(float setpoint, float measurement) {
    float out;

    /*
     * Error signal
     */
    float error = setpoint - measurement;

    /*
     * Proportional
     */
    float proportional = config.Kp * error;

    /*
     * Integral
     */
    _integrator = _integrator + 0.5f * config.Ki * config.sample_time * (error + _prevError);

    /* Anti-wind-up via _integrator clamping */
    if (config.integrator_limit_enable) {
        if (_integrator > config.integrator_limit_max) {
            _integrator = config.integrator_limit_max;
        } else if (_integrator < config.integrator_limit_min) {
            _integrator = config.integrator_limit_min;
        }
    }

    /*
     * Derivative (band-limited _differentiator)
     */

    _differentiator =
        -(2.0f * config.Kd *
              (measurement - _prevMeasurement) /* Note: derivative on measurement, therefore minus
                                                  sign in front of equation! */
          + (2.0f * config.tau - config.sample_time) * _differentiator) /
        (2.0f * config.tau + config.sample_time);

    /*
     * Compute output and apply limits
     */
    out = proportional + _integrator + _differentiator;

    if (config.output_limit_enable) {
        if (out > config.output_limit_max) {
            out = config.output_limit_max;
        } else if (out < config.output_limit_min) {
            out = config.output_limit_min;
        }
    }

    /* Store error and measurement for later use */
    _prevError       = error;
    _prevMeasurement = measurement;

    /* Return controller output */
    return out;
}

/**
 * @brief y = Kp(1 + Ki/s + Kd*s/(s*tua+1))
 * p_k = e_k;
 * i_k = Ki*T/2*(e_k+e_k1) + i_k1;
 * d_k = 2*Kd/(2*tau+T)*(e_k-e_k1) + (2*tau-T)/(2*tau+T)*d_k1;
 * o_k = Kp*(p_k + i_k + d_k);
 * @param setpointcontext.
 * @param measurement
 * @return float
 */
float PidController::update_serial(float setpoint, float measurement) {
    float out;

    /*
     * Error signal
     */
    float error = setpoint - measurement;

    /*
     * Proportional
     */
    float proportional = error;

    /*
     * Integral
     */
    _integrator = _integrator + 0.5f * config.Ki * config.sample_time * (error + _prevError);

    if (config.integrator_limit_enable) {
        /* Anti-wind-up via _integrator clamping */
        if (_integrator > config.integrator_limit_max) {
            _integrator = config.integrator_limit_max;
        } else if (_integrator < config.integrator_limit_min) {
            _integrator = config.integrator_limit_min;
        }
    }

    /*
     * Derivative (band-limited _differentiator)
     */

    _differentiator =
        -(2.0f * config.Kd * (measurement - _prevMeasurement)
          /* Note: derivative on measurement, therefore minus sign in front of equation! */
          + (2.0f * config.tau - config.sample_time) * _differentiator) /
        (2.0f * config.tau + config.sample_time);

    /*
     * Compute output and apply limits
     */
    out = config.Kp * (proportional + _integrator + _differentiator);

    if (config.output_limit_enable) {
        if (out > config.output_limit_max) {
            out = config.output_limit_max;
        } else if (out < config.output_limit_min) {
            out = config.output_limit_min;
        }
    }

    /* Store error and measurement for later use */
    _prevError       = error;
    _prevMeasurement = measurement;

    /* Return controller output */
    return out;
}
PidController::PidController() {
    reset();
}
Result PidController::apply_config() {
    return Configurable::apply_config();
}

}  // namespace wibot::control
