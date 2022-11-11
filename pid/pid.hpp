#ifndef __WWCONTROL_PID_HPP__
#define __WWCONTROL_PID_HPP__

namespace wwControl
{

class PidController
{
  public:
    enum class Mode
    {
        Serial,
        Parallel,
    };

    struct Config
    {
        Mode mode;
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
        bool intergrator_limit_enable;
        float intergrator_limit_max;
        float intergrator_limit_min;

        /* Sample time (in seconds) */
        float sample_time;
    };

  public:
    PidController(Config &config) : config(config){};
    PidController(Config &&config) : config(config){};
    void reset();
    float update(float setpoint, float measurement);
    Config config;

  protected:
  private:
    float update_serial(float setpoint, float measurement);
    float update_parallel(float setpoint, float measurement);

    /* Controller "memory" */
    float _integrator;
    float _prevError; /* Required for _integrator */
    float _differentiator;
    float _prevMeasurement; /* Required for _differentiator */
};

} // namespace wwControl

#endif // __WWCONTROL_PID_HPP__