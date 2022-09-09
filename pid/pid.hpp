#ifndef __WWCONTROL_PID_HPP__
#define __WWCONTROL_PID_HPP__

#ifdef __cplusplus
extern "C"
{
#endif

    typedef struct
    {

        /* Controller gains */
        float Kp;
        float Ki;
        float Kd;

        /* Derivative low-pass filter time constant */
        float tau;

        /* Output limits */
        float limMin;
        float limMax;

        /* Integrator limits */
        float limMinInt;
        float limMaxInt;

        /* Sample time (in seconds) */
        float T;

        /* Controller "memory" */
        float _integrator;
        float _prevError; /* Required for _integrator */
        float _differentiator;
        float _prevMeasurement; /* Required for _differentiator */

        /* Controller output */
        float out;

    } PIDController;

    void pid_controller_init(PIDController *pid);
    float pid_controller_update(PIDController *pid, float setpoint, float measurement);

#ifdef __cplusplus
}
#endif

#endif // __WWCONTROL_PID_HPP__