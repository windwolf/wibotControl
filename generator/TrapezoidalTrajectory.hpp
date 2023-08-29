//
// Created by zhouj on 2022/12/8.
//

#ifndef WWMOTOR_LIBS_WWCONTROL_GENERATOR_TRAPEZOIDALTRAJECTORY_HPP_
#define WWMOTOR_LIBS_WWCONTROL_GENERATOR_TRAPEZOIDALTRAJECTORY_HPP_

#include "base.hpp"
namespace wibot::control {
struct TrapezoidalTrajectoryConfig {
    float vel_limit   = 2.0f;  // [turn/s]
    float accel_limit = 0.5f;  // [turn/s^2]
    float decel_limit = 0.5f;  // [turn/s^2]
};

enum class TrapezoidalTrajectoryStage {
    Init,
    Accel,
    Cruise,
    Decel,
    Final,
};

struct TrapezoidalTrajectoryStep {
    float                      Y;
    float                      Yd;
    float                      Ydd;
    TrapezoidalTrajectoryStage stage;
};

class TrapezoidalTrajectory : Configurable<TrapezoidalTrajectoryConfig> {
   public:
    bool plan(float start_pos, float end_pos, float start_vel, float max_vel, float max_acc,
              float max_dec);
    /**
     * Eval step info by current time tick.
     * @note If use this function for pos control, succed
     * If use this function for vel (not pos) control, it become a open loop control,
     * so it will lose sync when real pos is not equal to the pos calculated by this function.
     *
     * @param t
     * @return
     */
    TrapezoidalTrajectoryStep evalByTime(float t);

    /**
     * Eval step info by current position.
     *
     * @param currentPos
     * @return
     */
    TrapezoidalTrajectoryStep evalByPos(float currentPos);

   private:
    float start_pos_;
    float end_pos_;
    float start_vel_;

    float acc_rated_;
    float vel_rated_;
    float dec_rated_;

    float acc_time_;
    float cruise_time_;
    float dec_time_;
    float final_time_;

    float yAccel_;
    float yCruise_;

    float t_;
};

}  // namespace wibot::control

#endif  // WWMOTOR_LIBS_WWCONTROL_GENERATOR_TRAPEZOIDALTRAJECTORY_HPP_
