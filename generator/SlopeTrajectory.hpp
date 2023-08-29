//
// Created by zhouj on 2022/12/8.
//

#ifndef WWMOTOR_LIBS_WWCONTROL_GENERATOR_SLOPETRAJECTORY_HPP_
#define WWMOTOR_LIBS_WWCONTROL_GENERATOR_SLOPETRAJECTORY_HPP_

namespace wibot::control {

enum class SlopeTrajectoryStage {
    Init,
    Cruise,
    Final,
};

struct SlopeTrajectoryStep {
    float                Y;
    float                Yd;
    SlopeTrajectoryStage stage;
};

class SlopeTrajectory {
   public:
    bool                plan(float start_pos, float end_pos, float vel);
    SlopeTrajectoryStep evalByTime(float t);
    SlopeTrajectoryStep evelByPos(float currentPos);

   private:
    float start_pos_;
    float end_pos_;

    float vel_;

    float final_time_;
};

}  // namespace wibot::control

#endif  // WWMOTOR_LIBS_WWCONTROL_GENERATOR_SLOPETRAJECTORY_HPP_
