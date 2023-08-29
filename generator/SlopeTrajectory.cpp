//
// Created by zhouj on 2022/12/8.
//

#include "SlopeTrajectory.hpp"
#include <cmath>

namespace wibot::control {

bool SlopeTrajectory::plan(float start_pos, float end_pos, float vel) {
    float dX = end_pos - start_pos;     // Distance to travel
    vel_     = std::copysign(vel, dX);  // Maximum Velocity (signed)

    final_time_ = dX / vel_;
    start_pos_  = start_pos;
    end_pos_    = end_pos;
    return true;
}

SlopeTrajectoryStep SlopeTrajectory::evalByTime(float t) {
    SlopeTrajectoryStep trajStep;
    if (t < 0.0f) {  // Initial Condition
        trajStep.Y     = start_pos_;
        trajStep.Yd    = 0.0f;
        trajStep.stage = SlopeTrajectoryStage::Init;
    } else if (t < final_time_) {  // slope
        float td       = t - final_time_;
        trajStep.Y     = end_pos_ - vel_ * td;
        trajStep.Yd    = vel_;
        trajStep.stage = SlopeTrajectoryStage::Cruise;
    } else if (t >= final_time_) {  // Final Condition
        trajStep.Y     = end_pos_;
        trajStep.Yd    = 0.0f;
        trajStep.stage = SlopeTrajectoryStage::Final;
    } else {
        // TODO: report error here
    }

    return trajStep;
}
SlopeTrajectoryStep SlopeTrajectory::evelByPos(float currentPos) {
    SlopeTrajectoryStep trajStep;
    if (start_pos_ >= end_pos_) {
        if (currentPos >= start_pos_) {
            trajStep.Y     = start_pos_;
            trajStep.Yd    = 0.0f;
            trajStep.stage = SlopeTrajectoryStage::Init;
        } else if (currentPos >= end_pos_) {
            trajStep.Y     = currentPos;
            trajStep.Yd    = vel_;
            trajStep.stage = SlopeTrajectoryStage::Cruise;
        } else if (currentPos < end_pos_) {
            trajStep.Y     = end_pos_;
            trajStep.Yd    = 0.0f;
            trajStep.stage = SlopeTrajectoryStage::Final;
        } else {
            // TODO: report error here
        }
    } else {
        if (currentPos <= start_pos_) {
            trajStep.Y     = start_pos_;
            trajStep.Yd    = 0.0f;
            trajStep.stage = SlopeTrajectoryStage::Init;
        } else if (currentPos <= end_pos_) {
            trajStep.Y     = currentPos;
            trajStep.Yd    = vel_;
            trajStep.stage = SlopeTrajectoryStage::Cruise;
        } else if (currentPos > end_pos_) {
            trajStep.Y     = end_pos_;
            trajStep.Yd    = 0.0f;
            trajStep.stage = SlopeTrajectoryStage::Final;
        } else {
            // TODO: report error here
        }
    }
    return trajStep;
}

}  // namespace wibot::control
