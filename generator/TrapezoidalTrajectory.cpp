//
// Created by zhouj on 2022/12/8.
//

#include <cmath>
#include "TrapezoidalTrajectory.hpp"

namespace wibot
{
	static float sign_hard(float val)
	{
		return (std::signbit(val)) ? -1.0f : 1.0f;
	}

	namespace control
	{
		bool TrapezoidalTrajectory::plan(float start_pos,
			float end_pos,
			float start_vel,
			float max_vel,
			float max_acc,
			float max_dec)
		{
			float dX = end_pos - start_pos;  // Distance to travel
			float stop_dist = (start_vel * start_vel) / (2.0f * max_dec); // Minimum stopping distance
			float dXstop = std::copysign(stop_dist, start_vel); // Minimum stopping displacement
			float s = sign_hard(dX - dXstop); // Sign of coast velocity (if any)
			acc_rated_ = s * max_acc;  // Maximum Acceleration (signed)
			dec_rated_ = -s * max_dec; // Maximum Deceleration (signed)
			vel_rated_ = s * max_vel;  // Maximum Velocity (signed)

			// If we start with a speed faster than cruising, then we need to decel instead of accel
			// aka "double deceleration move" in the paper
			if ((s * start_vel) > (s * vel_rated_))
			{
				acc_rated_ = -s * max_acc;
			}

			// Time to accel/decel to/from Vr (cruise speed)
			acc_time_ = (vel_rated_ - start_vel) / acc_rated_;
			dec_time_ = -vel_rated_ / dec_rated_;

			// Integral of velocity ramps over the full accel and decel times to get
			// minimum displacement required to reach cuising speed
			float dXmin = 0.5f * acc_time_ * (vel_rated_ + start_vel) + 0.5f * dec_time_ * vel_rated_;

			// Are we displacing enough to reach cruising speed?
			if (s * dX < s * dXmin)
			{
				// Short move (triangle profile)
				vel_rated_ = s * std::sqrt(std::max(
					(dec_rated_ * start_vel * start_vel + 2 * acc_rated_ * dec_rated_ * dX) / (dec_rated_ - acc_rated_),
					0.0f));
				acc_time_ = std::max(0.0f, (vel_rated_ - start_vel) / acc_rated_);
				dec_time_ = std::max(0.0f, -vel_rated_ / dec_rated_);
				cruise_time_ = 0.0f;
			}
			else
			{
				// Long move (trapezoidal profile)
				cruise_time_ = (dX - dXmin) / vel_rated_;
			}

			// Fill in the rest of the values used at evaluation-time
			final_time_ = acc_time_ + cruise_time_ + dec_time_;
			start_pos_ = start_pos;
			end_pos_ = end_pos;
			start_vel_ = start_vel;
			yAccel_ = start_pos + start_vel * acc_time_
				+ 0.5f * acc_rated_ * acc_time_ * acc_time_; // pos at end of accel phase

			return true;
		}
		void TrapezoidalTrajectory::config_apply(TrapezoidalTrajectoryConfig& config)
		{
			config_ = config;

		}
		TrapezoidalTrajectoryStep TrapezoidalTrajectory::eval(float t)
		{
			TrapezoidalTrajectoryStep trajStep;
			if (t < 0.0f)
			{  // Initial Condition
				trajStep.Y = start_pos_;
				trajStep.Yd = start_vel_;
				trajStep.Ydd = 0.0f;
			}
			else if (t < acc_time_)
			{  // Accelerating
				trajStep.Y = start_pos_ + start_vel_ * t + 0.5f * acc_rated_ * t * t;
				trajStep.Yd = start_vel_ + acc_rated_ * t;
				trajStep.Ydd = acc_rated_;
			}
			else if (t < acc_time_ + cruise_time_)
			{  // Coasting
				trajStep.Y = yAccel_ + vel_rated_ * (t - acc_time_);
				trajStep.Yd = vel_rated_;
				trajStep.Ydd = 0.0f;
			}
			else if (t < final_time_)
			{  // Deceleration
				float td = t - final_time_;
				trajStep.Y = end_pos_ + 0.5f * dec_rated_ * td * td;
				trajStep.Yd = dec_rated_ * td;
				trajStep.Ydd = dec_rated_;
			}
			else if (t >= final_time_)
			{  // Final Condition
				trajStep.Y = end_pos_;
				trajStep.Yd = 0.0f;
				trajStep.Ydd = 0.0f;
			}
			else
			{
				// TODO: report error here
			}
		}
	} // wibot
} // control
