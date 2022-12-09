//
// Created by zhouj on 2022/12/8.
//

#ifndef WWMOTOR_LIBS_WWCONTROL_GENERATOR_TRAPEZOIDALTRAJECTORY_HPP_
#define WWMOTOR_LIBS_WWCONTROL_GENERATOR_TRAPEZOIDALTRAJECTORY_HPP_

#include "base.hpp"
namespace wibot
{
	namespace control
	{
		struct TrapezoidalTrajectoryConfig
		{
			float vel_limit = 2.0f;   // [turn/s]
			float accel_limit = 0.5f; // [turn/s^2]
			float decel_limit = 0.5f; // [turn/s^2]
		};

		struct TrapezoidalTrajectoryStep
		{
			float Y;
			float Yd;
			float Ydd;
		};

		class TrapezoidalTrajectory : Configurable<TrapezoidalTrajectoryConfig>
		{
		 public:
			void config_apply(TrapezoidalTrajectoryConfig& config);
			bool plan(float start_pos, float end_pos, float start_vel, float max_vel, float max_acc, float max_dec);
			TrapezoidalTrajectoryStep eval(float t);
		 private:
			TrapezoidalTrajectoryConfig config_;

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

			float t_;
		};

	} // wibot
} // control

#endif //WWMOTOR_LIBS_WWCONTROL_GENERATOR_TRAPEZOIDALTRAJECTORY_HPP_
