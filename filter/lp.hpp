#ifndef __WWCONTROL_LP_HPP__
#define __WWCONTROL_LP_HPP__

#include "base.hpp"
namespace wibot::control
{
	struct FirstOrderLowPassFilterConfig
	{
		float sample_time;
		float cutoff_freq;
	};
	//template<typename T>
	class FirstOrderLowPassFilter : public Configurable<FirstOrderLowPassFilterConfig>
	{
	 public:

		void config_apply(FirstOrderLowPassFilterConfig& config);

		float filter(float input);

	 private:
		float _alpha;
		float _1_alpha;

		float _outputLast;
	};

	// using FirstOrderLowPassFilterf = FirstOrderLowPassFilter<float>;
	// using FirstOrderLowPassFilters = FirstOrderLowPassFilter<uint16_t>;
	// using FirstOrderLowPassFilterl = FirstOrderLowPassFilter<uint32_t>;
} // namespace wibot::control

#endif // __WWCONTROL_LP_HPP__
