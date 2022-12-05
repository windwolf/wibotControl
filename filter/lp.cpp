
#include "lp.hpp"

namespace wibot::control
{
	//template<typename T>
	void FirstOrderLowPassFilter::config_apply(FirstOrderLowPassFilterConfig& config)
	{
		Configurable::config_apply(config);
		_alpha = config.sample_time / (config.sample_time + 1.0f / config.cutoff_freq / 2.0f / _PI);
		_1_alpha = 1 - _alpha;
	};

	//template<typename T>
	float FirstOrderLowPassFilter::filter(float input)
	{
		_outputLast = _alpha * input + _1_alpha * _outputLast;
		return _outputLast;
	};

} // namespace wibot::control
