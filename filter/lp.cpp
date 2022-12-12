
#include <cmath>
#include "lp.hpp"

namespace wibot::control
{

	static float wrap(float x, float w)
	{
		return x - 2 * w * floor((x + w) / 2 / w);
	}
	//template<typename T>
	void FirstOrderLowPassFilter::apply_config()
	{
		_alpha = config.sample_time / (config.sample_time + 1.0f / config.cutoff_freq / 2.0f / _PI);
		_1_alpha = 1 - _alpha;
	};

	//template<typename T>
	float FirstOrderLowPassFilter::filter(float input)
	{
		if (!config.enable_wrap)
		{
			_outputLast = _alpha * input + _1_alpha * _outputLast;
		}
		else
		{
			_outputLast = wrap(_outputLast + _alpha * wrap(input - _outputLast, config.wrap_value), config.wrap_value);
		}

		return _outputLast;
	};

} // namespace wibot::control
