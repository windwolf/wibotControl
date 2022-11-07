#include "lp.hpp"

namespace wwControl
{

void FirstOrderLowPassFilter::init()
{
    _alpha = _config.sample_time / (_config.sample_time + 1.0f / _config.cutoff_freq / 2.0f / M_PI);
    _1_alpha = 1 - _a;
};

float FirstOrderLowPassFilter::filter(float input)
{
    _outputLast = _alpha * input + _1_alpha * _outputLast;
    return _outputLast;
};

} // namespace wwControl
