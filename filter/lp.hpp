#ifndef __WWCONTROL_LP_HPP__
#define __WWCONTROL_LP_HPP__

namespace wwControl
{
class FirstOrderLowPassFilter
{
  public:
    struct Config
    {
        float sample_time;
        float cutoff_freq;
    };

  public:
    FirstOrderLowPassFilter(Config &&config) : _config(config)
    {
        init();
    };

    void init();

    float filter(float input);

  private:
    Config _config;
    float _alpha;
    float _1_alpha;

    float _outputLast;
};

} // namespace wwControl

#endif // __WWCONTROL_LP_HPP__