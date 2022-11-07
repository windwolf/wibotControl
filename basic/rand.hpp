#ifndef __WWCONTROL_BASIC_RAND_HPP__
#define __WWCONTROL_BASIC_RAND_HPP__ 

#include "math.h"
#include "arm_math.h"

#ifdef __cplusplus
extern "C"
{
#endif

    float32_t rand1f();
    float32_t randrf(float32_t a, float32_t b);
    float32_t randnf(float32_t mu, float32_t sigma);

    float64_t rand1lf();
    float64_t randrlf(float64_t a, float64_t b);
    float64_t randnlf(float64_t mu, float64_t sigma);

#ifdef __cplusplus
}
#endif

#endif	// __WWCONTROL_BASIC_RAND_HPP__