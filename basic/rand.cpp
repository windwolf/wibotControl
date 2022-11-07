#include "rand.hpp"


/**
 * @brief generate a random number in range of 0 to 1, with precision 0.000001
 * precious
 * @return float32_t 
 */
float32_t rand1f()
{
    return (float32_t)rand() / (float32_t)RAND_MAX;
}
float32_t randrf(float32_t a, float32_t b)
{
    return a + (float32_t)rand() / (float32_t)RAND_MAX * (b - a);
}

float32_t randnf(float32_t mu, float32_t sigma)
{
    const float32_t epsilon = 0.00000001f;
    //const float32_t two_pi = 2.0 * PI;

    static float32_t z0, z1;
    static bool generate;
    generate = !generate;

    if (!generate)
    {
        return z1 * sigma + mu;
    }

    float32_t u1, u2;
    do
    {
        u1 = rand1f();
        u2 = rand1f();
    } while (u1 <= epsilon);

    float32_t mag, sin, cos;
    arm_sqrt_f32(-2.0 * log(u1), &mag);
    arm_sin_cos_f32(u2 * 360.0f, &sin, &cos);
    z0 = mag * cos;
    z1 = mag * sin;

    return z0 * sigma + mu;
};

// float64_t rand1lf()
// {
//     return (float32_t)rand() / (float32_t)RAND_MAX;
// };
// float64_t randrlf(float64_t a, float64_t b)
// {
//     return a + (float64_t)rand() / (float64_t)RAND_MAX * (b - a);
// };

// float64_t randnlf(float64_t mu, float64_t sigma)
// {
//     const float64_t epsilon = 0.00000001f;
//     const float64_t two_pi = 2.0 * PI;

//     static float64_t z0, z1;
//     static bool generate;
//     generate = !generate;

//     if (!generate)
//     {
//         return z1 * sigma + mu;
//     }

//     float64_t u1, u2;
//     do
//     {
//         u1 = rand1f();
//         u2 = rand1f();
//     } while (u1 <= epsilon);

//     float64_t mag, sin, cos;
//     arm_sqrt_f64(-2.0 * log(u1), &mag);
//     arm_sin_cos_f64(two_pi * u2, &sin, &cos);
//     z0 = mag * cos;
//     z1 = mag * sin;

//     return z0 * sigma + mu;
// };