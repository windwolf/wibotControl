#ifndef __WWCONTROL_BASIC_MATH_SHARED_HPP__
#define __WWCONTROL_BASIC_MATH_SHARED_HPP__

#include "base/base.hpp"
#include "math.h"
#include "arm_math.h"



#define MATH_MAT_ROW(mat, row) ((mat).pData + row * (mat).numCols)
#define MATH_MAT_COLUMN(mat, col) ((mat).pData + col)
#define MATH_MAT_ELEMENT(mat, row, col) ((mat).pData + row * (mat).numCols + col)

#define MATH_MAT_F32_DECLARE(mat, row, col)                                                        \
    arm_matrix_instance_f32 mat;                                                                   \
    float32_t mat##_data[(row) * (col)];                                                           \
    arm_mat_init_f32(&mat, (row), (col), mat##_data)

#define MATH_MAT_F32_SET(mat, val)                                                                 \
    memset((mat)->pData, val, (mat)->numRows *(mat)->numCols * sizeof(float32_t))

#define MATH_MAT_F32_COPY(dst, src)                                                                \
    memcpy((dst)->pData, (src)->pData, (dst)->numRows *(dst)->numCols * sizeof(float32_t))



template <uint16_t rowNum, uint16_t colNum> struct Matrix_f32
{
    Matrix_f32()
    {
        arm_mat_init_f32(&mat, (rowNum), (colNum), data);
    };
    Matrix_f32(const Matrix_f32 &obj)
    {
        arm_mat_init_f32(&mat, (rowNum), (colNum), data);
        memcpy(data, obj.data, sizeof(data));
    }
    void value_set(float32_t val)
    {
        memset(data, val, sizeof(data));
    }

    float32_t data[rowNum * colNum];
    arm_matrix_instance_f32 mat;
};

struct Math
{
    static float atan2(float y, float x)
    {
        float result;
        arm_atan2_f32(y, x, &result);
        return result;
    }
    static void sincos(float theta, float *sin, float *cos)
    {
        arm_sin_cos_f32(theta, sin, cos);
    }

    static float sin(float theta)
    {
        return arm_sin_f32(theta);
    }

    static float cos(float theta)
    {
        return arm_cos_f32(theta);
    }

    static float sqrt(float x)
    {
        float result;
        arm_sqrt_f32(x, &result);
        return result;
    }

    static float mod(float x, float y)
    {
        float result = std::fmod(x, y);
        return result >= 0 ? result : (result + y);
    }

    static float sign(float x)
    {
        return x >= 0 ? 1 : -1;
    }

    static Vector2f sign(Vector2f x)
    {
        Vector2f result;
        if (x.v1 > 0)
            result.v1 = 1.0f;
        else if (x.v1 < 0)

            result.v1 = -1.0f;
        else
            result.v1 = 0.0f;

        if (x.v2 > 0)
            result.v2 = 1.0f;
        else if (x.v2 < 0)
            result.v2 = -1.0f;
        else
            result.v2 = 0.0f;
        return result;
    }

    static float circle_normalize(float theta)
    {

        float result = std::fmod(theta, _2PI);
        return result >= 0 ? result : (result + _2PI);
    }

    static float floor(float x)
    {
        return std::floor(x);
    }
};

#endif // __WWCONTROL_BASIC_MATH_SHARED_HPP__