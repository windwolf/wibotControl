#ifndef __WWCONTROL_BASIC_MATH_SHARED_HPP__
#define __WWCONTROL_BASIC_MATH_SHARED_HPP__

#include "math.h"
#include "arm_math.h"

#ifdef __cplusplus
extern "C"
{
#endif

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

#ifdef __cplusplus
}
#endif

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

#endif // __WWCONTROL_BASIC_MATH_SHARED_HPP__