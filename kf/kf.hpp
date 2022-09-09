#ifndef __WWCONTROL_KF_HPP__
#define __WWCONTROL_KF_HPP__

#ifdef __cplusplus
extern "C"
{
#endif

#include "math.h"
#include "arm_math.h"
#include "arm_vec_math.h"

    struct KalmanFilter;
    typedef enum KF_TYPE
    {
        KF_TYPE_SIMPLE,
        KF_TYPE_EXTENDED,
        KF_TYPE_UNSCENTED,
    } KF_TYPE;

    typedef void (*KfFFunctonType)(const arm_matrix_instance_f32 *x, const arm_matrix_instance_f32 *u,
                                   arm_matrix_instance_f32 *out, arm_matrix_instance_f32 *F, void *userData);
    typedef void (*KfHFunctonType)(const arm_matrix_instance_f32 *x,
                                   arm_matrix_instance_f32 *out, arm_matrix_instance_f32 *H, void *userData);

    typedef struct KalmanFilter
    {
        KF_TYPE type;
        arm_matrix_instance_f32 *x;
        arm_matrix_instance_f32 *P;

        /**
         * @brief 
         * For simple kf, out = F*x+B*u, and update F if necessary.
         * For extended kf, estimate x, out = f(x, u, 0), and calculate jacobian matrix F_k <= dF/dx|x_hat_k-1,u_k if necessary.
         */
        KfFFunctonType F_func;

        /**
         * @brief 
         * For simple kf, update the H if necessary.
         * For extended kf, estimate z_hat, out <= H(x), and calculate jacobian matrix H_k <= dH/dx | x_hat_k if necessary.
         */
        KfHFunctonType H_func;

        uint16_t _dim_x;
        uint16_t _dim_z;

        arm_matrix_instance_f32 F;
        arm_matrix_instance_f32 H;

        void *userData;

    } KalmanFilter;

    /**
     * @brief 
     * 
     * @param kf 
     * @param f_func 
     * @param h_func 
     * @param dim_x 
     * @param dim_z 
     * @param F_room 
     * @param H_room 
     */
    void kf_create(KalmanFilter *kf,
                   KfFFunctonType f_func, KfHFunctonType h_func,
                   uint16_t dim_x, uint16_t dim_z,
                   float32_t *F_room, float32_t *H_room);

    void kf_init(KalmanFilter *kf, arm_matrix_instance_f32 *x, arm_matrix_instance_f32 *P);

    void kf_estimate(KalmanFilter *kf, arm_matrix_instance_f32 *u, arm_matrix_instance_f32 *Q);

    void kf_correct(KalmanFilter *kf, arm_matrix_instance_f32 *z, arm_matrix_instance_f32 *R);

#ifdef __cplusplus
}
#endif

#endif // __WWCONTROL_KF_HPP__