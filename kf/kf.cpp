#include "kf.hpp"
#include "math_shared.hpp"

#define LOG_MODULE "kf"
#include "log.h"

namespace wibot::control
{

	void KalmanFilter::init(arm_matrix_instance_f32* x, arm_matrix_instance_f32* P)
	{
		this->x = x;
		this->P = P;
	};

	void KalmanFilter::predict(const arm_matrix_instance_f32& u, const arm_matrix_instance_f32& Q)
	{

		MATH_MAT_F32_DECLARE(xk, dimX, 1);
		/**
		 * For extended kf, calculate the jacobian matrix F = dF/dx|_state, u
		 * F's jacobian matrix must calculate before estimate current steo of _state,
		 * because it use the previous state of the _state.
		 * For simple kf, update the state transition matrix for current step if necessary.
		 */
		transit(u, Q, xk);
		// F_func(x, u, &xk, &F, userData);

		MATH_MAT_F32_COPY(x, &xk);

		// calc P = F P F' + Q

		// Ft = F'
		MATH_MAT_F32_DECLARE(Ft, dimX, dimX);
		arm_mat_trans_f32(&F, &Ft);

		// FP= F*P
		MATH_MAT_F32_DECLARE(FP, dimX, dimX);
		arm_mat_mult_f32(&F, P, &FP);

		// FPFt = F*P*Ft
		MATH_MAT_F32_DECLARE(FPFt, dimX, dimX);
		arm_mat_mult_f32(&FP, &Ft, &FPFt);

		// P = F*P*Ft+Q
		arm_mat_add_f32(&FPFt, &Q, P);
	};

	void KalmanFilter::transit(const arm_matrix_instance_f32& u, const arm_matrix_instance_f32& Q,
		arm_matrix_instance_f32& x_next)
	{
		arm_mat_mult_f32(&F, x, &x_next);
		arm_mat_add_f32(&x_next, &u, &x_next);
	};

	void KalmanFilter::update(const arm_matrix_instance_f32& z, const arm_matrix_instance_f32& R)
	{
		// estimate z_hat_k = h(x_k, 0);
		// calculate jacobian H = dH/dx|x_k
		MATH_MAT_F32_DECLARE(z_hat, dimZ, 1);

		measure(z_hat);

		// y = z - z_hat
		MATH_MAT_F32_DECLARE(y, dimZ, 1);
		arm_mat_sub_f32(&z, &z_hat, &y);

		// Ht = H'
		MATH_MAT_F32_DECLARE(Ht, dimX, dimZ);
		arm_mat_trans_f32(&H, &Ht);

		// PHt = P*H'
		MATH_MAT_F32_DECLARE(PHt, dimX, dimZ);
		arm_mat_mult_f32(P, &Ht, &PHt);

		// calculate S = H*P*H'+R

		// HPHt = H*P*H'
		MATH_MAT_F32_DECLARE(HPHt, dimZ, dimZ);
		arm_mat_mult_f32(&H, &PHt, &HPHt);

		MATH_MAT_F32_DECLARE(S, dimZ, dimZ);
		arm_mat_add_f32(&HPHt, &R, &S);

		// L = chol(S)
		MATH_MAT_F32_DECLARE(L, dimZ, dimZ);
		arm_mat_cholesky_f32(&S, &L);

		// PHtt = (P*H')'
		MATH_MAT_F32_DECLARE(PHtt, dimZ, dimX);
		arm_mat_trans_f32(&PHt, &PHtt);

		// U = L^-1*(P*H')'
		MATH_MAT_F32_DECLARE(U, dimZ, dimX);
		arm_mat_solve_lower_triangular_f32(&L, &PHtt, &U);

		// Ut = U'
		MATH_MAT_F32_DECLARE(Ut, dimX, dimZ);
		arm_mat_trans_f32(&U, &Ut);

		// Lry = L^-1*y
		MATH_MAT_F32_DECLARE(Lry, dimZ, 1);
		arm_mat_solve_lower_triangular_f32(&L, &y, &Lry);

		// UtLry = U'*L^-1*y
		MATH_MAT_F32_DECLARE(UtLry, dimX, 1);
		arm_mat_mult_f32(&Ut, &Lry, &UtLry);

		// _state = _state + U'*L^-1*y
		arm_mat_add_f32(x, &UtLry, x); // TODO:

		// LrH = L^-1*H
		MATH_MAT_F32_DECLARE(LrH, dimZ, dimX);
		arm_mat_solve_lower_triangular_f32(&L, &H, &LrH);

		// UtLrH = U'*L^-1*H
		MATH_MAT_F32_DECLARE(UtLrH, dimX, dimX);
		arm_mat_mult_f32(&Ut, &Lry, &UtLrH);

		// I = I-U'*L^-1*H
		MATH_MAT_F32_DECLARE(I, dimX, dimX);
		MATH_MAT_F32_SET(&I, 0);
		*MATH_MAT_ELEMENT(I, 0, 0) = 1;
		*MATH_MAT_ELEMENT(I, 1, 1) = 1;
		*MATH_MAT_ELEMENT(I, 2, 2) = 1;
		arm_mat_sub_f32(&I, &UtLrH, &I); // TODO:

		// Pn = (I-U'*L^-1*H)*P
		MATH_MAT_F32_DECLARE(Pn, dimX, dimX);
		arm_mat_mult_f32(&I, P, &Pn);

		MATH_MAT_F32_COPY(&Pn, P);
	};

	void KalmanFilter::measure(arm_matrix_instance_f32& z_estimate)
	{
		arm_mat_mult_f32(&H, x, &z_estimate);
	};

	void ExtendedKalmanFilter::transit(const arm_matrix_instance_f32& u,
		const arm_matrix_instance_f32& Q,
		arm_matrix_instance_f32& x_next)
	{
		F_function(*x, u, userData, x_next, F);
	};

	void ExtendedKalmanFilter::measure(arm_matrix_instance_f32& z_estimate)
	{
		H_function(*x, userData, z_estimate, H);
	};

}
