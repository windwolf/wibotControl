#include "kf_test.hpp"
#include "kf/kf.hpp"
#include "minunit.h"
#include "stdlib.h"
#include "basic/math_shared.hpp"
#include "rand.hpp"

using namespace wibot::control;

static void skf_1d()
{
	printf("<------skf_1d------>\r\n");
	const int dim_x = 1;
	const int dim_z = 1;
	MATH_MAT_F32_DECLARE(F, dim_x, dim_x);
	*MATH_MAT_ELEMENT(F, 0, 0) = 0.75f;
	MATH_MAT_F32_DECLARE(H, dim_z, dim_x);
	*MATH_MAT_ELEMENT(H, 0, 0) = 1.0f;
	KalmanFilter kf(dim_x, dim_z, F, H);
	const int steps = 10;

	float32_t data[steps][2] = {
		{ 1000.0f, 1077.0f }, { 750.0f, 758.0f }, { 563.0f, 650.0f }, { 422.0f, 323.0f }, { 316.0f, 405.0f },
		{ 237.0f, 170.0f }, { 178.0f, 4.0f }, { 133.0f, 269.0f }, { 100.0f, 51.0f }, { 75.0f, 97.0f },
	};
	kf.userData = data;

	MATH_MAT_F32_DECLARE(x, 1, 1);
	*MATH_MAT_ELEMENT(x, 0, 0) = data[0][1];

	MATH_MAT_F32_DECLARE(P, 1, 1);
	*MATH_MAT_ELEMENT(P, 0, 0) = 1.0f;

	kf.init(&x, &P);

	MATH_MAT_F32_DECLARE(Q, 1, 1);
	*MATH_MAT_ELEMENT(Q, 0, 0) = 0.00;
	MATH_MAT_F32_DECLARE(R, 1, 1);
	*MATH_MAT_ELEMENT(R, 0, 0) = 200;

	MATH_MAT_F32_DECLARE(z, 1, 1);
	// printf("%f\t-.-\t%f\t%f\r\n", 10.0f, *MATH_MAT_ELEMENT(P, 0, 0), *MATH_MAT_ELEMENT(x, 0, 0));
	MATH_MAT_F32_DECLARE(ux, 1, 1);
	for (size_t i = 1; i < steps; i++)
	{
		printf("%f\t", data[i][0]);
		kf.predict(ux, Q);

		printf("%f\t", data[i][1]);
		*MATH_MAT_ELEMENT(z, 0, 0) = data[i][1];
		kf.update(z, R);

		printf("%f\t%f\r\n", *MATH_MAT_ELEMENT(P, 0, 0), *MATH_MAT_ELEMENT(x, 0, 0));
	}
}

static void skf_4senser_fused()
{
	printf("<------skf_4senser_fused------>\r\n");
	srand(3);
	const int dim_x = 1;
	const int dim_z = 4;
	MATH_MAT_F32_DECLARE(F, dim_x, dim_x);
	*MATH_MAT_ELEMENT(F, 0, 0) = 1.0f;
	MATH_MAT_F32_DECLARE(H, dim_z, dim_x);
	*MATH_MAT_ELEMENT(H, 0, 0) = 1.0f;
	*MATH_MAT_ELEMENT(H, 1, 0) = 1.0f;
	KalmanFilter kf(dim_x, dim_z, F, H);

	const int steps = 30;
	float32_t data[steps][dim_x + dim_z];
	for (size_t i = 0; i < steps; i++)
	{
		data[i][0] = 0.0f;
		data[i][1] = randnf(0, 0.1f) + ((i == 3) ? 2 : 0);
		data[i][2] = randnf(0, 1.0f) + ((i == 10) ? 2 : 0);
		data[i][3] = randnf(0, 0.5f) + ((i == 17) ? 2 : 0);
		data[i][4] = randnf(0, 0.2f) + ((i == 24) ? 2 : 0);
	}

	kf.userData = data;

	MATH_MAT_F32_DECLARE(x, dim_x, 1);
	*MATH_MAT_ELEMENT(x, 0, 0) = data[0][0];

	MATH_MAT_F32_DECLARE(P, dim_x, dim_x);
	*MATH_MAT_ELEMENT(P, 0, 0) = 0.01f;

	kf.init(&x, &P);

	MATH_MAT_F32_DECLARE(Q, dim_x, dim_x);
	*MATH_MAT_ELEMENT(Q, 0, 0) = 0.0f;
	MATH_MAT_F32_DECLARE(R, dim_z, dim_z);
	*MATH_MAT_ELEMENT(R, 0, 0) = 0.01f;
	*MATH_MAT_ELEMENT(R, 0, 1) = 0.0f;
	*MATH_MAT_ELEMENT(R, 0, 2) = 0.0f;
	*MATH_MAT_ELEMENT(R, 0, 3) = 0.0f;
	*MATH_MAT_ELEMENT(R, 1, 0) = 0.0f;
	*MATH_MAT_ELEMENT(R, 1, 1) = 1.0f;
	*MATH_MAT_ELEMENT(R, 1, 2) = 0.0f;
	*MATH_MAT_ELEMENT(R, 1, 3) = 0.0f;
	*MATH_MAT_ELEMENT(R, 2, 0) = 0.0f;
	*MATH_MAT_ELEMENT(R, 2, 1) = 0.0f;
	*MATH_MAT_ELEMENT(R, 2, 2) = 0.25f;
	*MATH_MAT_ELEMENT(R, 2, 3) = 0.0f;
	*MATH_MAT_ELEMENT(R, 3, 0) = 0.0f;
	*MATH_MAT_ELEMENT(R, 3, 1) = 0.0f;
	*MATH_MAT_ELEMENT(R, 3, 2) = 0.0f;
	*MATH_MAT_ELEMENT(R, 3, 3) = 0.04f;

	MATH_MAT_F32_DECLARE(z, dim_z, 1);

	MATH_MAT_F32_DECLARE(u, dim_x, 1);

	for (size_t i = 1; i < steps; i++)
	{
		printf("%f\t", data[i][0]);
		kf.predict(u, Q);

		printf("%f\t", data[i][1]);
		printf("%f\t", data[i][2]);
		printf("%f\t", data[i][3]);
		printf("%f\t", data[i][4]);
		*MATH_MAT_ELEMENT(z, 0, 0) = data[i][1];
		*MATH_MAT_ELEMENT(z, 1, 0) = data[i][2];
		*MATH_MAT_ELEMENT(z, 2, 0) = data[i][3];
		*MATH_MAT_ELEMENT(z, 3, 0) = data[i][4];
		kf.update(z, R);

		printf("%f\r\n", *MATH_MAT_ELEMENT(x, 0, 0));
	}
}

// static void skf_dynamic_system()
// {
//     printf("<------skf_dynamic_system------>\r\n");
//     // x: [x, y, spd_x, spd_y], z: [gps_x, gps_y, acc_x, acc_y, avel_z]
//     // u: [steeling, accel]
//     const int dim_x = 4;
//     const int dim_z = 5;
//     float32_t accTimeStep = 0.1f;
//     float32_t gpsTimeStep = 1.0f;
//     const int steps = 30;
//     float32_t data[steps][dim_x + dim_z];
//     data[0][0] = 0.0f; // x[x]
//     data[0][1] = 0.0f; // x[y]
//     data[0][2] = 0.0f; // x[spd_x]
//     data[0][3] = 0.0f; // x[spd_y]
//     data[0][4] = 0.0f; // u[steeling]
//     data[0][5] = 0.1f; // z[gps_x]
//     data[0][6] = 0.1f; // z[gps_y]
//     data[0][7] = 0.1f; // z[acc_x]
//     data[0][8] = 0.1f; // z[acc_y]
//     data[0][9] = 0.1f; // z[aval_z]
//     for (size_t i = 1; i < steps; i++)
//     {
//         data[i][0] = 0.0f; // x[x]
//         data[i][1] = 0.0f; // x[y]
//         data[i][2] = 0.0f; // x[spd_x]
//         data[i][3] = 0.0f; // x[spd_y]
//         data[i][4] = 0.0f; // u[steeling]
//         data[i][5] = 0.1f; // z[gps_x]
//         data[i][6] = 0.1f; // z[gps_y]
//         data[i][7] = 0.1f; // z[acc_x]
//         data[i][8] = 0.1f; // z[acc_y]
//         data[i][9] = 0.1f; // z[aval_z]
//     }

//     MATH_MAT_F32_DECLARE(F, dim_x, dim_x);

//     MATH_MAT_F32_DECLARE(H, dim_z, dim_x);

//     KalmanFilter kf(dim_x, dim_z, F, H);
//     kf.userData = data;

//     MATH_MAT_F32_DECLARE(x, dim_x, 1);
//     *MATH_MAT_ELEMENT(x, 0, 0) = data[0][0];

//     MATH_MAT_F32_DECLARE(P, dim_x, dim_x);
//     *MATH_MAT_ELEMENT(P, 0, 0) = 0.01f;

//     kf.init(&x, &P);

//     MATH_MAT_F32_DECLARE(Q, dim_x, dim_x);
//     *MATH_MAT_ELEMENT(Q, 0, 0) = 0.0f;
//     MATH_MAT_F32_DECLARE(R, dim_z, dim_z);
//     *MATH_MAT_ELEMENT(R, 0, 0) = 0.0033f;
//     *MATH_MAT_ELEMENT(R, 0, 1) = 0.0f;
//     *MATH_MAT_ELEMENT(R, 0, 2) = 0.0f;
//     *MATH_MAT_ELEMENT(R, 0, 3) = 0.0f;
//     *MATH_MAT_ELEMENT(R, 1, 0) = 0.0f;
//     *MATH_MAT_ELEMENT(R, 1, 1) = 0.3336f;
//     *MATH_MAT_ELEMENT(R, 1, 2) = 0.0f;
//     *MATH_MAT_ELEMENT(R, 1, 3) = 0.0f;
//     *MATH_MAT_ELEMENT(R, 2, 0) = 0.0f;
//     *MATH_MAT_ELEMENT(R, 2, 1) = 0.0f;
//     *MATH_MAT_ELEMENT(R, 2, 2) = 0.0831f;
//     *MATH_MAT_ELEMENT(R, 2, 3) = 0.0f;
//     *MATH_MAT_ELEMENT(R, 3, 0) = 0.0f;
//     *MATH_MAT_ELEMENT(R, 3, 1) = 0.0f;
//     *MATH_MAT_ELEMENT(R, 3, 2) = 0.0f;
//     *MATH_MAT_ELEMENT(R, 3, 3) = 0.0134f;

//     MATH_MAT_F32_DECLARE(z, dim_z, 1);
//     MATH_MAT_F32_DECLARE(u, dim_x, 1);
//     for (size_t i = 1; i < steps; i++)
//     {
//         printf("%f\t", data[i][0]);
//         kf.predict(u, Q);

//         printf("%f\t", data[i][1]);
//         printf("%f\t", data[i][2]);
//         printf("%f\t", data[i][3]);
//         printf("%f\t", data[i][4]);
//         *MATH_MAT_ELEMENT(z, 0, 0) = data[i][1];
//         *MATH_MAT_ELEMENT(z, 1, 0) = data[i][2];
//         *MATH_MAT_ELEMENT(z, 2, 0) = data[i][3];
//         *MATH_MAT_ELEMENT(z, 3, 0) = data[i][4];
//         kf.update(z, R);

//         printf("%f\r\n", *MATH_MAT_ELEMENT(x, 0, 0));
//     }
// }

void kf_test()
{
	skf_1d();
	skf_4senser_fused();
}
