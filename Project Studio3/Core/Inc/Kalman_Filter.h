/*
 * Kalman_Filter.h
 *
 *  Created on: May 17, 2023
 *      Author: tucha
 */

#ifndef INC_KALMAN_FILTER_H_
#define INC_KALMAN_FILTER_H_
#include "arm_math.h"

//init variable

// Model Matrix
float32_t A_f32[9] = {
		1, 0.01, 0.00005,
		0, 1, 0.01,
		0, 0, 1
};
arm_matrix_instance_f32 A;

float32_t B_f32[3] = {
		0,
		0,
		0
};
arm_matrix_instance_f32 B;

float32_t C_f32[3] = {
		1,
		0,
		0
};
arm_matrix_instance_f32 C;

float32_t D_f32[1] = {
		0
};
arm_matrix_instance_f32 D;

float32_t G_f32[3] = {
		0.00005,
		0.01,
		1
};
arm_matrix_instance_f32 G;

// Covariance
float32_t Q_f32[1] = {
		10000
};
arm_matrix_instance_f32 Q;

float32_t R_f32[1] = {
		0.001
};
arm_matrix_instance_f32 R;

// Measurement matrix
float32_t M_f32[1] = {
		0
};
arm_matrix_instance_f32 M;

float32_t I3_f32[9] = {
		1, 0, 0,
		0, 1, 0,
		0, 0, 1
};
arm_matrix_instance_f32 I3;

//Initia Condition
float32_t X_f32[3] = {
		0,
		0,
		0
};
arm_matrix_instance_f32 X;

float32_t U_f32[1] = {
		0
};
arm_matrix_instance_f32 U;

float32_t P_f32[9] = {
		0, 0, 0,
		0, 0, 0,
		0, 0, 0
};
arm_matrix_instance_f32 P;

//Transpose Matrix
float32_t At_f32[9];
arm_matrix_instance_f32 At;

float32_t Ct_f32[3];
arm_matrix_instance_f32 Ct;

float32_t Gt_f32[3];
arm_matrix_instance_f32 Gt;

//Prediction Matrix
float32_t X_predict_f32[3];
arm_matrix_instance_f32 X_predict;

float32_t P_predict_f32[9];
arm_matrix_instance_f32 P_predict;

//Prediction_Stage Matrix
float32_t AmX_f32[3];
arm_matrix_instance_f32 AmX;

float32_t BmU_f32[3];
arm_matrix_instance_f32 BmU;

float32_t AmP_f32[9];
arm_matrix_instance_f32 AmP;

float32_t AmPmAt_f32[9];
arm_matrix_instance_f32 AmPmAt;

float32_t GmQ_f32[3];
arm_matrix_instance_f32 GmQ;

float32_t GmQmGt_f32[9];
arm_matrix_instance_f32 GmQmGt;

//Update_Stage Matrix
float32_t K_f32[3];
arm_matrix_instance_f32 K;

float32_t P_predictmCt_f32[3];
arm_matrix_instance_f32 P_predictmCt;

float32_t CmP_predictmCt_f32[1];
arm_matrix_instance_f32 CmP_predictmCt;

float32_t CmP_predictmCtaR_f32[1];
arm_matrix_instance_f32 CmP_predictmCtaR;

float32_t inv_CmP_predictmCtaR_f32[1];
arm_matrix_instance_f32 inv_CmP_predictmCtaR;

float32_t CmX_predict_f32[1];
arm_matrix_instance_f32 CmX_predict;

float32_t MsCmX_predict_f32[1];
arm_matrix_instance_f32 MsCmX_predict;

float32_t KmMsCmX_predict_f32[3];
arm_matrix_instance_f32 KmMsCmX_predict;

float32_t KmC_f32[9];
arm_matrix_instance_f32 KmC;

float32_t I3sKmC_f32[9];
arm_matrix_instance_f32 I3sKmC;

void Initial_KalmanFilter()
{
	//init all matrix

	arm_mat_init_f32(&A, 3, 3,(float32_t*) &A_f32);
	arm_mat_init_f32(&B, 3, 1,(float32_t*) &B_f32);
	arm_mat_init_f32(&C, 1, 3,(float32_t*) &C_f32);
	arm_mat_init_f32(&D, 1, 1,(float32_t*) &D_f32);
	arm_mat_init_f32(&G, 3, 1,(float32_t*) &G_f32);

	arm_mat_init_f32(&I3, 3, 3,(float32_t*) &I3_f32);
	arm_mat_init_f32(&X, 3, 1,(float32_t*) &X_f32);
	arm_mat_init_f32(&U, 1, 1,(float32_t*) &U_f32);
	arm_mat_init_f32(&P, 3, 3,(float32_t*) &P_f32);

	arm_mat_init_f32(&At, 3, 3,(float32_t*) &At_f32);
	arm_mat_init_f32(&Ct, 3, 1,(float32_t*) &Ct_f32);
	arm_mat_init_f32(&Gt, 1, 3,(float32_t*) &Gt_f32);

	arm_mat_init_f32(&X_predict, 3, 1,(float32_t*) &X_predict_f32);
	arm_mat_init_f32(&P_predict, 3, 3,(float32_t*) &P_predict_f32);

	arm_mat_init_f32(&AmX, 3, 1,(float32_t*) &AmX_f32);
	arm_mat_init_f32(&BmU, 3, 1,(float32_t*) &BmU_f32);
	arm_mat_init_f32(&AmP, 3, 3,(float32_t*) &AmP_f32);
	arm_mat_init_f32(&AmPmAt, 3, 3,(float32_t*) &AmPmAt_f32);
	arm_mat_init_f32(&GmQ, 3, 1,(float32_t*) &GmQ_f32);
	arm_mat_init_f32(&GmQmGt, 3, 3,(float32_t*) &GmQmGt_f32);

	arm_mat_init_f32(&K, 3, 1,(float32_t*) &K_f32);
	arm_mat_init_f32(&P_predictmCt, 3, 1,(float32_t*) &P_predictmCt_f32);
	arm_mat_init_f32(&CmP_predictmCt, 1, 1,(float32_t*) &CmP_predictmCt_f32);
	arm_mat_init_f32(&CmP_predictmCtaR, 1, 1,(float32_t*) &CmP_predictmCtaR_f32);
	arm_mat_init_f32(&inv_CmP_predictmCtaR, 1, 1,(float32_t*) &inv_CmP_predictmCtaR_f32);
	arm_mat_init_f32(&CmX_predict, 1, 1,(float32_t*) &CmX_predict_f32);
	arm_mat_init_f32(&MsCmX_predict, 1, 1,(float32_t*) &MsCmX_predict_f32);
	arm_mat_init_f32(&KmMsCmX_predict, 3, 1,(float32_t*) &KmMsCmX_predict_f32);
	arm_mat_init_f32(&KmC, 3, 3,(float32_t*) &KmC_f32);
	arm_mat_init_f32(&I3sKmC, 3, 3,(float32_t*) &I3sKmC_f32);

	arm_mat_init_f32(&M, 1, 1,(float32_t*) &M_f32);
	arm_mat_init_f32(&Q, 1, 1,(float32_t*) &Q_f32);
	arm_mat_init_f32(&R, 1, 1,(float32_t*) &R_f32);
}

void KalmanFilter()
{
	// Prediction
	arm_mat_mult_f32(&A, &X, &AmX);//A*X
	arm_mat_mult_f32(&B, &U, &BmU); //B*U
	arm_mat_add_f32(&AmX, &BmU, &X_predict); //X

	arm_mat_mult_f32(&A, &P, &AmP); //A*P
	arm_mat_trans_f32(&A, &At); //At
	arm_mat_mult_f32(&AmP, &At, &AmPmAt); //A*P*At
	arm_mat_mult_f32(&G, &Q, &GmQ); //G*Q
	arm_mat_trans_f32(&G, &Gt); //Gt
	arm_mat_mult_f32(&GmQ, &Gt, &GmQmGt); //G*Q*Gt
	arm_mat_add_f32(&AmPmAt, &GmQmGt, &P_predict); //P

	arm_mat_trans_f32(&C, &Ct); //Ct
	arm_mat_mult_f32(&P_predict, &Ct, &P_predictmCt); //P_predict*Ct
	arm_mat_mult_f32(&C, &P_predictmCt, &CmP_predictmCt); //C*P_predict*Ct
	arm_mat_add_f32(&CmP_predictmCt, &R, &CmP_predictmCtaR); //C*P_predict*Ct + R
	arm_mat_inverse_f32(&CmP_predictmCtaR, &inv_CmP_predictmCtaR);
	arm_mat_mult_f32(&P_predictmCt, &inv_CmP_predictmCtaR, &K); //K

	arm_mat_mult_f32(&C, &X_predict, &CmX_predict); //C*X_predict
	arm_mat_sub_f32(&M, &CmX_predict, &MsCmX_predict); // M - C*X_predict
	arm_mat_mult_f32(&K, &MsCmX_predict, &KmMsCmX_predict); //K*(M - C*X_predict)
	arm_mat_add_f32(&X_predict, &KmMsCmX_predict, &X); //X

	arm_mat_mult_f32(&K, &C, &KmC); // K*C
	arm_mat_sub_f32(&I3, &KmC, &I3sKmC); // I(3) - K*C
	arm_mat_mult_f32(&I3sKmC, &P_predict, &P);
}

#endif /* INC_KALMAN_FILTER_H_ */
