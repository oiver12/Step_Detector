#include <stdint.h>
#include <assert.h>
#include "Kalman/kalman.h"
#include "Kalman/cholesky.h"
#include <string.h>
#include <math.h>
#include <stdio.h>

void kalman_filter_init(kalman_t *kf)
{
    matrix_init(&kf->x, KALMAN_NUM_STATES, 1, kf->x_Buffer);
    matrix_init(&kf->z, KALMAN_NUM_MEASUREMENTS, 1, kf->z_Buffer);
    matrix_init(&kf->F, KALMAN_NUM_STATES, KALMAN_NUM_STATES, kf->F_Buffer);
    matrix_init(&kf->u, KALMAN_NUM_INPUTS, 1, kf->u_Buffer);
    matrix_init(&kf->G, KALMAN_NUM_STATES, KALMAN_NUM_INPUTS, kf->G_Buffer);
    matrix_init(&kf->P, KALMAN_NUM_STATES, KALMAN_NUM_STATES, kf->P_Buffer);
    matrix_init(&kf->Q, KALMAN_NUM_STATES, KALMAN_NUM_STATES, kf->Q_Buffer);
    matrix_init(&kf->R, KALMAN_NUM_MEASUREMENTS, KALMAN_NUM_MEASUREMENTS, kf->R_Buffer);
    matrix_init(&kf->H, KALMAN_NUM_MEASUREMENTS, KALMAN_NUM_STATES, kf->H_Buffer);
    matrix_init(&kf->K, KALMAN_NUM_STATES, KALMAN_NUM_MEASUREMENTS, kf->K_Buffer);
    matrix_init(&kf->S, KALMAN_NUM_MEASUREMENTS, KALMAN_NUM_MEASUREMENTS, kf->S_Buffer);
    matrix_init(&kf->y, KALMAN_NUM_MEASUREMENTS, 1, kf->y_Buffer);
    // temporary Matrix
    matrix_init(&kf->temporary.S_inv, KALMAN_NUM_MEASUREMENTS, KALMAN_NUM_MEASUREMENTS, kf->temporary.S_inv_Buffer);
    matrix_init(&kf->temporary.HP, KALMAN_NUM_MEASUREMENTS, KALMAN_NUM_STATES, kf->temporary.HP_Buffer);
    matrix_init(&kf->temporary.PHt, KALMAN_NUM_STATES, KALMAN_NUM_MEASUREMENTS, kf->temporary.PHt_Buffer);
    matrix_init(&kf->temporary.KHP, KALMAN_NUM_STATES, KALMAN_NUM_STATES, kf->temporary.KHP_Buffer);
    matrix_init(&kf->temporary.KH, KALMAN_NUM_STATES, KALMAN_NUM_STATES, kf->temporary.KH_Buffer);
    matrix_init(&kf->temporary.KHtP, KALMAN_NUM_STATES, KALMAN_NUM_STATES, kf->temporary.KHtP_Buffer);
    matrix_init(&kf->temporary.xtemp, KALMAN_NUM_STATES, 1, kf->temporary.xtemp_Buffer);
    matrix_init(&kf->temporary.Gu, KALMAN_NUM_STATES, 1, kf->temporary.Gu_Buffer);
    matrix_init(&kf->temporary.identity, KALMAN_NUM_STATES, KALMAN_NUM_STATES, kf->temporary.identity_Buffer);
    matrix_set(&kf->temporary.identity, 0, 0, 1);
    matrix_set(&kf->temporary.identity, 1, 0, 0);
    matrix_set(&kf->temporary.identity, 0, 1, 0);
    matrix_set(&kf->temporary.identity, 1, 1, 1);
}

void kalman_filter_reset(kalman_t *kf)
{
    matrix_setZero(&kf->x);
    matrix_setZero(&kf->F);
    matrix_setZero(&kf->z);
    matrix_setZero(&kf->u);
    matrix_setZero(&kf->G);
    matrix_setZero(&kf->P);
    matrix_setZero(&kf->Q);
    matrix_setZero(&kf->R);
    matrix_setZero(&kf->H);
    matrix_setZero(&kf->K);
    matrix_setZero(&kf->S);
    matrix_setZero(&kf->y);
    // temporary Matrix
    matrix_setZero(&kf->temporary.S_inv);
    matrix_setZero(&kf->temporary.HP);
    matrix_setZero(&kf->temporary.PHt);
    matrix_setZero(&kf->temporary.KHP);
    matrix_setZero(&kf->temporary.KH);
    matrix_setZero(&kf->temporary.KHtP);
    matrix_setZero(&kf->temporary.xtemp);
    matrix_setZero(&kf->temporary.Gu);
    matrix_setZero(&kf->temporary.identity);
    matrix_set(&kf->temporary.identity, 0, 0, 1);
    matrix_set(&kf->temporary.identity, 1, 0, 0);
    matrix_set(&kf->temporary.identity, 0, 1, 0);
    matrix_set(&kf->temporary.identity, 1, 1, 1);
}

void kalman_update(kalman_t *kf)
{
    // const pointer
    matrix_t *RESTRICT const P = &kf->P;
    // const matrix_t and const pointer
    const matrix_t *RESTRICT const H = &kf->H;
    matrix_t *RESTRICT const K = &kf->K;
    matrix_t *RESTRICT const S = &kf->S;
    matrix_t *RESTRICT const y = &kf->y;
    matrix_t *RESTRICT const x = &kf->x;

    // temporaries
    matrix_data_t *RESTRICT const aux = kf->temporary.aux;
    matrix_t *RESTRICT const Sinv = &kf->temporary.S_inv;
    matrix_t *RESTRICT const temp_HP = &kf->temporary.HP;
    matrix_t *RESTRICT const temp_KH = &kf->temporary.KH;
    matrix_t *RESTRICT const temp_KHP = &kf->temporary.KHP;
    matrix_t *RESTRICT const temp_PHt = &kf->temporary.PHt;
    matrix_t *RESTRICT const temp_KHtP = &kf->temporary.KHtP;
    const matrix_t *RESTRICT const temp_identity = &kf->temporary.identity;
    // printf("---------------Update-------------\n");
    // printf("K1=");
    // matrix_print(K);
    /*******************************************/
    /*  K = P*H'*(H*P*H'+R)^-1                 */
    /*******************************************/
    // TODO: vereinfachen: nur ein PH'
    //  S = H*P*H' + R
    matrix_mult(H, P, temp_HP, aux);   // temp_HP = H*P
    matrix_mult_transb(temp_HP, H, S); // S = temp_HP*H'
    matrix_add_inplace(S, &kf->R);     // S += R
    // K = P*H' * S^-1
    int result = cholesky_decompose_lower(S);
    matrix_invert_lower(S, Sinv);        // Sinv = S^-1
    matrix_mult_transb(P, H, temp_PHt);  // temp_Pht = P*H'
    matrix_mult(temp_PHt, Sinv, K, aux); // K = temp_Pht*S^-1
    // printf("K2 %i=", result);
    // matrix_print(K);
    // printf("X1=");
    // matrix_print(x);
    /*******************************************/
    /*  x = x + K*(z - H*x)                    */
    /*******************************************/
    // y = z - H*x
    matrix_mult_rowvector(H, x, y);
    matrix_sub_inplace_b(&kf->z, y);
    // x = x + K*y
    matrix_multadd_rowvector(K, y, x);
    // printf("X2=");
    // matrix_print(x);
    // printf("P1=");
    // matrix_print(P);
    /*******************************************/
    /*  P = (1 - K*H)*P*(1 - KH)' + KRK'        */
    /********************************************/
    matrix_mult(K, H, temp_KH, aux);                  // temp_KH = K*H
    matrix_sub_inplace_b(temp_identity, temp_KH);     // temp_KH = 1 - K*H
    matrix_mult(temp_KH, P, temp_KHP, aux);           // temp_KHP = (1 - K*H)*P
    matrix_mult_transb(temp_KHP, temp_KH, temp_KHtP); // temp_KHtP = (1 - K*H)*P*(1 - K*H)'
    matrix_mult(K, &kf->R, temp_PHt, aux);            // temp_PHt = K*R
    matrix_mult_transb(temp_PHt, K, temp_KH);         // temp_KH = K*R*K'
    matrix_add(temp_KHtP, temp_KH, P);                // P = (1 - K*H)*P*(1 - K*H)' + K*R*K'
    // printf("P2=");
    // matrix_print(P);
}

void kalman_predict(kalman_t *kf)
{
    // matrices and vectors
    // const matrix_t and const pointer
    const matrix_t *RESTRICT const F = &kf->F;
    // const pointer
    matrix_t *RESTRICT const x = &kf->x;
    const matrix_t *RESTRICT const G = &kf->G;
    const matrix_t *RESTRICT const u = &kf->u;
    matrix_t *RESTRICT const P = &kf->P;
    const matrix_t *RESTRICT const Q = &kf->Q;

    // temporaries
    matrix_data_t *RESTRICT const aux = kf->temporary.aux;
    matrix_t *RESTRICT const xtemp = &kf->temporary.xtemp;
    matrix_t *RESTRICT const temp_Gu = &kf->temporary.Gu;
    matrix_t *RESTRICT const temp_FP = &kf->temporary.KHP;
    // printf("---------------Predict---------------\n");
    // printf("X1=");
    // matrix_print(x);
    /************************************************************************/
    /* x = F*x + G*u                                                        */
    /************************************************************************/
    matrix_mult_rowvector(F, x, xtemp); // xtemp = F*x
    matrix_mult(G, u, temp_Gu, aux);    // temp_Gu = G*u
    matrix_add(xtemp, temp_Gu, x);      // x = xtemp + temp_Gu
    // printf("X2=");
    // matrix_print(x);
    // printf("P1=");
    // matrix_print(P);
    /************************************************************************/
    /* P = F*P*F' + Q                                                       */
    /************************************************************************/
    matrix_mult(F, P, temp_FP, aux);   // temp_FP = F*P
    matrix_mult_transb(temp_FP, F, P); // P = temp_FP*F'
    matrix_add_inplace(P, Q);          // P = P + Q
    // printf("P2=");
    // matrix_print(P);
}

