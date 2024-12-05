#ifndef _KALMAN_H_
#define _KALMAN_H_

#include <stdint.h>
#include "Kalman/matrix.h"

#define KALMAN_NUM_STATES 2       // height, velocity
#define KALMAN_NUM_INPUTS 1       // vertical acceleration
#define KALMAN_NUM_MEASUREMENTS 1 // height from Barometer

#define KALMAN_AUX_ROWS_SIZE ((KALMAN_NUM_STATES > KALMAN_NUM_MEASUREMENTS) ? KALMAN_NUM_STATES : KALMAN_NUM_MEASUREMENTS)

typedef struct
{
    // State Vector
    matrix_t x;
    matrix_data_t x_Buffer[KALMAN_NUM_STATES * 1];
    // Output Vector
    matrix_t z;
    matrix_data_t z_Buffer[KALMAN_NUM_MEASUREMENTS * 1];
    // State Transition Matrix
    matrix_t F;
    matrix_data_t F_Buffer[KALMAN_NUM_STATES * KALMAN_NUM_STATES];
    // Input Variable
    matrix_t u;
    matrix_data_t u_Buffer[KALMAN_NUM_INPUTS * 1];
    // Control Matrix
    matrix_t G;
    matrix_data_t G_Buffer[KALMAN_NUM_STATES * KALMAN_NUM_INPUTS];
    // Estimate Uncertainty
    matrix_t P;
    matrix_data_t P_Buffer[KALMAN_NUM_STATES * KALMAN_NUM_STATES];
    // Process Noise Uncertainty
    matrix_t Q;
    matrix_data_t Q_Buffer[KALMAN_NUM_STATES * KALMAN_NUM_STATES];
    // Measurement Uncertainty
    matrix_t R;
    matrix_data_t R_Buffer[KALMAN_NUM_MEASUREMENTS * KALMAN_NUM_MEASUREMENTS];
    // Oberservation Matrix
    matrix_t H;
    matrix_data_t H_Buffer[KALMAN_NUM_MEASUREMENTS * KALMAN_NUM_STATES];
    // Kalman Gain
    matrix_t K;
    matrix_data_t K_Buffer[KALMAN_NUM_STATES * KALMAN_NUM_MEASUREMENTS];
    // Residual covariance matrix
    matrix_t S;
    matrix_data_t S_Buffer[KALMAN_NUM_MEASUREMENTS * KALMAN_NUM_MEASUREMENTS];
    // Innovativ vector
    matrix_t y;
    matrix_data_t y_Buffer[KALMAN_NUM_MEASUREMENTS * 1];
    struct
    {
        matrix_data_t aux[KALMAN_AUX_ROWS_SIZE * 1];
        matrix_t S_inv;
        matrix_data_t S_inv_Buffer[KALMAN_NUM_MEASUREMENTS * KALMAN_NUM_MEASUREMENTS];
        matrix_t HP;
        matrix_data_t HP_Buffer[KALMAN_NUM_MEASUREMENTS * KALMAN_NUM_STATES];
        matrix_t KH;
        matrix_data_t KH_Buffer[KALMAN_NUM_STATES * KALMAN_NUM_STATES];
        matrix_t KHP;
        matrix_data_t KHP_Buffer[KALMAN_NUM_STATES * KALMAN_NUM_STATES];
        matrix_t KHtP;
        matrix_data_t KHtP_Buffer[KALMAN_NUM_STATES * KALMAN_NUM_STATES];
        matrix_t PHt;
        matrix_data_t PHt_Buffer[KALMAN_NUM_STATES * KALMAN_NUM_MEASUREMENTS];
        matrix_t identity;
        matrix_data_t identity_Buffer[KALMAN_NUM_STATES * KALMAN_NUM_STATES];
        matrix_t xtemp;
        matrix_data_t xtemp_Buffer[KALMAN_NUM_STATES * 1];
        matrix_t Gu;
        matrix_data_t Gu_Buffer[KALMAN_NUM_STATES * 1];
    } temporary;

} kalman_t;

void kalman_filter_init(kalman_t *kf);
void kalman_filter_reset(kalman_t *kf);

void kalman_update(kalman_t *kf);

void kalman_predict(kalman_t *kf);

#endif
