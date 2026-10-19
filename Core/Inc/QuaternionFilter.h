/* Includes adapted Arduino orientation-filter code.
 * Original library/version is no longer known; see THIRD_PARTY.md.
 * The project MIT grant does not relicense these adapted portions.
 */
/*
 * QuaternionFilter.h
 *
 *  Created on: Nov 28, 2024
 *      Author: olive
 */

#ifndef INC_QUATERNIONFILTER_H_
#define INC_QUATERNIONFILTER_H_

#define g ((float) 9.81f)

typedef struct {
    float phi_rad;
    float theta_rad;

    float P[4];
    float Q[2];
    float R[3];
} KalmanRollPitch_t;

void QuaternionFilter_KalmanInit(KalmanRollPitch_t *klm, float Pinit, float *Q, float *R);
void QuaternionFiler_KalmanPredict(KalmanRollPitch_t *kal, float *gyr_rps, float T);
void QuaternionFilter_KalmanUpdate(KalmanRollPitch_t *kal, float *acc_mps2);

void QuaternionFilter_init();
void QuaternionFilter_Madgwick(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, float deltaT, float* q);
void QuaternionFilter_Mahony(float ax, float ay, float az, float gx, float gy, float gz, float deltaT, float* q);
void Quaternion_get_RPY(float qw, float qx, float qy, float qz, float* roll, float* pitch, float* yaw);


#endif /* INC_QUATERNIONFILTER_H_ */
