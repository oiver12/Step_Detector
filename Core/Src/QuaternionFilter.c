/*
 * QuaternionFilter.c
 *
 *  Created on: Nov 28, 2024
 *      Author: olive
 */
#include "QuaternionFilter.h"

#include <math.h>
#include <stdio.h>

#define PI 3.14159265358979323846

// für Madgwick
float GyroMeasError = PI * (40.0f / 180.0f); // gyroscope measurement error in rads/s (start at 40 deg/s)
float GyroMeasDrift = PI * (0.0f / 180.0f);  // gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)
float beta;                                  // compute beta
float zeta;                                  // compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value

float twoKp = (2.0f * 5.0f); // 2 * proportional gain (Kp)
float twoKi = (2.0f * 0.0f);
float integralFBx = 0.0f, integralFBy = 0.0f, integralFBz = 0.0f; // integral error terms scaled by Ki

/* für mahony */
float Kp = 30.0;
float Ki = 0.0;

void QuaternionFilter_KalmanInit(KalmanRollPitch_t *klm, float Pinit, float *Q, float *R)
{
    klm->phi_rad = 0.0f;
    klm->theta_rad = 0.0f;

    klm->P[0] = Pinit;
    klm->P[1] = 0.0f;
    klm->P[2] = 0.0f;
    klm->P[3] = Pinit;

    klm->Q[0] = Q[0];
    klm->Q[1] = Q[1];
    klm->R[0] = R[0];
    klm->R[1] = R[1];
    klm->R[2] = R[2];
}

void QuaternionFiler_KalmanPredict(KalmanRollPitch_t *kal, float *gyr_rps, float T)
{
    float p = gyr_rps[0];
    float q = gyr_rps[1];
    float r = gyr_rps[2];

    float sp = sin(kal->phi_rad);
    float cp = cos(kal->phi_rad);
    float tt = tan(kal->theta_rad);

    kal->phi_rad = kal->phi_rad + T * (p + tt * (q * sp + r * cp));
    kal->theta_rad = kal->theta_rad + T * (q * cp - r * sp);

    sp = sin(kal->phi_rad);
    cp = cos(kal->phi_rad);
    float st = sin(kal->theta_rad);
    float ct = cos(kal->theta_rad);
    tt = st / ct;

    float A[4] = {tt * (q * cp - r * sp), (r * cp + q * sp) * (tt * tt + 1.0f),
                  -(r * cp + q * sp), 0.0f};
    float Ptmp[4] = {T * (kal->Q[0] + 2.0f * A[0] * kal->P[0] + A[1] * kal->P[1] + A[1] * kal->P[2]), T * (A[0] * kal->P[1] + A[2] * kal->P[0] + A[1] * kal->P[3] + A[3] * kal->P[1]),
                     T * (A[0] * kal->P[2] + A[2] * kal->P[0] + A[1] * kal->P[3] + A[3] * kal->P[2]), T * (kal->Q[1] + A[2] * kal->P[1] + A[2] * kal->P[2] + 2.0f * A[3] * kal->P[3])};

    kal->P[0] = kal->P[0] + Ptmp[0];
    kal->P[1] = kal->P[1] + Ptmp[1];
    kal->P[2] = kal->P[2] + Ptmp[2];
    kal->P[3] = kal->P[3] + Ptmp[3];
}

void QuaternionFilter_KalmanUpdate(KalmanRollPitch_t *kal, float *acc_mps2)
{
    float ax = acc_mps2[0];
    float ay = acc_mps2[1];
    float az = acc_mps2[2];

    float sp = sin(kal->phi_rad);
    float cp = cos(kal->phi_rad);
    float st = sin(kal->theta_rad);
    float ct = cos(kal->theta_rad);

    float h[3] = {g * st,
                  -g * ct * sp,
                  -g * ct * cp};
    float C[6] = {0.0f, g * ct,
                  -g * cp * ct, g * sp * st,
                  g * sp * ct, g * cp * st};

    float G[9] = {kal->P[3] * C[1] * C[1] + kal->R[0], C[1] * C[2] * kal->P[2] + C[1] * C[3] * kal->P[3], 0.0f,
                  C[1] * (C[2] * kal->P[1] + C[3] * kal->P[3]), kal->R[1] + C[2] * (C[2] * kal->P[0] + C[3] * kal->P[2]) + C[3] * (C[2] * kal->P[1] + C[3] * kal->P[3]), C[4],
                  C[1] * (C[4] * kal->P[1] + C[5] * kal->P[3]), C[2] * (C[4] * kal->P[0] + C[5] * kal->P[2]) + C[3] * (C[4] * kal->P[1] + C[5] * kal->P[3]), kal->R[2]};

    // float Gdetinv = 1.0f / (G[0] * G[4] * G[8] - G[0] * G[5] * G[7] - G[1] * G[3] * G[8] + G[1] * G[5] * G[6] + G[2] * G[3] * G[7] - G[2] * G[4] * G[6]);

    // float Ginv[9] = { Gdetinv * (G[4]*G[8] - G[5]*G[7]), -Gdetinv * (G[1]*G[8] - G[2]*G[7]), Gdetinv * (G[1]*G[5] - G[2]*G[4]),
    //                  -Gdetinv * (G[3]*G[8] - G[5]*G[6]),  Gdetinv * (G[0]*G[8] - G[2]*G[6]),-Gdetinv * (G[0]*G[5] - G[2]*G[3]),
    //                   Gdetinv * (G[3]*G[7] - G[4]*G[6]), -Gdetinv * (G[0]*G[7] - G[1]*G[6]), Gdetinv * (G[0]*G[4] - G[1]*G[3]) };

    float K[6] = {G[0] * (C[0] * kal->P[0] + C[1] * kal->P[1]) + G[3] * (C[2] * kal->P[0] + C[3] * kal->P[1]) + G[6] * (C[4] * kal->P[0] + C[5] * kal->P[1]), G[1] * (C[0] * kal->P[0] + C[1] * kal->P[1]) + G[4] * (C[2] * kal->P[0] + C[3] * kal->P[1]) + G[7] * (C[4] * kal->P[0] + C[5] * kal->P[1]), G[2] * (C[0] * kal->P[0] + C[1] * kal->P[1]) + G[5] * (C[2] * kal->P[0] + C[3] * kal->P[1]) + G[8] * (C[4] * kal->P[0] + C[5] * kal->P[1]),
                  G[0] * (C[0] * kal->P[2] + C[1] * kal->P[3]) + G[3] * (C[2] * kal->P[2] + C[3] * kal->P[3]) + G[6] * (C[4] * kal->P[2] + C[5] * kal->P[3]), G[1] * (C[0] * kal->P[2] + C[1] * kal->P[3]) + G[4] * (C[2] * kal->P[2] + C[3] * kal->P[3]) + G[7] * (C[4] * kal->P[2] + C[5] * kal->P[3]), G[2] * (C[0] * kal->P[2] + C[1] * kal->P[3]) + G[5] * (C[2] * kal->P[2] + C[3] * kal->P[3]) + G[8] * (C[4] * kal->P[2] + C[5] * kal->P[3])};

    float Ptmp[4];
    Ptmp[0] = -kal->P[0] * (C[0] * K[0] + C[2] * K[1] + C[4] * K[2] - 1.0f) - kal->P[2] * (C[1] * K[0] + C[3] * K[1] + C[5] * K[2]);
    Ptmp[1] = -kal->P[1] * (C[0] * K[0] + C[2] * K[1] + C[4] * K[2] - 1.0f) - kal->P[3] * (C[1] * K[0] + C[3] * K[1] + C[5] * K[2]);
    Ptmp[2] = -kal->P[0] * (C[0] * K[3] + C[2] * K[4] + C[4] * K[5]) - kal->P[2] * (C[1] * K[3] + C[3] * K[4] + C[5] * K[5] - 1.0f);
    Ptmp[3] = -kal->P[1] * (C[0] * K[3] + C[2] * K[4] + C[4] * K[5]) - kal->P[3] * (C[1] * K[3] + C[3] * K[4] + C[5] * K[5] - 1.0f);

    kal->P[0] = kal->P[0] + Ptmp[0];
    kal->P[1] = kal->P[1] + Ptmp[1];
    kal->P[2] = kal->P[2] + Ptmp[2];
    kal->P[3] = kal->P[3] + Ptmp[3];

    kal->phi_rad = kal->phi_rad + K[0] * (ax - h[0]) + K[1] * (ay - h[1]) + K[2] * (az - h[2]);
    kal->theta_rad = kal->theta_rad + K[3] * (ax - h[0]) + K[4] * (ay - h[1]) + K[5] * (az - h[2]);
}

void QuaternionFilter_init()
{
    beta = sqrtf(3.0f / 4.0f) * GyroMeasError;
    zeta = sqrtf(3.0f / 4.0f) * GyroMeasDrift;
}

// Madgwick Quaternion Update
void QuaternionFilter_Madgwick(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, float deltaT, float *q)
{
    double q0 = q[0], q1 = q[1], q2 = q[2], q3 = q[3]; // short name local variable for readability
    double recipNorm;
    double s0, s1, s2, s3;
    double qDot1, qDot2, qDot3, qDot4;
    double hx, hy;
    double _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz, _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;

    // Rate of change of quaternion from gyroscope
    qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
    qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
    qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
    qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

    // Normalise accelerometer measurement
    double a_norm = ax * ax + ay * ay + az * az;
    if (a_norm == 0.)
        return; // handle NaN
    recipNorm = 1.0 / sqrt(a_norm);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    // Normalise magnetometer measurement
    double m_norm = mx * mx + my * my + mz * mz;
    if (m_norm == 0.)
        return; // handle NaN
    recipNorm = 1.0 / sqrt(m_norm);
    mx *= recipNorm;
    my *= recipNorm;
    mz *= recipNorm;

    // Auxiliary variables to avoid repeated arithmetic
    _2q0mx = 2.0f * q0 * mx;
    _2q0my = 2.0f * q0 * my;
    _2q0mz = 2.0f * q0 * mz;
    _2q1mx = 2.0f * q1 * mx;
    _2q0 = 2.0f * q0;
    _2q1 = 2.0f * q1;
    _2q2 = 2.0f * q2;
    _2q3 = 2.0f * q3;
    _2q0q2 = 2.0f * q0 * q2;
    _2q2q3 = 2.0f * q2 * q3;
    q0q0 = q0 * q0;
    q0q1 = q0 * q1;
    q0q2 = q0 * q2;
    q0q3 = q0 * q3;
    q1q1 = q1 * q1;
    q1q2 = q1 * q2;
    q1q3 = q1 * q3;
    q2q2 = q2 * q2;
    q2q3 = q2 * q3;
    q3q3 = q3 * q3;

    // Reference direction of Earth's magnetic field
    hx = mx * q0q0 - _2q0my * q3 + _2q0mz * q2 + mx * q1q1 + _2q1 * my * q2 + _2q1 * mz * q3 - mx * q2q2 - mx * q3q3;
    hy = _2q0mx * q3 + my * q0q0 - _2q0mz * q1 + _2q1mx * q2 - my * q1q1 + my * q2q2 + _2q2 * mz * q3 - my * q3q3;
    _2bx = sqrt(hx * hx + hy * hy);
    _2bz = -_2q0mx * q2 + _2q0my * q1 + mz * q0q0 + _2q1mx * q3 - mz * q1q1 + _2q2 * my * q3 - mz * q2q2 + mz * q3q3;
    _4bx = 2.0f * _2bx;
    _4bz = 2.0f * _2bz;

    // Gradient decent algorithm corrective step
    s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - ax) + _2q1 * (2.0f * q0q1 + _2q2q3 - ay) - _2bz * q2 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q3 + _2bz * q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
    s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - ax) + _2q0 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q1 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + _2bz * q3 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q2 + _2bz * q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q3 - _4bz * q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
    s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - ax) + _2q3 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q2 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + (-_4bx * q2 - _2bz * q0) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q1 + _2bz * q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q0 - _4bz * q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
    s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - ax) + _2q2 * (2.0f * q0q1 + _2q2q3 - ay) + (-_4bx * q3 + _2bz * q1) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q0 + _2bz * q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
    recipNorm = 1.0 / sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
    s0 *= recipNorm;
    s1 *= recipNorm;
    s2 *= recipNorm;
    s3 *= recipNorm;

    // Apply feedback step
    qDot1 -= beta * s0;
    qDot2 -= beta * s1;
    qDot3 -= beta * s2;
    qDot4 -= beta * s3;

    // Integrate rate of change of quaternion to yield quaternion
    q0 += qDot1 * deltaT;
    q1 += qDot2 * deltaT;
    q2 += qDot3 * deltaT;
    q3 += qDot4 * deltaT;

    // Normalise quaternion
    recipNorm = 1.0 / sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;

    q[0] = q0;
    q[1] = q1;
    q[2] = q2;
    q[3] = q3;
}

void QuaternionFilter_Mahony(float ax, float ay, float az, float gx, float gy, float gz, float deltaT, float *q)
{
    float recipNorm;
    float vx, vy, vz;
    float ex, ey, ez; // error terms
    float qa, qb, qc;
    static float ix = 0.0, iy = 0.0, iz = 0.0; // integral feedback terms
    float tmp;

    // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
    tmp = ax * ax + ay * ay + az * az;
    if (tmp > 0.0)
    {
        // Normalise accelerometer (assumed to measure the direction of gravity in body frame)
        recipNorm = 1.0 / sqrt(tmp);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        // Estimated direction of gravity in the body frame (factor of two divided out)
        vx = q[1] * q[3] - q[0] * q[2];
        vy = q[0] * q[1] + q[2] * q[3];
        vz = q[0] * q[0] - 0.5f + q[3] * q[3];

        // Error is cross product between estimated and measured direction of gravity in body frame
        // (half the actual magnitude)
        ex = (ay * vz - az * vy);
        ey = (az * vx - ax * vz);
        ez = (ax * vy - ay * vx);

        // Compute and apply to gyro term the integral feedback, if enabled
        if (Ki > 0.0f)
        {
            ix += Ki * ex * deltaT; // integral error scaled by Ki
            iy += Ki * ey * deltaT;
            iz += Ki * ez * deltaT;
            gx += ix; // apply integral feedback
            gy += iy;
            gz += iz;
        }

        // Apply proportional feedback to gyro term
        gx += Kp * ex;
        gy += Kp * ey;
        gz += Kp * ez;
    }

    // Integrate rate of change of quaternion, q cross gyro term
    deltaT = 0.5 * deltaT;
    gx *= deltaT; // pre-multiply common factors
    gy *= deltaT;
    gz *= deltaT;
    qa = q[0];
    qb = q[1];
    qc = q[2];
    q[0] += (-qb * gx - qc * gy - q[3] * gz);
    q[1] += (qa * gx + qc * gz - q[3] * gy);
    q[2] += (qa * gy - qb * gz + q[3] * gx);
    q[3] += (qa * gz + qb * gy - qc * gx);

    // renormalise quaternion
    recipNorm = 1.0 / sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
    q[0] = q[0] * recipNorm;
    q[1] = q[1] * recipNorm;
    q[2] = q[2] * recipNorm;
    q[3] = q[3] * recipNorm;
}

void Quaternion_get_RPY(float qw, float qx, float qy, float qz, float *roll, float *pitch, float *yaw)
{
    float magnetic_declination = 3;
    float a12, a22, a31, a32, a33; // rotation matrix coefficients for Euler angles and gravity components
    a12 = 2.0f * (qx * qy + qw * qz);
    a22 = qw * qw + qx * qx - qy * qy - qz * qz;
    a31 = 2.0f * (qw * qx + qy * qz);
    a32 = 2.0f * (qx * qz - qw * qy);
    a33 = qw * qw - qx * qx - qy * qy + qz * qz;
    *roll = atan2f(a31, a33);
    *pitch = -asinf(a32);
    *yaw = atan2f(a12, a22);
    *roll *= 180.0f / PI;
    *pitch *= 180.0f / PI;
    *yaw *= 180.0f / PI;
    *yaw += magnetic_declination;
    if (*yaw >= +180.f)
        *yaw -= 360.f;
    else if (*yaw < -180.f)
        *yaw += 360.f;
}

