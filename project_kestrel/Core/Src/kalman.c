#include "kalman.h"

void Kalman_Init(Kalman_t *kalman) {
    // 1. Set initial tuning parameters (You can tweak these!)
    // Q_angle: Trust the gyro integration. Lower = smoother but slower drift correction.
    kalman->Q_angle = 0.001f;
    
    // Q_bias: Trust that the gyro bias doesn't change quickly.
    kalman->Q_bias = 0.003f;
    
    // R_measure: Trust the accelerometer. Higher = handle more vibration, but more lag.
    kalman->R_measure = 0.03f;

    // 2. Reset state
    kalman->angle = 0.0f; // Start at 0 degrees
    kalman->bias = 0.0f;  // Start assuming no bias
    kalman->rate = 0.0f;

    // 3. Reset covariance matrix (P)
    // We start with 0 error because we force the angle to 0.
    kalman->P[0][0] = 0.0f;
    kalman->P[0][1] = 0.0f;
    kalman->P[1][0] = 0.0f;
    kalman->P[1][1] = 0.0f;
}

float Kalman_GetAngle(Kalman_t *kalman, float newAngle, float newRate, float dt) {
    
    /* ====================================================================
       STEP 1: PREDICT (Time Update)
       Project the state ahead based on Gyro rate and dt
       ==================================================================== */

    // 1. Predict the angle
    // Formula: angle = angle + (gyro_rate - gyro_bias) * dt
    kalman->rate = newRate - kalman->bias;
    kalman->angle += dt * kalman->rate;

    // 2. Predict the Error Covariance Matrix (P)
    // This is the expansion of: P = F * P * F' + Q
    // We assume the system is linear and depends on dt.
    kalman->P[0][0] += dt * (dt * kalman->P[1][1] - kalman->P[0][1] - kalman->P[1][0] + kalman->Q_angle);
    kalman->P[0][1] -= dt * kalman->P[1][1];
    kalman->P[1][0] -= dt * kalman->P[1][1];
    kalman->P[1][1] += kalman->Q_bias * dt;

    /* ====================================================================
       STEP 2: UPDATE (Measurement Update)
       Correct the prediction using the Accelerometer angle
       ==================================================================== */

    // 3. Calculate the Innovation (Difference between predicted and measured)
    // Formula: y = z - H * x
    float y = newAngle - kalman->angle;

    // 4. Calculate Innovation Covariance (S)
    // Formula: S = H * P * H' + R
    float S = kalman->P[0][0] + kalman->R_measure;

    // 5. Calculate Kalman Gain (K)
    // Formula: K = P * H' * inv(S)
    float K[2];
    K[0] = kalman->P[0][0] / S; // Gain for Angle
    K[1] = kalman->P[1][0] / S; // Gain for Bias

    // 6. Update the Angle and Bias state
    // Formula: x = x + K * y
    kalman->angle += K[0] * y;
    kalman->bias += K[1] * y;

    // 7. Update the Error Covariance Matrix (P)
    // Formula: P = (I - K * H) * P
    float P00_temp = kalman->P[0][0];
    float P01_temp = kalman->P[0][1];

    kalman->P[0][0] -= K[0] * P00_temp;
    kalman->P[0][1] -= K[0] * P01_temp;
    kalman->P[1][0] -= K[1] * P00_temp;
    kalman->P[1][1] -= K[1] * P01_temp;

    // Return the clean, filtered angle
    return kalman->angle;
}