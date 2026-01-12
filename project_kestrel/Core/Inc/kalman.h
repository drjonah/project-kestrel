#ifndef KALMAN_H
#define KALMAN_H

typedef struct {
    // --- Tuning Parameters ---
    float Q_angle;   // Noise variance for the accelerometer
    float Q_bias;    // Nise variance for the gyro bias
    float R_measure; // Measurement noise variance

    // --- State Variables ---
    float angle;     // The angle calculated by the Kalman filter
    float bias;      // The gyro bias calculated by the Kalman filter
    float rate;      // Unbiased rate calculated from the rate and the calculated bias - (newRate - bias)

    // --- Error Covariance Matrix (2x2 Matrix) ---
    // P[0][0] = error in angle
    // P[1][1] = error in bias
    // P[0][1] and P[1][0] = correlation between angle and bias
    float P[2][2];
} Kalman_t;

// Function Prototypes
void Kalman_Init(Kalman_t *kalman);
float Kalman_GetAngle(Kalman_t *kalman, float newAngle, float newRate, float dt);

#endif