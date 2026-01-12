#ifndef KALMAN_H
#define KALMAN_H

typedef struct {
    // --- Tuning Parameters (The "Knobs") ---
    float Q_angle;   // Process noise variance for the accelerometer
    float Q_bias;    // Process noise variance for the gyro bias
    float R_measure; // Measurement noise variance

    // --- State Variables (The "Memory") ---
    float angle;     // The angle calculated by the Kalman filter - part of the 2x1 state vector
    float bias;      // The gyro bias calculated by the Kalman filter - part of the 2x1 state vector
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