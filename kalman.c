#include "kalman.h"

struct Kalman init_kalman ( struct Kalman k){
    k.Q_angle = 0.001f;
    k.Q_bias = 0.003f;
    k.R_measure = 0.03f;

    k.angle;
    k.bias = 0.0f;

    k.P[0][0] = 0.0f;
    k.P[0][1] = 0.0f;
    k.P[1][0] = 0.0f;
    k.P[1][1] = 0.0f;

    return k;
};

// The angle should be in degrees and the rate should be in degrees per second and the delta time in seconds
float getAngle(struct Kalman k, float new_angle, float new_rate, float dt) {
    k.rate = new_rate - k.bias;
    k.angle += dt * k.rate;

    // Update estimation error covariance - Project the error covariance ahead
    /* Step 2 */
    k.P[0][0] += dt * (dt * k.P[1][1] - k.P[0][1] - k.P[1][0] + k.Q_angle);
    k.P[0][1] -= dt * k.P[1][1];
    k.P[1][0] -= dt * k.P[1][1];
    k.P[1][1] += k.Q_bias * dt;

    // Discrete Kalman filter measurement update equations - Measurement Update ("Correct")
    // Calculate Kalman gain - Compute the Kalman gain
    /* Step 4 */
    float S = k.P[0][0] + k.R_measure; // Estimate error
    /* Step 5 */
    float K[2]; // Kalman gain - This is a 2x1 vector
    K[0] = k.P[0][0] / S;
    K[1] = k.P[1][0] / S;

    // Calculate angle and bias - Update estimate with measurement zk (new_angle)
    /* Step 3 */
    float y = new_angle - k.angle; // Angle difference
    /* Step 6 */
    k.angle += K[0] * y;
    k.bias += K[1] * y;

    // Calculate estimation error covariance - Update the error covariance
    /* Step 7 */
    float P00_temp = k.P[0][0];
    float P01_temp = k.P[0][1];

    k.P[0][0] -= K[0] * P00_temp;
    k.P[0][1] -= K[0] * P01_temp;
    k.P[1][0] -= K[1] * P00_temp;
    k.P[1][1] -= K[1] * P01_temp;

    return k.angle;
};
