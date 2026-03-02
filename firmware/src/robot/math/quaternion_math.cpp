#ifdef ROLE_ROBOT

#include "quaternion_math.h"
#include <math.h>

void multiplyQuaternions(float w1, float x1, float y1, float z1,
                         float w2, float x2, float y2, float z2,
                         float* rw, float* rx, float* ry, float* rz) {
    *rw = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2;
    *rx = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2;
    *ry = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2;
    *rz = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2;
}

void quaternionToEuler(float qw, float qx, float qy, float qz, float* roll, float* pitch, float* yaw) {
    // Pitch (Y-axis) - This is the balancing angle
    float sinp = 2.0f * (qw * qy - qz * qx);
    if (fabs(sinp) >= 1.0f) {
        *pitch = copysign(90.0f, sinp);
    } else {
        *pitch = asin(sinp) * 180.0f / M_PI;
    }

    // Roll (X-axis) - Side-to-side tilt
    *roll = atan2(2.0f * (qw * qx + qy * qz), 1.0f - 2.0f * (qx * qx + qy * qy)) * 180.0f / M_PI;

    // Yaw (Z-axis) - Turning/Heading
    *yaw = atan2(2.0f * (qw * qz + qx * qy), 1.0f - 2.0f * (qy * qy + qz * qz)) * 180.0f / M_PI;
}

#endif // ROLE_ROBOT