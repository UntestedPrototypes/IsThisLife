#ifndef VECTORS_H
#define VECTORS_H

#include <stdint.h>
#include <Arduino.h>


/* =========================
   Generic 3D vector
   ========================= */
struct Vec3 {
    float x;
    float y;
    float z;
};

/* =========================
   Quaternion
   ========================= */
struct Quaternion {
    float w;
    float x;
    float y;
    float z;
};

/* =========================
   System state
   ========================= */
struct SystemState {
    Vec3 position;      // x, y, z (z optional if unused)
    Vec3 orientation;  
     /*   Geometry meaning:
    systemState.orientation.x = phi    // rod azimuth (world x–y plane)
    systemState.orientation.y = theta  // rod polar angle from +Z
    systemState.orientation.z = psi    // in-plane spin about rod axis
    */
    uint32_t timestamp_ms;
};

/* =========================
   External instances
   ========================= */
extern Vec3 vecZero;
extern Quaternion quatIdentity;
extern SystemState systemState;

#endif // VECTORS_H
