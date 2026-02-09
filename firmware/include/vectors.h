#ifdef ROLE_ROBOT

#ifndef VECTORS_H
#define VECTORS_H

#include <math.h>
#include <stdint.h>

/* =========================================================
   Generic 3D vector
   ========================================================= */
struct Vec3 {
    float x;
    float y;
    float z;

    constexpr Vec3() : x(0.0f), y(0.0f), z(0.0f) {}
    constexpr Vec3(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}
};

/* =========================================================
   Math Constants & Utilities
   ========================================================= */
#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

inline float wrapAngle(float angle) {
    while (angle > M_PI)  angle -= 2.0f * M_PI;
    while (angle <= -M_PI) angle += 2.0f * M_PI;
    return angle;
}

// Vec3 Operations
inline float vecDot(const Vec3& a, const Vec3& b) {
    return a.x*b.x + a.y*b.y + a.z*b.z;
}

inline constexpr Vec3 operator+(const Vec3& a, const Vec3& b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
inline constexpr Vec3 operator-(const Vec3& a, const Vec3& b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
inline constexpr Vec3 operator*(const Vec3& v, float s) { return {v.x * s, v.y * s, v.z * s}; }

#endif // VECTORS_H
#endif // ROLE_ROBOT