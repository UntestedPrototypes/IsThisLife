#ifndef VECTORS_H
#define VECTORS_H

#include <stdint.h>
#include <math.h>
#include <utility/imumaths.h>   // imu::Vector<3>

/* =========================================================
   System-wide state (TEMPORARY: kept here by request)
   ========================================================= */
struct Vec3;   // forward declaration

struct SystemState {
    Vec3 position;
    Vec3 orientation;
    uint32_t timestamp_ms;
};

/* =========================================================
   Generic 3D vector
   ========================================================= */
struct Vec3 {
    float x;
    float y;
    float z;

    // Default constructor (zero-initialized)
    constexpr Vec3() : x(0.0f), y(0.0f), z(0.0f) {}
    constexpr Vec3(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}

    // Interop with Adafruit IMU math
    Vec3(const imu::Vector<3>& v) : x(v.x()), y(v.y()), z(v.z()) {}
};

/* =========================================================
   Vec3 <-> Vec3 operators
   ========================================================= */

inline constexpr Vec3 operator+(const Vec3& a, const Vec3& b) {
    return Vec3{a.x + b.x, a.y + b.y, a.z + b.z};
}

inline constexpr Vec3 operator-(const Vec3& a, const Vec3& b) {
    return Vec3{a.x - b.x, a.y - b.y, a.z - b.z};
}

inline Vec3& operator+=(Vec3& a, const Vec3& b) {
    a.x += b.x; a.y += b.y; a.z += b.z;
    return a;
}

inline Vec3& operator-=(Vec3& a, const Vec3& b) {
    a.x -= b.x; a.y -= b.y; a.z -= b.z;
    return a;
}

inline constexpr Vec3 operator*(const Vec3& v, float s) {
    return Vec3{v.x * s, v.y * s, v.z * s};
}

inline constexpr Vec3 operator*(float s, const Vec3& v) {
    return v * s;
}

inline constexpr Vec3 operator/(const Vec3& v, float s) {
    return Vec3{v.x / s, v.y / s, v.z / s};
}

inline Vec3& operator*=(Vec3& v, float s) {
    v.x *= s; v.y *= s; v.z *= s;
    return v;
}

inline Vec3& operator/=(Vec3& v, float s) {
    v.x /= s; v.y /= s; v.z /= s;
    return v;
}

/* =========================================================
   Interop with imu::Vector<3>
   ========================================================= */

inline Vec3& operator+=(Vec3& a, const imu::Vector<3>& b) {
    a.x += b.x();
    a.y += b.y();
    a.z += b.z();
    return a;
}

inline Vec3& operator-=(Vec3& a, const imu::Vector<3>& b) {
    a.x -= b.x();
    a.y -= b.y();
    a.z -= b.z();
    return a;
}

inline Vec3 operator+(Vec3 a, const imu::Vector<3>& b) {
    a += b;
    return a;
}

inline Vec3 operator-(Vec3 a, const imu::Vector<3>& b) {
    a -= b;
    return a;
}

inline imu::Vector<3> operator-(const imu::Vector<3>& a, const Vec3& b) {
    imu::Vector<3> out = a;
    out[0] = a.x() - b.x;
    out[1] = a.y() - b.y;
    out[2] = a.z() - b.z;
    return out;
}

/* =========================================================
   Vector math utilities
   ========================================================= */

inline float vecDot(const Vec3& a, const Vec3& b) {
    return a.x*b.x + a.y*b.y + a.z*b.z;
}

inline float vecNorm(const Vec3& v) {
    return sqrtf(vecDot(v, v));
}

inline Vec3 vecNormalize(const Vec3& v) {
    float n = vecNorm(v);
    if (n < 1e-6f) {
        return Vec3{0.0f, 0.0f, 0.0f};
    }
    return v / n;
}

/* =========================================================
   Quaternion
   ========================================================= */
struct Quaternion {
    float w;
    float x;
    float y;
    float z;
};

/* =========================================================
   Globals (defined in vectors.cpp)
   ========================================================= */
extern const Vec3 vecZero;
extern const Quaternion quatIdentity;
extern SystemState systemState;

#endif // VECTORS_H
