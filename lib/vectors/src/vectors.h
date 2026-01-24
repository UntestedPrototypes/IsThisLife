#ifndef VECTORS_H
#define VECTORS_H

#include <stdint.h>
#include <math.h>
#include <utility/imumaths.h>   // imu::Vector<3>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

/* =========================================================
   Generic 3D vector
   ========================================================= */
struct Vec3 {
    float x;
    float y;
    float z;

    constexpr Vec3() : x(0.0f), y(0.0f), z(0.0f) {}
    constexpr Vec3(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}

    // Interop with Adafruit IMU math
    Vec3(const imu::Vector<3>& v) : x(v.x()), y(v.y()), z(v.z()) {}
};

/* =========================================================
   Quaternion
   ========================================================= */
struct Quaternion {
    float w;
    float x;
    float y;
    float z;
    
    constexpr Quaternion() : w(1.0f), x(0.0f), y(0.0f), z(0.0f) {}
    constexpr Quaternion(float w_, float x_, float y_, float z_) 
        : w(w_), x(x_), y(y_), z(z_) {}
};

/* =========================================================
   System-wide state 
   ========================================================= */
struct SystemState {
    Vec3 position;
    
    // x: Phi (Secondary/Spin), y: Psi (Primary/Drive), z: Alpha (Yaw)
    Vec3 orientation; 
    
    uint32_t timestamp_ms;
};

/* =========================================================
   Math Constants & Utilities
   ========================================================= */

inline float wrapAngle(float angle) {
    while (angle > M_PI)  angle -= 2.0f * M_PI;
    while (angle <= -M_PI) angle += 2.0f * M_PI;
    return angle;
}

/* =========================================================
   Vec3 Operations
   ========================================================= */

inline float vecDot(const Vec3& a, const Vec3& b) {
    return a.x*b.x + a.y*b.y + a.z*b.z;
}

inline float vecNorm(const Vec3& v) {
    return sqrtf(vecDot(v, v));
}

inline Vec3 vecNormalize(const Vec3& v) {
    float n = vecNorm(v);
    if (n < 1e-6f) return Vec3{0.0f, 0.0f, 0.0f};
    return Vec3{v.x / n, v.y / n, v.z / n};
}

inline Vec3 cross(const Vec3& a, const Vec3& b) {
    return Vec3(
        a.y * b.z - a.z * b.y,
        a.z * b.x - a.x * b.z,
        a.x * b.y - a.y * b.x
    );
}

// Operators
inline constexpr Vec3 operator+(const Vec3& a, const Vec3& b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
inline constexpr Vec3 operator-(const Vec3& a, const Vec3& b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
inline constexpr Vec3 operator*(const Vec3& v, float s) { return {v.x * s, v.y * s, v.z * s}; }
inline constexpr Vec3 operator*(float s, const Vec3& v) { return v * s; }
inline constexpr Vec3 operator/(const Vec3& v, float s) { return {v.x / s, v.y / s, v.z / s}; }

inline Vec3& operator+=(Vec3& a, const Vec3& b) { a.x += b.x; a.y += b.y; a.z += b.z; return a; }
inline Vec3& operator-=(Vec3& a, const Vec3& b) { a.x -= b.x; a.y -= b.y; a.z -= b.z; return a; }
inline Vec3& operator*=(Vec3& v, float s) { v.x *= s; v.y *= s; v.z *= s; return v; }
inline Vec3& operator/=(Vec3& v, float s) { v.x /= s; v.y /= s; v.z /= s; return v; }

// IMU Interop
inline Vec3& operator+=(Vec3& a, const imu::Vector<3>& b) { a.x += b.x(); a.y += b.y(); a.z += b.z(); return a; }
inline Vec3 operator-(Vec3 a, const imu::Vector<3>& b) { a -= b; return a; }

/* =========================================================
   Quaternion Operations
   ========================================================= */

// Multiplication: out = a * b
inline Quaternion quatMult(const Quaternion& a, const Quaternion& b) {
    return Quaternion(
        a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
        a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
        a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
        a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w
    );
}

// Conjugate (Inverse for unit quats)
inline Quaternion quatInverse(const Quaternion& q) {
    return Quaternion(q.w, -q.x, -q.y, -q.z);
}

// Create Quaternion from Angle-Axis
inline Quaternion quatFromAngleAxis(float angle, const Vec3& axis) {
    float half = angle * 0.5f;
    float s = sinf(half);
    return Quaternion(cosf(half), s * axis.x, s * axis.y, s * axis.z);
}

// Rotate vector v by quaternion q (body -> world)
inline Vec3 quatRotate(const Quaternion& q, const Vec3& v) {
    // q * [0,v] * q_inv
    // Optimized implementation
    float ix =  q.w * v.x + q.y * v.z - q.z * v.y;
    float iy =  q.w * v.y + q.z * v.x - q.x * v.z;
    float iz =  q.w * v.z + q.x * v.y - q.y * v.x;
    float iw = -q.x * v.x - q.y * v.y - q.z * v.z;

    return Vec3(
        ix * q.w + iw * -q.x + iy * -q.z - iz * -q.y,
        iy * q.w + iw * -q.y + iz * -q.x - ix * -q.z,
        iz * q.w + iw * -q.z + ix * -q.y - iy * -q.x
    );
}

// Extract Twist Angle: Rotation of q around specific axis
inline float quatGetTwist(const Quaternion& q, const Vec3& axis) {
    Vec3 v = {q.x, q.y, q.z};
    float dot = vecDot(v, axis);
    // angle = 2 * atan2(v_parallel, w)
    return wrapAngle(2.0f * atan2f(dot, q.w));
}

/* =========================================================
   Globals
   ========================================================= */
extern const Vec3 vecZero;
extern const Quaternion quatIdentity;
extern SystemState systemState;

#endif // VECTORS_H