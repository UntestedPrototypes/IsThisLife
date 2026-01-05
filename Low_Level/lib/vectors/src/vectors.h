#ifndef VECTORS_H
#define VECTORS_H

#include <stdint.h>
#include <math.h>
#include <utility/imumaths.h>   // imu::Vector<3>



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
   System-wide state 
   ========================================================= */

struct SystemState {
    Vec3 position;
    Vec3 orientation;
    uint32_t timestamp_ms;
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
   Cross product
   ========================================================= */
inline Vec3 cross(const Vec3& a, const Vec3& b)
{
    return Vec3(
        a.y * b.z - a.z * b.y,
        a.z * b.x - a.x * b.z,
        a.x * b.y - a.y * b.x
    );
}

/* =========================================================
   Quaternion-vector rotation
   Rotate vector v by quaternion q (body -> world)
   ========================================================= */
inline Vec3 quatRotate(const Quaternion& q, const Vec3& v)
{
    // Using q * [0,v] * q_conj
    Vec3 qv(q.x, q.y, q.z);

    Vec3 t = 2.0f * cross(qv, v);
    Vec3 v_rot =
        v + q.w * t + cross(qv, t);

    return v_rot;
}


/* =========================================================
   Globals (defined in vectors.cpp)
   ========================================================= */
extern const Vec3 vecZero;
extern const Quaternion quatIdentity;
extern SystemState systemState;

#endif // VECTORS_H
