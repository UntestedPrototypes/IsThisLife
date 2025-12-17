#include <vectors.h>
#include <math.h>
#include <Arduino.h>
#include <angles.h>


/* ============================================================
   Vector math utilities (using your Vec3)
   ============================================================ */

static inline float vecDot(const Vec3& a, const Vec3& b) {
    return a.x*b.x + a.y*b.y + a.z*b.z;
}

static inline Vec3 vecScale(const Vec3& v, float s) {
    return { s*v.x, s*v.y, s*v.z };
}

static inline Vec3 vecSub(const Vec3& a, const Vec3& b) {
    return { a.x - b.x, a.y - b.y, a.z - b.z };
}

static inline Vec3 vecNormalize(const Vec3& v) {
    float n = sqrtf(v.x*v.x + v.y*v.y + v.z*v.z);
    return { v.x/n, v.y/n, v.z/n };
}

/* ============================================================
   Quaternion rotation (world ← IMU)
   ============================================================ */

static Vec3 rotateByQuaternion(const Quaternion& q, const Vec3& v) {
    // v' = q * v * q_conjugate
    Vec3 t = {
        2.0f * (q.y*v.z - q.z*v.y),
        2.0f * (q.z*v.x - q.x*v.z),
        2.0f * (q.x*v.y - q.y*v.x)
    };

    return {
        v.x + q.w*t.x + (q.y*t.z - q.z*t.y),
        v.y + q.w*t.y + (q.z*t.x - q.x*t.z),
        v.z + q.w*t.z + (q.x*t.y - q.y*t.x)
    };
}

/* ============================================================
   Core extraction: theta and psi
   ============================================================ */

static void computeThetaPsi(
    const Quaternion& qWI,
    const Vec3& rodAxisIMU,
    const Vec3& planeRefIMU,
    float& theta,
    float& psi,
    float& phi
) {
    /* --- Rod axis in world frame --- */
    Vec3 n = rotateByQuaternion(qWI, rodAxisIMU);
    n = vecNormalize(n);

    /* --- Theta (polar angle from +Z) --- */
    float nz = n.z;
    if (nz >  1.0f) nz =  1.0f;
    if (nz < -1.0f) nz = -1.0f;
    theta = acosf(nz);

    /* --- Phi (azimuth in x–y plane) --- */
    phi = atan2f(n.y, n.x);

    /* --- Plane basis vectors --- */
    Vec3 e_phi = {
        -sinf(phi),
         cosf(phi),
         0.0f
    };

    Vec3 e_theta = {
         cosf(theta)*cosf(phi),
         cosf(theta)*sinf(phi),
        -sinf(theta)
    };

    /* --- Rotate in-plane reference direction --- */
    Vec3 sW = rotateByQuaternion(qWI, planeRefIMU);

    /* --- Project into plane perpendicular to rod axis --- */
    Vec3 s_perp = vecSub(sW, vecScale(n, vecDot(sW, n)));
    s_perp = vecNormalize(s_perp);

    /* --- Psi (spin about rod axis) --- */
    psi = atan2f(
        vecDot(s_perp, e_theta),
        vecDot(s_perp, e_phi)
    );
}

/* ============================================================
   Public update function
   ============================================================ */

/*
   Call this once per IMU update.

   Inputs:
     qWI            → fused BNO055 quaternion (world ← IMU)
     timestamp_ms   → system time

   Geometry assumptions:
     - rodAxisIMU   → unit vector along rod axis in IMU frame
     - planeRefIMU  → unit vector fixed in internal plane
*/

void bno055_updateFromQuaternion(
    const Quaternion& qWI,
    uint32_t timestamp_ms
) {
    /* ---- Calibration vectors (EDIT IF NEEDED) ---- */
    static const Vec3 rodAxisIMU  = { 1.0f, 0.0f, 0.0f };
    static const Vec3 planeRefIMU = { 0.0f, 1.0f, 0.0f };

    float theta, psi, phi;

    computeThetaPsi(
        qWI,
        rodAxisIMU,
        planeRefIMU,
        theta,
        psi,
        phi
    );

    /* ---- Store results ----
       orientation.x = phi
       orientation.y = theta
       orientation.z = psi
    */
    systemState.orientation.x = phi;
    systemState.orientation.y = theta;
    systemState.orientation.z = psi;

    systemState.timestamp_ms = timestamp_ms;
}
