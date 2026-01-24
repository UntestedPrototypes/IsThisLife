#include "angles.h"
#include <math.h>

/* =========================================================
   Meanings of the key vectors (all are unit vectors)
   =========================================================
   q            : quaternion rotating IMU-frame vectors into WORLD (same as your old usage)
   localPsi     : rod axis expressed in the IMU frame (from calibration stage 1)
   localPhi     : plane normal axis expressed in the IMU frame (from calibration stage 2)

   rodW         : rod axis in WORLD  = R(q) * localPsi
   nW           : plane normal WORLD = R(q) * localPhi

   gW           : gravity direction in WORLD (unit). WORLD_UP = +Z, so gravity is -WORLD_UP.
*/

static const Vec3 WORLD_UP = {0.0f, 0.0f, 1.0f};
static const Vec3 WORLD_X  = {1.0f, 0.0f, 0.0f};
static const float EPS_NORM = 1e-4f;

/* ---------- helpers ---------- */

static float clampf(float x, float lo, float hi) {
    return (x < lo) ? lo : (x > hi) ? hi : x;
}

// Wrap angle to (-pi, pi]
static float wrapToPi(float a) {
    while (a <= -M_PI) a += 2.0f * (float)M_PI;
    while (a >   M_PI) a -= 2.0f * (float)M_PI;
    return a;
}

// Project v onto plane perpendicular to axis (axis must be unit)
static Vec3 projectPerp(const Vec3& v, const Vec3& axisUnit) {
    return v - vecDot(v, axisUnit) * axisUnit;
}

// Safe normalize: if too small, returns fallback (assumed already unit or reasonable)
static Vec3 safeNormalize(const Vec3& v, const Vec3& fallback) {
    float n = vecNorm(v);
    if (n < EPS_NORM) return fallback;
    return (1.0f / n) * v;
}

// Signed angle from "fromUnit" to "toUnit" about "axisUnit" (all unit recommended)
// angle = atan2( axis · (from × to), from · to )
static float signedAngleAboutAxis(const Vec3& fromUnit, const Vec3& toUnit, const Vec3& axisUnit) {
    float y = vecDot(axisUnit, cross(fromUnit, toUnit));
    float x = vecDot(fromUnit, toUnit);
    return atan2f(y, x);
}

// Circular mean of two angles (handles wrap)
static float circularMean2(float a, float b) {
    float s = sinf(a) + sinf(b);
    float c = cosf(a) + cosf(b);
    if (fabsf(s) < 1e-6f && fabsf(c) < 1e-6f) {
        // Opposite angles: mean is ambiguous; pick a.
        return a;
    }
    return atan2f(s, c);
}

/* =========================================================
   Core decomposition using gravity-based signed-angle formulas
   =========================================================
   psi:
     u = rod axis in world
     n = plane normal in world
     a = normalize( g - (g·u)u )   (gravity projected perpendicular to rod)
     psi = atan2( u·(a×n), a·n )

   phi:
     n = plane normal in world
     x = a plane-fixed in-plane reference axis in world
     b = normalize( g - (g·n)n )   (gravity projected into plane)
     phi = atan2( n·(b×x), b·x )

   Choice for plane-fixed in-plane axis:
     We build a reference axis in IMU frame that is guaranteed in-plane:
       xLocal = normalize( localPhi × localPsi )
     then xW = R(q) * xLocal
*/
IMUAngles performDecomposition(const Quaternion& q, const Vec3& localPhi, const Vec3& localPsi) {

    // Gravity direction in WORLD (unit)
    const Vec3 gW = {-WORLD_UP.x, -WORLD_UP.y, -WORLD_UP.z};

    // 1) Rod axis in WORLD (unit)
    Vec3 rodW = quatRotate(q, localPsi);
    rodW = safeNormalize(rodW, WORLD_UP);

    // 2) Plane normal in WORLD (unit)
    Vec3 nW = quatRotate(q, localPhi);
    nW = safeNormalize(nW, WORLD_X);

    /* ---------- PSI: twist about the rod axis ---------- */

    // Reference direction around rod from gravity projection: aW ⟂ rodW
    Vec3 aW_raw = projectPerp(gW, rodW);

    // Fallback if rodW ≈ gravity: use WORLD_X projected perpendicular to rod
    Vec3 aW_fallback = safeNormalize(projectPerp(WORLD_X, rodW), WORLD_X);
    Vec3 aW = safeNormalize(aW_raw, aW_fallback);

    // Signed twist from aW to nW about rodW
    float psi = signedAngleAboutAxis(aW, nW, rodW);
    psi = wrapToPi(psi);

    /* ---------- PHI: spin about the plane normal ---------- */

    // In-plane gravity reference: bW ⟂ nW
    Vec3 bW_raw = projectPerp(gW, nW);

    // Fallback if plane is horizontal (nW ≈ gravity): use rodW projected into plane
    Vec3 bW_fallback = safeNormalize(projectPerp(rodW, nW), WORLD_X);
    Vec3 bW = safeNormalize(bW_raw, bW_fallback);

    // Build a plane-fixed in-plane reference axis in IMU frame
    Vec3 xLocal_raw = cross(localPhi, localPsi);
    Vec3 xLocal_fallback = cross(localPhi, WORLD_X);
    xLocal_fallback = safeNormalize(xLocal_fallback, Vec3{0.0f, 1.0f, 0.0f});
    Vec3 xLocal = safeNormalize(xLocal_raw, xLocal_fallback);

    // Rotate that axis into WORLD
    Vec3 xW = quatRotate(q, xLocal);

    // Make xW strictly in-plane (numerical cleanup)
    xW = projectPerp(xW, nW);
    xW = safeNormalize(xW, bW); // if degenerate, fall back to bW

    // Signed spin from bW to xW about nW
    float phi = signedAngleAboutAxis(bW, xW, nW);
    phi = wrapToPi(phi);

    return {phi, psi, rodW};
}

/* =========================================================
   Aggregate into SystemState
   =========================================================
   orientation.x = PhiL
   orientation.y = mean Psi (circular mean)
   orientation.z = theta heading (as in your original logic)
   position.x    = PhiR (diagnostic)
*/
SystemState calculateSystemState(const IMUAngles& angL, const IMUAngles& angR, uint32_t timestamp_ms) {
    SystemState newState;

    // x = PhiL
    newState.orientation.x = angL.phi;

    // y = Avg Psi (use circular mean to handle wrap at ±pi)
    newState.orientation.y = circularMean2(angL.psi, angR.psi);

    // Heading (theta) using your cross-product logic
    Vec3 avgRod = (angL.rodW + angR.rodW) * 0.5f;
    avgRod = safeNormalize(avgRod, Vec3{0.0f, 1.0f, 0.0f});

    Vec3 forward = cross(WORLD_UP, avgRod);
    newState.orientation.z = (vecNorm(forward) > 0.01f) ? atan2f(forward.y, forward.x) : 0.0f;

    // Save PhiR to position.x for your 2s diagnostic logs
    newState.position.x = angR.phi;

    newState.timestamp_ms = timestamp_ms;
    return newState;
}
