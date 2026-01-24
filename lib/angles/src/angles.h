#ifndef ANGLES_H
#define ANGLES_H

#include <Arduino.h>
#include "vectors.h"

/**
 * IMUAngles: The raw math output for a single sensor.
 * Passed from the math engine back to the IMU class.
 */
struct IMUAngles {
    float phi;   // Plane Spin (Twist)
    float psi;   // Rod Tilt (Swing relative to Gravity)
    Vec3  rodW;  // Current Rod vector in World Frame (for Yaw/theta)
};

/* ============================================================
    Math Service Functions (Stateless)
   ============================================================ */

/**
 * Core Decomposition logic.
 * The IMU calls this, passing its orientation and internal axes.
 */
IMUAngles performDecomposition(const Quaternion& q, const Vec3& localPhi, const Vec3& localPsi);

/**
 * Aggregator.
 * Takes the results from two sensors and updates the SystemState.
 */
SystemState calculateSystemState(const IMUAngles& angL, const IMUAngles& angR, uint32_t timestamp_ms);

#endif // ANGLES_H