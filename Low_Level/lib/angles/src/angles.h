#ifndef ANGLES_H
#define ANGLES_H

#include <Arduino.h>
#include <stdint.h>
#include <vectors.h>

/*
  Update systemState.orientation from an IMU quaternion.

  Geometry meaning:
    systemState.orientation.x = phi    // rod azimuth (world x–y plane)
    systemState.orientation.y = theta  // rod polar angle from +Z
    systemState.orientation.z = psi    // in-plane spin about rod axis
*/

/*
  qWI:
    Fused orientation quaternion (e.g., from BNO055)
    Represents rotation: world ← IMU

  timestamp_ms:
    System time in milliseconds
*/
void angles_updateFromQuaternion(
    const Quaternion& qWI,
    uint32_t timestamp_ms
);

#endif // ANGLES_H
