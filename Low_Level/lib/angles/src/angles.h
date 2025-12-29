#ifndef ANGLES_H
#define ANGLES_H

#include <stdint.h>

class IMU;   // forward declaration

/*
  Update systemState.orientation from an IMU instance.

  Geometry meaning:
    systemState.orientation.x = phi    // rod azimuth (world x–y plane)
    systemState.orientation.y = theta  // rod polar angle from +Z
    systemState.orientation.z = psi    // in-plane spin about rod axis
*/
void angles_updateFromIMU(
    const IMU& imu,
    uint32_t timestamp_ms
);

#endif // ANGLES_H
