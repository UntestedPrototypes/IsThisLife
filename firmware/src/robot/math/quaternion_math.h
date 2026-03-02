#ifdef ROLE_ROBOT

#ifndef QUATERNION_MATH_H
#define QUATERNION_MATH_H

/**
 * Combines two quaternions (r = q1 * q2).
 * Used for relative orientation and coordinate frame transformations.
 */
void multiplyQuaternions(float w1, float x1, float y1, float z1,
                         float w2, float x2, float y2, float z2,
                         float* rw, float* rx, float* ry, float* rz);

/**
 * Converts Quaternion components to standard Euler angles (Degrees).
 */
void quaternionToEuler(float qw, float qx, float qy, float qz, 
                       float* roll, float* pitch, float* yaw);

#endif // QUATERNION_MATH_H
#endif // ROLE_ROBOT