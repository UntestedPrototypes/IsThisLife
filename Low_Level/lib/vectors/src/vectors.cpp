#include <vectors.h>
#include <Arduino.h>



const Vec3 vecZero(0.0f, 0.0f, 0.0f);
const Quaternion quatIdentity{1.0f, 0.0f, 0.0f, 0.0f};

SystemState systemState = {
    {0.0f, 0.0f, 0.0f},
    {0.0f, 0.0f, 0.0f},
    0
};
