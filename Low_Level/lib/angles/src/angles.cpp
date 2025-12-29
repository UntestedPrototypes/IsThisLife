#include <imu.h>
#include <vectors.h>
#include <math.h>
#include <angles.h>

/* ============================================================
   Quaternion rotation
   ============================================================ */

static Vec3 rotateByQuaternion(const Quaternion& q, const Vec3& v) {
    Vec3 t = {
        2.0f * (q.y * v.z - q.z * v.y),
        2.0f * (q.z * v.x - q.x * v.z),
        2.0f * (q.x * v.y - q.y * v.x)
    };

    return {
        v.x + q.w * t.x + (q.y * t.z - q.z * t.y),
        v.y + q.w * t.y + (q.z * t.x - q.x * t.z),
        v.z + q.w * t.z + (q.x * t.y - q.y * t.x)
    };
}

/* ============================================================
   Core extraction
   ============================================================ */

static void computeAngles(
    const Quaternion& qWI,
    const Vec3& rodAxisIMU,
    const Vec3& planeRefIMU,
    float& phi,
    float& theta,
    float& psi
) {
    Vec3 n = vecNormalize(rotateByQuaternion(qWI, rodAxisIMU));

    float nz = n.z;
    if (nz >  1.0f) nz =  1.0f;
    if (nz < -1.0f) nz = -1.0f;
    theta = acosf(nz);

    phi = atan2f(n.y, n.x);

    Vec3 e_phi = { -sinf(phi), cosf(phi), 0.0f };
    Vec3 e_theta = {
        cosf(theta) * cosf(phi),
        cosf(theta) * sinf(phi),
       -sinf(theta)
    };

    Vec3 sW = rotateByQuaternion(qWI, planeRefIMU);

    Vec3 s_perp = sW - n * vecDot(sW, n);
    s_perp = vecNormalize(s_perp);

    psi = atan2f(
        vecDot(s_perp, e_theta),
        vecDot(s_perp, e_phi)
    );
}

/* ============================================================
   Public API
   ============================================================ */

void bno055_updateFromIMU(
    const IMU& imu,
    uint32_t timestamp_ms
) {
    float phi, theta, psi;

    computeAngles(
        imu.getQuaternion(),
        imu.getRodAxisIMU(),
        imu.getPlaneRefIMU(),
        phi,
        theta,
        psi
    );

    systemState.orientation.x = phi;
    systemState.orientation.y = theta;
    systemState.orientation.z = psi;
    systemState.timestamp_ms  = timestamp_ms;
}
