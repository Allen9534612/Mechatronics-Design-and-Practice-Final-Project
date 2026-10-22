#include "PlatformKinematics.h"

PlatformKinematics::PlatformKinematics(double topRadius, double topArm,
                                       double bottomArm, double baseRadius)
    : lp(topRadius), l1(topArm), l2(bottomArm), lb(baseRadius) {}

double PlatformKinematics::maxHeight() const {
    return sqrt(sq(l1 + l2) - sq(lp - lb));
}

double PlatformKinematics::minHeight() const {
    if (l1 > l2) return sqrt(sq(l1) - sq(lb + l2 - lp));
    if (l2 > l1) return sqrt(sq(l2 - l1) - sq(lp - lb));
    return 0.0;
}

bool PlatformKinematics::solveVector(double a, double b, double c, double h,
                                     float modelDeg[3]) const {
    const double s3 = sqrt(3.0);

    // ---- 上盤三個球關節 A1、A2、A3 ----
    double d1 = sqrt(4 * c * c + sq(a - s3 * b));
    double d2 = sqrt(c * c + a * a);
    double d3 = sqrt(4 * c * c + sq(a + s3 * b));
    if (d1 < 1e-9 || d2 < 1e-9 || d3 < 1e-9) return false;

    double A1[3] = {-(lp * c) / d1, (s3 * lp * c) / d1,
                    h + ((a - s3 * b) * lp) / d1};
    double A2[3] = {(lp * c) / d2, 0.0, h - (lp * a) / d2};
    double A3[3] = {-(lp * c) / d3, -(s3 * lp * c) / d3,
                    h + ((a + s3 * b) * lp) / d3};

    if (fabs(A1[2]) < 1e-9 || fabs(A2[2]) < 1e-9 || fabs(A3[2]) < 1e-9)
        return false;

    // ---- 中間關節 C1、C2、C3（擺臂末端）----
    double p1 = (-A1[0] + s3 * A1[1] - 2 * lb) / A1[2];
    double q1 =
        (sq(A1[0]) + sq(A1[1]) + sq(A1[2]) + l2 * l2 - l1 * l1 - lb * lb) /
        (2 * A1[2]);
    double r1 = p1 * p1 + 4;
    double sv1 = 2 * p1 * q1 + 4 * lb;
    double t1 = q1 * q1 + lb * lb - l2 * l2;

    double p2 = (lb - A2[0]) / A2[2];
    double q2 =
        (sq(A2[0]) + sq(A2[2]) - lb * lb + l2 * l2 - l1 * l1) / (2 * A2[2]);
    double r2 = p2 * p2 + 1;
    double sv2 = 2 * (p2 * q2 - lb);
    double t2 = q2 * q2 - l2 * l2 + lb * lb;

    double p3 = (-A3[0] - s3 * A3[1] - 2 * lb) / A3[2];
    double q3 =
        (sq(A3[0]) + sq(A3[1]) + sq(A3[2]) + l2 * l2 - l1 * l1 - lb * lb) /
        (2 * A3[2]);
    double r3 = p3 * p3 + 4;
    double sv3 = 2 * p3 * q3 + 4 * lb;
    double t3 = q3 * q3 + lb * lb - l2 * l2;

    double disc1 = sv1 * sv1 - 4 * r1 * t1;
    double disc2 = sv2 * sv2 - 4 * r2 * t2;
    double disc3 = sv3 * sv3 - 4 * r3 * t3;
    if (disc1 < 0 || disc2 < 0 || disc3 < 0)
        return false;  // 該姿態超出機構可達範圍

    double c11 = (-sv1 - sqrt(disc1)) / (2 * r1);
    double c12 = -s3 * c11;
    double z1 = l2 * l2 - 4 * c11 * c11 - 4 * lb * c11 - lb * lb;

    double c21 = (-sv2 + sqrt(disc2)) / (2 * r2);
    double z2 = l2 * l2 - sq(c21 - lb);

    double c31 = (-sv3 - sqrt(disc3)) / (2 * r3);
    double c32 = s3 * c31;
    double z3 = l2 * l2 - 4 * c31 * c31 - 4 * lb * c31 - lb * lb;

    if (z1 < 0 || z2 < 0 || z3 < 0) return false;

    double c13 = sqrt(z1), c23 = sqrt(z2), c33 = sqrt(z3);

    // ---- 擺臂角度 ----
    double th1 = M_PI / 2 - atan2(sqrt(c11 * c11 + c12 * c12) - lb, c13);
    double th2 = atan2(c23, c21 - lb);
    double th3 = M_PI / 2 - atan2(sqrt(c31 * c31 + c32 * c32) - lb, c33);

    float out[3] = {(float)(90.0 - degrees(th1)), (float)(90.0 - degrees(th2)),
                    (float)(90.0 - degrees(th3))};

    for (int i = 0; i < 3; i++) {
        if (isnan(out[i]) || isinf(out[i])) return false;
        modelDeg[i] = out[i];
    }
    return true;
}

bool PlatformKinematics::solveSpherical(float thetaDeg, float phiDeg, float h,
                                        float modelDeg[3]) const {
    double th = radians(thetaDeg);
    double ph = radians(phiDeg);
    double a = sin(th) * cos(ph);
    double b = sin(th) * sin(ph);
    double c = cos(th);
    return solveVector(a, b, c, h, modelDeg);
}

float PlatformKinematics::computeMaxTilt(float h) const {
    float lo = 0.0f, hi = 25.0f;
    float tmp[3];

    if (!solveSpherical(0.0f, 0.0f, h, tmp)) return 0.0f;

    while (hi - lo > 0.05f) {
        float mid = (lo + hi) / 2;
        bool ok = true;
        for (int phi = 0; phi < 360 && ok; phi += 10)
            ok = solveSpherical(mid, (float)phi, h, tmp);

        if (ok)
            lo = mid;
        else
            hi = mid;
    }
    return max(0.0f, lo - 0.5f);
}