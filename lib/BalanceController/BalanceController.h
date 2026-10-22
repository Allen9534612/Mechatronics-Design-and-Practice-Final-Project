#ifndef BALANCE_CONTROLLER_H
#define BALANCE_CONTROLLER_H

#include <Arduino.h>
#include <math.h>

#include "Config.h"

// 球位置 PID。輸入球位置 (mm)，輸出平台傾角 theta 與傾斜方向 phi (度)。
// phi 的定義與 PlatformKinematics::solveSpherical() 相同：平台往 phi 方向那一
// 側下傾。球在 +X 時要抬高 +X 側，即往 -X 傾，phi = 180。
class BalanceController {
public:
    float kp = PID_KP;  // 度/mm
    float ki = PID_KI;  // 度/(mm*s)
    float kd = PID_KD;  // 度/(mm/s)

    float outputAlpha = PID_OUTPUT_ALPHA;
    float dAlpha = PID_D_ALPHA;
    float integralLimit = PID_INTEGRAL_LIMIT_DEG;

    void reset();

    // ballPresent 為 false 時清除積分，輸出逐漸歸零。
    void update(bool ballPresent, float x, float y, float targetX,
                float targetY, float dt, float maxTilt, float& thetaDeg,
                float& phiDeg);

    float integralX() const { return integX; }
    float integralY() const { return integY; }

private:
    float integX = 0, integY = 0;
    float lastX = 0, lastY = 0;
    float dX = 0, dY = 0;
    float outX = 0, outY = 0;
    bool hasLast = false;

    static void toSpherical(float tx, float ty, float maxTilt, float& thetaDeg,
                            float& phiDeg);
};

#endif
