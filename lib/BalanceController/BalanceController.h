#ifndef BALANCE_CONTROLLER_H
#define BALANCE_CONTROLLER_H

#include <Arduino.h>
#include <math.h>

#include "Config.h"

// 球位置 PID：輸入球位置 (mm，盤面中心為原點)，輸出平台傾斜角 theta 與傾斜方向 phi（度）
// 增益單位：kp = 度/mm、ki = 度/(mm·s)、kd = 度/(mm/s)，預設值見 Config.h
//
// 方向約定與 PlatformKinematics::solveSpherical() 相同：
//   平台在 phi 方向那一側往下傾。
//   球在 +X 時，要讓 +X 那一側抬高 → 往 -X 傾 → phi = 180 度。
class BalanceController {
public:
    float kp = PID_KP;  // 度 / mm
    float ki = PID_KI;  // 度 / (mm·s)
    float kd = PID_KD;  // 度 / (mm/s)

    float outputAlpha = PID_OUTPUT_ALPHA;  // 輸出低通濾波（1 = 不濾波）
    float dAlpha = PID_D_ALPHA;            // 微分項低通濾波（雜訊較大時調小）
    float integralLimit = PID_INTEGRAL_LIMIT_DEG;  // 積分項最多貢獻幾度

    void reset();

    // ballPresent = false：球不在盤上 → 清掉積分，輸出慢慢歸零
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