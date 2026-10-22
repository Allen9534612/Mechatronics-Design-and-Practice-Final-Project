#ifndef BALANCE_CONTROLLER_H
#define BALANCE_CONTROLLER_H

#include <Arduino.h>
#include <math.h>

// 球位置 PID：輸入球位置 (mm)，輸出平台傾斜角 theta 與傾斜方向 phi（度）
//
// 方向約定與 PlatformKinematics::solveSpherical() 相同：
//   平台在 phi 方向那一側往下傾。
//   球在 +X 時，要讓 +X 那一側抬高 → 往 -X 傾 → phi = 180 度。
class BalanceController {
public:
    float kp = 0.12f;  // 度 / mm
    float ki = 0.01f;  // 度 / (mm·s)
    float kd = 0.06f;  // 度 / (mm/s)

    float outputAlpha = 0.65f;   // 輸出低通濾波（1 = 不濾波）
    float dAlpha = 0.5f;         // 微分項低通濾波（紅外線陣列雜訊較大時調小）
    float integralLimit = 3.0f;  // 積分項最多貢獻幾度

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