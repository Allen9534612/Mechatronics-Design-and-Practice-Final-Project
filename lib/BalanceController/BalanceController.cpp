#include "BalanceController.h"

void BalanceController::reset() {
    integX = integY = 0;
    lastX = lastY = 0;
    dX = dY = 0;
    outX = outY = 0;
    hasLast = false;
}

void BalanceController::update(bool ballPresent, float x, float y,
                               float targetX, float targetY, float dt,
                               float maxTilt, float& thetaDeg, float& phiDeg) {
    dt = constrain(dt, 0.005f, 0.1f);

    if (!ballPresent) {
        integX = integY = 0;
        hasLast = false;
        outX *= 0.9f;
        outY *= 0.9f;
        toSpherical(outX, outY, maxTilt, thetaDeg, phiDeg);
        return;
    }

    float errX = x - targetX;
    float errY = y - targetY;

    // 微分取「量測值」的變化，避免改目標點時輸出暴衝
    if (hasLast) {
        float rawDX = (x - lastX) / dt;
        float rawDY = (y - lastY) / dt;
        dX = dAlpha * rawDX + (1 - dAlpha) * dX;
        dY = dAlpha * rawDY + (1 - dAlpha) * dY;
    } else {
        dX = dY = 0;
    }
    lastX = x;
    lastY = y;
    hasLast = true;

    // 積分（輸出飽和時停止累積，避免 windup）
    bool saturated = sqrt(outX * outX + outY * outY) >= maxTilt;
    if (!saturated && ki > 0) {
        integX += errX * dt;
        integY += errY * dt;
        float lim = integralLimit / ki;
        integX = constrain(integX, -lim, lim);
        integY = constrain(integY, -lim, lim);
    }

    // u = 球偏離方向的修正量；平台要往反方向傾，所以取負號
    float uX = kp * errX + ki * integX + kd * dX;
    float uY = kp * errY + ki * integY + kd * dY;

    outX = outputAlpha * (-uX) + (1 - outputAlpha) * outX;
    outY = outputAlpha * (-uY) + (1 - outputAlpha) * outY;

    toSpherical(outX, outY, maxTilt, thetaDeg, phiDeg);
}

void BalanceController::toSpherical(float tx, float ty, float maxTilt,
                                    float& thetaDeg, float& phiDeg) {
    float mag = sqrt(tx * tx + ty * ty);
    thetaDeg = min(mag, maxTilt);
    if (mag < 1e-4f) {
        phiDeg = 0;
        return;
    }
    phiDeg = degrees(atan2(ty, tx));
    if (phiDeg < 0) phiDeg += 360.0f;
}