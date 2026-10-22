#ifndef PLATFORM_KINEMATICS_H
#define PLATFORM_KINEMATICS_H

#include <Arduino.h>
#include <math.h>

#include "Config.h"

// 三軸平台逆向運動學，移植自 I-M-Robotics-Lab/Ball-Balancing-Robot
// 的 robotKinematics.py（長度單位改為 mm）。
//
// 俯視、Z 軸朝上，三隻腳逆時針排列：腳 2 在 +X 軸（0 度）、腳 1 在 120 度、
// 腳 3 在 240 度。
class PlatformKinematics {
public:
    // 參數預設值取自 Config.h：lp 上盤球關節半徑、l1 連桿長度、l2 伺服擺臂
    // 長度、lb 伺服轉軸半徑。
    PlatformKinematics(double topRadius = PLATFORM_TOP_RADIUS_MM,
                       double topArm = PLATFORM_TOP_ARM_MM,
                       double bottomArm = PLATFORM_BOTTOM_ARM_MM,
                       double baseRadius = PLATFORM_BASE_RADIUS_MM);

    // 平台中心的可用高度範圍。
    double maxHeight() const;
    double minHeight() const;

    // 以法向量 (a, b, c) 與高度 h 求三顆伺服的模型角（度）。模型角 = 90 - 擺臂
    // 與水平外側方向的夾角。姿態超出機構可達範圍時回傳 false。
    bool solveVector(double a, double b, double c, double h,
                     float modelDeg[3]) const;

    // 以球座標指定姿態：thetaDeg 為傾角，phiDeg 為傾斜方向（平台往該方向那一
    // 側下傾）。
    bool solveSpherical(float thetaDeg, float phiDeg, float h,
                        float modelDeg[3]) const;

    // 高度 h 下任何方向都解得出來的最大傾角（度），已扣除 0.5 度餘裕。
    float computeMaxTilt(float h) const;

private:
    double lp, l1, l2, lb;
};

#endif
