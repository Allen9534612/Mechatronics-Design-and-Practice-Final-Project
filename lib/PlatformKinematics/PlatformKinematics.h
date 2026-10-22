#ifndef PLATFORM_KINEMATICS_H
#define PLATFORM_KINEMATICS_H

#include <Arduino.h>
#include <math.h>

// 三軸平台逆向運動學（移植自 I-M-Robotics-Lab/Ball-Balancing-Robot 的
// robotKinematics.py）
//
// 座標系（從上往下看，Z 軸朝上）：
//   腳 2（A2/B2）在 +X 軸上（0 度）
//   腳 1（A1/B1）在 120 度
//   腳 3（A3/B3）在 240 度
//   三隻腳依逆時針排列
//
// 長度單位：mm（原專案為 cm，數值已乘以 10）
class PlatformKinematics {
public:
    PlatformKinematics(double topRadius = 71.25,   // lp：上盤球關節半徑
                       double topArm = 62.0,       // l1：連桿（上臂）長度
                       double bottomArm = 45.0,    // l2：伺服擺臂（下臂）長度
                       double baseRadius = 40.0);  // lb：伺服轉軸半徑

    // 平台中心可用的最大、最小高度
    double maxHeight() const;
    double minHeight() const;

    // 以法向量 (a, b, c) 與高度 h 求三顆伺服的「模型角度」（度）
    // 模型角度 = 90 - 擺臂與水平外側方向的夾角，與原專案 controller.py 相同
    bool solveVector(double a, double b, double c, double h,
                     float modelDeg[3]) const;

    // 以球座標指定姿態：
    //   thetaDeg = 平台傾斜角（法向量與 Z 軸夾角）
    //   phiDeg   = 傾斜方向；平台在 phi 方向那一側往「下」傾
    bool solveSpherical(float thetaDeg, float phiDeg, float h,
                        float modelDeg[3]) const;

    // 二分搜尋：在高度 h 下，任何方向都解得出來的最大傾角（度），並保留 0.5
    // 度餘裕
    float computeMaxTilt(float h) const;

private:
    double lp, l1, l2, lb;
};

#endif