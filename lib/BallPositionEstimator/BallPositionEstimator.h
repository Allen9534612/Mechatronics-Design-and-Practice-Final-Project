#ifndef BALL_POSITION_ESTIMATOR_H
#define BALL_POSITION_ESTIMATOR_H

#include "Config.h"

struct BallEstimate {
    bool valid = false;   // false 表示這一幀找不到球。
    float gridX = -1.0f;  // 球中心的格點座標（可為小數），原點在格點 (0, 0)。
    float gridY = -1.0f;
    float distMm = 0.0f;  // 球附近的加權平均距離。
    float xMm = 0.0f;     // 以視野中心為原點，+x 向右、+y 向下。
    float yMm = 0.0f;
};

class BallPositionEstimator {
public:
    // grid_strength 與 grid_dist 為各區域的訊號強度與距離 (mm)，無效區域為 0。
    BallEstimate estimate(
        const float grid_strength[MAX_GRID_SIZE][MAX_GRID_SIZE],
        const float grid_dist[MAX_GRID_SIZE][MAX_GRID_SIZE],
        int current_grid_size, int valid_zone_count) const;

    // 格點座標轉成距離 distMm 處、以視野中心為原點的水平位置 (mm)。
    static float gridToMm(float gridCoord, int current_grid_size, float distMm);
};

#endif
