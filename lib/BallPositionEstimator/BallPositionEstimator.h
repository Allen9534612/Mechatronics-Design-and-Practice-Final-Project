#ifndef BALL_POSITION_ESTIMATOR_H
#define BALL_POSITION_ESTIMATOR_H

#include "Config.h"

// 球位置估測結果
struct BallEstimate {
    bool valid = false;  // false = 這一幀找不到球
    float gridX = -1.0f;  // 球中心的格點座標（0 ~ N-1，可為小數；原點在格點 (0,0)）
    float gridY = -1.0f;
    float distMm = 0.0f;  // 球附近的加權平均距離（mm）
    float xMm = 0.0f;     // 球位置（mm），感測器視野中心為原點，+x 向右、+y 向下
    float yMm = 0.0f;
};

class BallPositionEstimator {
public:
    // grid_strength：各區域訊號強度（無效區域為 0）
    // grid_dist    ：各區域距離 mm（無效區域為 0）
    BallEstimate estimate(const float grid_strength[MAX_GRID_SIZE][MAX_GRID_SIZE],
                          const float grid_dist[MAX_GRID_SIZE][MAX_GRID_SIZE],
                          int current_grid_size, int valid_zone_count) const;

    // 格點座標 → 視野中心為原點的 mm（距離 distMm 處的水平位置）
    static float gridToMm(float gridCoord, int current_grid_size, float distMm);
};

#endif
