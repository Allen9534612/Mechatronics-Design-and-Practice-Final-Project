#ifndef BALL_POSITION_ESTIMATOR_H
#define BALL_POSITION_ESTIMATOR_H

#define MAX_GRID_SIZE 8

class BallPositionEstimator {
public:
    void estimate(const float grid_strength[MAX_GRID_SIZE][MAX_GRID_SIZE],
                  int current_grid_size, int valid_zone_count, float& center_x,
                  float& center_y);
};

#endif