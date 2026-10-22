#include "BallPositionEstimator.h"

void BallPositionEstimator::estimate(
    const float grid_strength[MAX_GRID_SIZE][MAX_GRID_SIZE],
    int current_grid_size, int valid_zone_count, float& center_x,
    float& center_y) {
    center_x = -1.0;
    center_y = -1.0;

    if (valid_zone_count > 0) {
        float minV = 999999.0, maxV = -1.0;
        for (int y = 0; y < current_grid_size; y++) {
            for (int x = 0; x < current_grid_size; x++) {
                if (grid_strength[y][x] > maxV) maxV = grid_strength[y][x];
                if (grid_strength[y][x] < minV) minV = grid_strength[y][x];
            }
        }

        float range = maxV - minV;
        if (range < 1e-6) range = 1.0;

        float wx = 0, wy = 0, sumW = 0;

        for (int iy = -1; iy <= current_grid_size; iy++) {
            int my = iy;
            if (my < 0) my = 0;
            if (my >= current_grid_size) my = current_grid_size - 1;

            for (int ix = -1; ix <= current_grid_size; ix++) {
                int mx = ix;
                if (mx < 0) mx = 0;
                if (mx >= current_grid_size) mx = current_grid_size - 1;

                float v = (grid_strength[my][mx] - minV) / range;
                if (v < 0) v = 0.0;

                float w = v * v * v;

                wx += ix * w;
                wy += iy * w;
                sumW += w;
            }
        }

        if (sumW > 1e-6) {
            center_x = wx / sumW;
            center_y = wy / sumW;
        }
    }
}