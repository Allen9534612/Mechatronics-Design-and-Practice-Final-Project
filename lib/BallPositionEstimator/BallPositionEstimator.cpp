#include "BallPositionEstimator.h"

#include <math.h>

namespace {

// 用三個連續格點 v[0..2]（對應索引 i0..i0+2）做 log 拋物線擬合，求次格點峰值位置。
// 高斯形狀時是精確解；峰值在邊緣時視窗往內移，等於用同側三點外推，
// 避免像質心那樣把邊緣的球往內拉，也不在邊界外補假資料。
// 擬合不成立（有 0、不是凸峰）時退回質心。結果限制在 peakIdx 附近 ±1 格。
float subCellPeak(const float v[3], int i0, int peakIdx) {
    if (v[0] > 0 && v[1] > 0 && v[2] > 0) {
        float lp = logf(v[0]), lc = logf(v[1]), ln = logf(v[2]);
        float denom = lp - 2 * lc + ln;
        if (denom < -1e-6f) {
            float pos = (i0 + 1) + 0.5f * (lp - ln) / denom;
            if (pos > peakIdx + 1) pos = (float)peakIdx + 1;
            if (pos < peakIdx - 1) pos = (float)peakIdx - 1;
            return pos;
        }
    }
    float sum = v[0] + v[1] + v[2];
    float wsum = v[0] * i0 + v[1] * (i0 + 1) + v[2] * (i0 + 2);
    return sum > 1e-6f ? wsum / sum : (float)peakIdx;
}

}  // namespace

float BallPositionEstimator::gridToMm(float gridCoord, int current_grid_size,
                                      float distMm) {
    float pitchDeg = TOF_FOV_DEG / current_grid_size;
    float angleDeg = (gridCoord - (current_grid_size - 1) / 2.0f) * pitchDeg;
    return tanf(radians(angleDeg)) * distMm;
}

BallEstimate BallPositionEstimator::estimate(
    const float grid_strength[MAX_GRID_SIZE][MAX_GRID_SIZE],
    const float grid_dist[MAX_GRID_SIZE][MAX_GRID_SIZE],
    int current_grid_size, int valid_zone_count) const {
    BallEstimate out;
    const int n = current_grid_size;
    if (valid_zone_count <= 0 || n < 1 || n > MAX_GRID_SIZE) return out;

    // 找峰值（只看有效區域），同時算整體對比
    float minV = 999999.0f, maxV = -1.0f;
    int px = -1, py = -1;
    for (int y = 0; y < n; y++) {
        for (int x = 0; x < n; x++) {
            float s = grid_strength[y][x];
            if (s < minV) minV = s;
            if (s > maxV) {
                maxV = s;
                px = x;
                py = y;
            }
        }
    }
    if (px < 0 || maxV - minV < 1e-6f) return out;  // 全平坦 = 沒有球

    // 峰值附近的邊際和（x 方向累加三列、y 方向累加三行），減少單點雜訊；
    // 取三個連續格點的視窗，峰值在邊緣時視窗往內移
    auto clampStart = [n](int peak) {
        int i0 = peak - 1;
        if (i0 > n - 3) i0 = n - 3;
        if (i0 < 0) i0 = 0;
        return i0;
    };
    int x0 = clampStart(px), y0 = clampStart(py);
    // 底值（對比的 1%）避免背景格被減到 0 使 log 無法計算、或被雜訊放大
    const float floorV = 0.03f * (maxV - minV);
    float colSum[3] = {floorV, floorV, floorV};
    float rowSum[3] = {floorV, floorV, floorV};
    for (int k = 0; k < 3; k++) {
        for (int dy = -1; dy <= 1; dy++) {
            int yy = py + dy, cx = x0 + k;
            if (yy >= 0 && yy < n && cx < n)
                colSum[k] += grid_strength[yy][cx] - minV;
        }
        for (int dx = -1; dx <= 1; dx++) {
            int xx = px + dx, cy = y0 + k;
            if (xx >= 0 && xx < n && cy < n)
                rowSum[k] += grid_strength[cy][xx] - minV;
        }
    }
    out.gridX = subCellPeak(colSum, x0, px);
    out.gridY = subCellPeak(rowSum, y0, py);

    // 球附近的加權平均距離（權重 = 去背景後的訊號強度）
    float dsum = 0, wsum = 0;
    for (int dy = -1; dy <= 1; dy++) {
        for (int dx = -1; dx <= 1; dx++) {
            int x = px + dx, y = py + dy;
            if (x < 0 || x >= n || y < 0 || y >= n) continue;
            if (grid_dist[y][x] <= 0) continue;
            float w = grid_strength[y][x] - minV;
            dsum += w * grid_dist[y][x];
            wsum += w;
        }
    }
    if (wsum > 1e-6f) {
        out.distMm = dsum / wsum;
    } else {
        // 視窗內沒有有效距離 → 用所有有效區域的平均距離
        float s = 0;
        int c = 0;
        for (int y = 0; y < n; y++)
            for (int x = 0; x < n; x++)
                if (grid_dist[y][x] > 0) { s += grid_dist[y][x]; c++; }
        if (c == 0) return out;
        out.distMm = s / c;
    }

    out.xMm = gridToMm(out.gridX, n, out.distMm);
    out.yMm = gridToMm(out.gridY, n, out.distMm);
    out.valid = true;
    return out;
}
