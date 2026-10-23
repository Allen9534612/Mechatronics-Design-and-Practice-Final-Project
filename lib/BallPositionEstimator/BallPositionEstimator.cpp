#include "BallPositionEstimator.h"

#include <math.h>

namespace {

// 小陣列排序後取中位數（count <= 64）。
float median(float* v, int count) {
    for (int i = 1; i < count; i++) {
        float key = v[i];
        int j = i - 1;
        while (j >= 0 && v[j] > key) {
            v[j + 1] = v[j];
            j--;
        }
        v[j + 1] = key;
    }
    return (count % 2) ? v[count / 2]
                       : 0.5f * (v[count / 2 - 1] + v[count / 2]);
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
    const float grid_dist[MAX_GRID_SIZE][MAX_GRID_SIZE], int current_grid_size,
    int valid_zone_count) const {
    BallEstimate out;
    const int n = current_grid_size;
    if (n < 1 || n > MAX_GRID_SIZE) return out;
    if (valid_zone_count < (n == 8 ? 8 : 6)) return out;

    // 8x8 與 4x4 各用自己的一組參數（見 Config.h）。
    const bool hi = n == 8;
    const float spikeJump =
        hi ? BALL_SPIKE_JUMP_LOG10_8X8 : BALL_SPIKE_JUMP_LOG10_4X4;
    const float rise = hi ? BALL_RISE_LOG10_8X8 : BALL_RISE_LOG10_4X4;
    const float minPeak =
        hi ? BALL_MIN_PEAK_LOG10_8X8 : BALL_MIN_PEAK_LOG10_4X4;
    const int minZones = hi ? BALL_MIN_ZONES_8X8 : BALL_MIN_ZONES_4X4;

    // 1. 取 log10 強度。透明壓克力板的鏡面反射會讓正對感測器的那一格特別強，
    //    球（漫反射）則是一整片偏強，所以用強度而不是距離判斷。
    float logS[MAX_GRID_SIZE][MAX_GRID_SIZE];
    bool ok[MAX_GRID_SIZE][MAX_GRID_SIZE];
    for (int y = 0; y < n; y++) {
        for (int x = 0; x < n; x++) {
            ok[y][x] = grid_strength[y][x] > 0.0f;
            logS[y][x] = ok[y][x] ? log10f(grid_strength[y][x]) : 0.0f;
        }
    }

    // 2. 去除單點尖峰：比周圍有效區域的中位數高出 spikeJump 以上
    //    的區域，視為板子的鏡面反射，以鄰居中位數取代。
    float s[MAX_GRID_SIZE][MAX_GRID_SIZE];
    for (int y = 0; y < n; y++) {
        for (int x = 0; x < n; x++) {
            s[y][x] = logS[y][x];
            if (!ok[y][x]) continue;
            float nb[8];
            int cnt = 0;
            for (int dy = -1; dy <= 1; dy++) {
                for (int dx = -1; dx <= 1; dx++) {
                    int yy = y + dy, xx = x + dx;
                    if ((dy == 0 && dx == 0) || yy < 0 || yy >= n || xx < 0 ||
                        xx >= n || !ok[yy][xx])
                        continue;
                    nb[cnt++] = logS[yy][xx];
                }
            }
            if (cnt >= 3) {
                float med = median(nb, cnt);
                if (logS[y][x] - med > spikeJump) s[y][x] = med;
            }
        }
    }

    // 3. 背景強度取最低的 1/8 百分位（球可能佔掉相當比例的區域，尤其是 4x4），
    //    門檻 = 背景 + rise，且不低於 minPeak（空板子的非尖峰區域都低於此值）。
    float vals[MAX_GRID_SIZE * MAX_GRID_SIZE];
    int nv = 0;
    for (int y = 0; y < n; y++)
        for (int x = 0; x < n; x++)
            if (ok[y][x]) vals[nv++] = s[y][x];
    float tmp[MAX_GRID_SIZE * MAX_GRID_SIZE];
    for (int i = 0; i < nv; i++) tmp[i] = vals[i];
    median(tmp, nv);  // 就地排序，借用來取百分位。
    float base = tmp[nv / 8];
    float thr = base + rise;
    if (thr < minPeak) thr = minPeak;

    // 4. 找出超過門檻的最大 4 連通區塊。
    int label[MAX_GRID_SIZE][MAX_GRID_SIZE];
    for (int y = 0; y < n; y++)
        for (int x = 0; x < n; x++) label[y][x] = -1;
    int bestId = -1, bestSize = 0, nextId = 0;
    for (int y0 = 0; y0 < n; y0++) {
        for (int x0 = 0; x0 < n; x0++) {
            if (!ok[y0][x0] || s[y0][x0] <= thr || label[y0][x0] >= 0) continue;
            int stack[MAX_GRID_SIZE * MAX_GRID_SIZE];
            int sp = 0, size = 0;
            stack[sp++] = y0 * n + x0;
            label[y0][x0] = nextId;
            while (sp > 0) {
                int cur = stack[--sp];
                int cy = cur / n, cx = cur % n;
                size++;
                const int dys[4] = {1, -1, 0, 0};
                const int dxs[4] = {0, 0, 1, -1};
                for (int k = 0; k < 4; k++) {
                    int yy = cy + dys[k], xx = cx + dxs[k];
                    if (yy < 0 || yy >= n || xx < 0 || xx >= n) continue;
                    if (!ok[yy][xx] || s[yy][xx] <= thr || label[yy][xx] >= 0)
                        continue;
                    label[yy][xx] = nextId;
                    stack[sp++] = yy * n + xx;
                }
            }
            if (size > bestSize) {
                bestSize = size;
                bestId = nextId;
            }
            nextId++;
        }
    }
    if (bestId < 0 || bestSize < minZones) return out;

    // 5. 以（強度 - 門檻）加權求球中心與平均距離。
    float sw = 0, gx = 0, gy = 0, dsum = 0, dw = 0;
    for (int y = 0; y < n; y++) {
        for (int x = 0; x < n; x++) {
            if (label[y][x] != bestId) continue;
            float w = s[y][x] - thr;
            sw += w;
            gx += x * w;
            gy += y * w;
            if (grid_dist[y][x] > 0) {
                dsum += grid_dist[y][x] * w;
                dw += w;
            }
        }
    }
    if (sw <= 0 || dw <= 0) return out;

    out.gridX = gx / sw;
    out.gridY = gy / sw;
    out.distMm = dsum / dw;
    out.xMm = gridToMm(out.gridX, n, out.distMm);
    out.yMm = gridToMm(out.gridY, n, out.distMm);
    out.valid = true;
    return out;
}
