#ifndef IR_SENSOR_ARRAY_H
#define IR_SENSOR_ARRAY_H

#include <Arduino.h>
#include <Wire.h>
#include <vl53l7cx_class.h>

#include "Config.h"

// VL53L7CX 多區域 ToF 感測器，支援 4x4 與 8x8 解析度。
class IRSensorArray {
public:
    int current_grid_size = TOF_DEFAULT_GRID_SIZE;

    IRSensorArray();

    // 初始化感測器並開始量測。找不到裝置時會停在 halt()。
    void begin();

    void changeResolution(int new_size);

    // 有新畫面時回傳 true，並填入各區域的訊號強度與距離 (mm)；沒有通過篩選
    // 的區域為 0。
    bool readData(float grid_strength[MAX_GRID_SIZE][MAX_GRID_SIZE],
                  float grid_dist[MAX_GRID_SIZE][MAX_GRID_SIZE],
                  int& valid_zone_count);

private:
    VL53L7CX sensor;
    VL53L7CX_ResultsData results;
    void halt(const char* msg, int status);
};

#endif
