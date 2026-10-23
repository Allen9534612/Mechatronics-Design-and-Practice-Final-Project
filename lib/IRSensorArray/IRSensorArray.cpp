#include "IRSensorArray.h"

IRSensorArray::IRSensorArray() : sensor(&Wire, TOF_LPN_PIN) {}

void IRSensorArray::halt(const char* msg, int status) {
    Serial.printf("%s (status=%d)\n", msg, status);
    while (1) delay(10);
}

void IRSensorArray::changeResolution(int new_size) {
    if (new_size == current_grid_size) return;

    sensor.vl53l7cx_stop_ranging();
    delay(50);

    if (new_size == 4) {
        sensor.vl53l7cx_set_resolution(VL53L7CX_RESOLUTION_4X4);
        sensor.vl53l7cx_set_ranging_frequency_hz(TOF_FREQ_4X4_HZ);
        current_grid_size = 4;
    } else if (new_size == 8) {
        sensor.vl53l7cx_set_resolution(VL53L7CX_RESOLUTION_8X8);
        sensor.vl53l7cx_set_ranging_frequency_hz(TOF_FREQ_8X8_HZ);
        current_grid_size = 8;
    }

    sensor.vl53l7cx_start_ranging();
}

void IRSensorArray::begin() {
    Serial.println("初始化 VL53L7CX 韌體中...");

    Wire.begin(TOF_I2C_SDA_PIN, TOF_I2C_SCL_PIN);
    Wire.setClock(TOF_I2C_FREQ_HZ);

    sensor.begin();

    Wire.beginTransmission(TOF_I2C_ADDRESS);
    int bus_status = Wire.endTransmission();
    if (bus_status != 0) {
        Serial.printf("在 SDA=GPIO%d / SCL=GPIO%d 找不到 I2C 裝置 0x%02X\n",
                      TOF_I2C_SDA_PIN, TOF_I2C_SCL_PIN, TOF_I2C_ADDRESS);
        halt("請檢查電源與接線", bus_status);
    }

    Serial.printf("在 SDA=GPIO%d / SCL=GPIO%d 偵測到 I2C 裝置 0x%02X\n",
                  TOF_I2C_SDA_PIN, TOF_I2C_SCL_PIN, TOF_I2C_ADDRESS);

    int status = sensor.init_sensor();
    if (status != 0) halt("找不到感測器或韌體載入失敗", status);
    Serial.println("成功找到裝置");

    const bool use8x8 = current_grid_size == 8;
    if (sensor.vl53l7cx_set_resolution(use8x8 ? VL53L7CX_RESOLUTION_8X8
                                              : VL53L7CX_RESOLUTION_4X4) != 0)
        halt("設定解析度失敗", 0);
    if (sensor.vl53l7cx_set_ranging_frequency_hz(use8x8 ? TOF_FREQ_8X8_HZ
                                                        : TOF_FREQ_4X4_HZ) != 0)
        halt("設定頻率失敗", 0);
    if (sensor.vl53l7cx_set_sharpener_percent(TOF_SHARPENER_PERCENT) != 0)
        halt("設定銳化失敗", 0);

    Serial.printf("模式：預設 %dx%d 解析度，頻率 %dHz\n", current_grid_size,
                  current_grid_size,
                  use8x8 ? TOF_FREQ_8X8_HZ : TOF_FREQ_4X4_HZ);

    if (sensor.vl53l7cx_start_ranging() != 0) halt("開始測量失敗", 0);
    Serial.println("開始測量");
}

bool IRSensorArray::readData(float grid_strength[MAX_GRID_SIZE][MAX_GRID_SIZE],
                             float grid_dist[MAX_GRID_SIZE][MAX_GRID_SIZE],
                             int& valid_zone_count) {
    uint8_t ready = 0;
    if (sensor.vl53l7cx_check_data_ready(&ready) == 0 && ready) {
        if (sensor.vl53l7cx_get_ranging_data(&results) == 0) {
            valid_zone_count = 0;

            for (int y = 0; y < current_grid_size; y++) {
                for (int x = 0; x < current_grid_size; x++) {
                    grid_strength[y][x] = 0;
                    grid_dist[y][x] = 0;

                    int zone =
                        y * current_grid_size + (current_grid_size - 1 - x);
                    int targets_found = results.nb_target_detected[zone];

                    int furthest_distance = -1;
                    uint32_t furthest_strength = 0;

                    for (int t = 0; t < targets_found; t++) {
                        int index = (zone * VL53L7CX_NB_TARGET_PER_ZONE) + t;
                        int dist = results.distance_mm[index];
                        uint32_t strength = results.signal_per_spad[index];
                        uint8_t status = results.target_status[index];

                        if (dist >= TOF_FILTER_MIN_DISTANCE &&
                            dist <= TOF_FILTER_MAX_DISTANCE &&
                            strength >= TOF_FILTER_MIN_STRENGTH &&
                            strength <= TOF_FILTER_MAX_STRENGTH &&
                            (status == 5 || status == 9)) {
                            if (dist > furthest_distance) {
                                furthest_distance = dist;
                                furthest_strength = strength;
                            }
                        }
                    }

                    if (furthest_distance > 0) {
                        grid_strength[y][x] = (float)furthest_strength;
                        grid_dist[y][x] = (float)furthest_distance;
                        valid_zone_count++;
                    }
                }
            }
            return true;
        }
    }
    return false;
}