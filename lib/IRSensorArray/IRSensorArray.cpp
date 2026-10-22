#include "IRSensorArray.h"

IRSensorArray::IRSensorArray() : sensor(&Wire, -1) {}

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
        sensor.vl53l7cx_set_ranging_frequency_hz(60);
        current_grid_size = 4;
    } else if (new_size == 8) {
        sensor.vl53l7cx_set_resolution(VL53L7CX_RESOLUTION_8X8);
        sensor.vl53l7cx_set_ranging_frequency_hz(15);
        current_grid_size = 8;
    }

    sensor.vl53l7cx_start_ranging();
}

void IRSensorArray::begin() {
    Serial.println("初始化 VL53L7CX 韌體中...");

    Wire.begin(SDA_PIN, SCL_PIN);
    Wire.setClock(I2C_CLOCK_SPEED);

    sensor.begin();

    Wire.beginTransmission(SENSOR_I2C_ADDRESS);
    int bus_status = Wire.endTransmission();
    if (bus_status != 0) {
        Serial.printf("在 SDA=GPIO%d / SCL=GPIO%d 找不到 I2C 裝置 0x%02X\n",
                      SDA_PIN, SCL_PIN, SENSOR_I2C_ADDRESS);
        halt("請檢查電源與接線", bus_status);
    }

    Serial.printf("在 SDA=GPIO%d / SCL=GPIO%d 偵測到 I2C 裝置 0x%02X\n",
                  SDA_PIN, SCL_PIN, SENSOR_I2C_ADDRESS);

    int status = sensor.init_sensor();
    if (status != 0) halt("找不到感測器或韌體載入失敗", status);
    Serial.println("成功找到裝置");

    if (sensor.vl53l7cx_set_resolution(VL53L7CX_RESOLUTION_4X4) != 0)
        halt("設定解析度失敗", 0);
    if (sensor.vl53l7cx_set_ranging_frequency_hz(60) != 0)
        halt("設定頻率失敗", 0);

    Serial.println("模式：預設 4x4 解析度，頻率 60Hz");

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

                        if (dist >= FILTER_MIN_DISTANCE &&
                            dist <= FILTER_MAX_DISTANCE &&
                            strength >= FILTER_MIN_STRENGTH &&
                            strength <= FILTER_MAX_STRENGTH &&
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