// 全專案共用設定（腳位、感測器、機構尺寸、伺服校正、PID 預設值）。
// 單位：長度 mm、角度 度、週期 ms。球位置以盤面中心為原點。

#pragma once

#include <Arduino.h>

// 主控制迴圈。USE_IR_SENSOR 為 false 時改由序列埠手動輸入球位置。
static constexpr long SERIAL_BAUD = 115200;
static constexpr uint32_t CONTROL_PERIOD_MS = 20;
static constexpr uint32_t STAT_PERIOD_MS = 100;
static constexpr bool USE_IR_SENSOR = true;

// I2C 匯流排，PCA9685 與 VL53L7CX 共用。
static constexpr uint8_t I2C_SDA_PIN = 22;
static constexpr uint8_t I2C_SCL_PIN = 21;
static constexpr uint32_t I2C_FREQ_HZ = 400000;

// VL53L7CX。TOF_LPN_PIN 沒接線時為 -1；TOF_FOV_DEG 為單邊視角。
static constexpr uint8_t TOF_I2C_ADDRESS = 0x29;
static constexpr int TOF_LPN_PIN = -1;
static constexpr int MAX_GRID_SIZE = 8;
static constexpr int TOF_DEFAULT_GRID_SIZE = 4;
static constexpr uint8_t TOF_FREQ_4X4_HZ = 60;
static constexpr uint8_t TOF_FREQ_8X8_HZ = 15;
static constexpr float TOF_FOV_DEG = 60.0f;

// 區域有效性篩選：距離 (mm) 與 signal_per_spad。
static constexpr int TOF_FILTER_MIN_DISTANCE = 150;
static constexpr int TOF_FILTER_MAX_DISTANCE = 800;
static constexpr int TOF_FILTER_MIN_STRENGTH = 1;
static constexpr int TOF_FILTER_MAX_STRENGTH = 4000;

// 感測器座標轉平台座標：先翻轉，再逆時針旋轉。
static constexpr float SENSOR_ROTATION_DEG = 0.0f;
static constexpr bool SENSOR_FLIP_X = false;
static constexpr bool SENSOR_FLIP_Y = false;

// PCA9685 與伺服脈寬。
static constexpr uint8_t PCA9685_ADDRESS = 0x40;
static constexpr uint16_t SERVO_PWM_FREQ_HZ = 50;
static constexpr uint16_t SERVO_MIN_US = 500;
static constexpr uint16_t SERVO_MAX_US = 2500;

// 腳 1/2/3 在 PCA9685 的 channel。俯視逆時針排列：腳 2 在 +X 軸，腳 1 在
// 120 度，腳 3 在 240 度。
static constexpr uint8_t SERVO_CHANNEL[3] = {0, 1, 2};

// 伺服校正：角度 = neutral + gain * (模型角 - 水平時的模型角) + offset，
// 再限制在 [minDeg, maxDeg]。gain 為 +1 或 -1（馬達裝反時用 -1）。
struct ServoCal {
    float neutral;
    float gain;
    float offset;
    float minDeg;
    float maxDeg;
};

// neutral 統一為 90，各馬達的差異以 offset 補償。
static constexpr ServoCal SERVO_CAL_DEFAULT[3] = {
    {90.0f, 1.0f, 0.0f, 40.0f, 140.0f},
    {90.0f, 1.0f, -4.5f, 40.0f, 140.0f},
    {90.0f, 1.0f, -4.0f, 40.0f, 140.0f},
};

// 平台機構尺寸，符號對應 PlatformKinematics。MAX_TILT_DEG 會再與運動學
// 可達的最大傾角取小。
static constexpr double PLATFORM_TOP_RADIUS_MM = 71.25;  // lp
static constexpr double PLATFORM_TOP_ARM_MM = 62.0;      // l1
static constexpr double PLATFORM_BOTTOM_ARM_MM = 45.0;   // l2
static constexpr double PLATFORM_BASE_RADIUS_MM = 40.0;  // lb
static constexpr float PLATFORM_DEFAULT_HEIGHT_MM = 82.6f;
static constexpr float MAX_TILT_DEG = 15.0f;

// PID：輸入為球位置誤差 (mm)，輸出為平台傾角 (度)。
// kp 單位為度/mm，ki 為度/(mm*s)，kd 為度/(mm/s)。
static constexpr float BALANCE_TARGET_X = 0.0f;
static constexpr float BALANCE_TARGET_Y = 0.0f;
static constexpr float PID_KP = 0.12f;
static constexpr float PID_KI = 0.01f;
static constexpr float PID_KD = 0.06f;
static constexpr float PID_OUTPUT_ALPHA = 0.65f;       // 輸出低通，1 = 不濾波。
static constexpr float PID_D_ALPHA = 0.5f;             // 微分項低通。
static constexpr float PID_INTEGRAL_LIMIT_DEG = 3.0f;  // 積分項貢獻上限。
