#pragma once
#include <Arduino.h>

// 平衡球平台專案共用設定：腳位、感測器、機構尺寸、伺服校正、PID 參數。
// 所有模組（src/main.cpp 與 lib/*）都從這裡取值，不要在其他檔案重複定義。
//
// 單位約定：長度 mm、角度 度、時間 s（週期用 ms）。
// 球位置座標：盤面中心為原點（mm），由 BallPositionEstimator 輸出、PID 輸入。

// ====== 主控制迴圈 ======
static constexpr long SERIAL_BAUD = 115200;
static constexpr uint32_t CONTROL_PERIOD_MS = 20;  // 控制週期（50 Hz）
static constexpr uint32_t PRINT_PERIOD_MS = 250;   // 狀態列印間隔

// false：手動輸入球位置，每輸入一次只算一次、轉到位就停
// true ：由 VL53L7CX 持續回傳位置，PID 以 CONTROL_PERIOD_MS 連續控制
static constexpr bool USE_IR_SENSOR = true;

// ====== I2C 匯流排（PCA9685 與 VL53L7CX 共用）======
static constexpr uint8_t I2C_SDA_PIN = 22;
static constexpr uint8_t I2C_SCL_PIN = 21;
static constexpr uint32_t I2C_FREQ_HZ = 400000;

// ====== VL53L7CX ToF 感測器（多區域測距）======
static constexpr uint8_t TOF_I2C_ADDRESS = 0x29;
static constexpr int TOF_LPN_PIN = -1;  // LPn 沒接就維持 -1
static constexpr int MAX_GRID_SIZE = 8;  // 最大解析度 8x8
static constexpr int TOF_DEFAULT_GRID_SIZE = 4;
static constexpr uint8_t TOF_FREQ_4X4_HZ = 60;
static constexpr uint8_t TOF_FREQ_8X8_HZ = 15;  // 8x8 最高 15 Hz
static constexpr float TOF_FOV_DEG = 60.0f;     // VL53L7CX 單邊視角（方形 60°x60°，對角約 90°）

// 區域有效性篩選（距離 mm、signal_per_spad）
static constexpr int TOF_FILTER_MIN_DISTANCE = 150;
static constexpr int TOF_FILTER_MAX_DISTANCE = 800;
static constexpr int TOF_FILTER_MIN_STRENGTH = 1;
static constexpr int TOF_FILTER_MAX_STRENGTH = 4000;

// ====== 感測器座標 → 平台座標 ======
// 感測器裝好後，如果 X/Y 軸方向和平台不同，在這裡轉
static constexpr float SENSOR_ROTATION_DEG = 0.0f;  // 相對平台逆時針旋轉幾度
static constexpr bool SENSOR_FLIP_X = false;
static constexpr bool SENSOR_FLIP_Y = false;

// ====== 伺服 / PCA9685 ======
static constexpr uint8_t PCA9685_ADDRESS = 0x40;
static constexpr uint16_t SERVO_PWM_FREQ_HZ = 50;
static constexpr uint16_t SERVO_MIN_US = 500;
static constexpr uint16_t SERVO_MAX_US = 2500;

// 腳 1 / 腳 2 / 腳 3 接在 PCA9685 的哪個 channel
// （從上往下看：腳 2 在 +X 軸，腳 1 在 120 度，腳 3 在 240 度，逆時針排列）
static constexpr uint8_t SERVO_CHANNEL[3] = {0, 1, 2};

// 伺服校正（每顆 MG996 都要自己量）
//   伺服角 = neutral + gain × (模型角 − 水平時的模型角) + offset
//   neutral：平台水平時該伺服的角度
//   gain   ：+1 或 -1，馬達裝反時改成 -1
//   offset ：微調用
//   minDeg / maxDeg：保護機構的角度上下限
struct ServoCal {
    float neutral;
    float gain;
    float offset;
    float minDeg;
    float maxDeg;
};

// neutral 統一 90，各馬達落差用 offset 補：90 + offset = 實測水平角
static constexpr ServoCal SERVO_CAL_DEFAULT[3] = {
    {90.0f, 1.0f, 0.0f, 40.0f, 140.0f},   // 腳 1（CH0）：90 + 0   = 90 度
    {90.0f, 1.0f, -4.5f, 40.0f, 140.0f},  // 腳 2（CH1）：90 - 4.5 = 85.5 度
    {90.0f, 1.0f, -4.0f, 40.0f, 140.0f},  // 腳 3（CH2）：90 - 4   = 86 度
};

// ====== 平台機構幾何（mm）======
static constexpr double PLATFORM_TOP_RADIUS_MM = 71.25;  // lp：上盤球關節半徑
static constexpr double PLATFORM_TOP_ARM_MM = 62.0;      // l1：連桿（上臂）長度
static constexpr double PLATFORM_BOTTOM_ARM_MM = 45.0;   // l2：伺服擺臂（下臂）長度
static constexpr double PLATFORM_BASE_RADIUS_MM = 40.0;  // lb：伺服轉軸半徑
static constexpr float PLATFORM_DEFAULT_HEIGHT_MM = 82.6f;  // 平台工作高度
static constexpr float MAX_TILT_DEG = 15.0f;  // 平台允許的最大傾斜角（再與運動學可達範圍取小）

// ====== 平衡控制（PID）======
// 輸入：球位置誤差（mm，盤面中心為原點）
// 輸出：平台傾角（度）。各增益單位：
//   kp：度 / mm      ki：度 / (mm·s)      kd：度 / (mm/s)
static constexpr float BALANCE_TARGET_X = 0.0f;  // 目標球位置（mm）
static constexpr float BALANCE_TARGET_Y = 0.0f;
static constexpr float PID_KP = 0.12f;
static constexpr float PID_KI = 0.01f;
static constexpr float PID_KD = 0.06f;
static constexpr float PID_OUTPUT_ALPHA = 0.65f;  // 輸出低通濾波（1 = 不濾波）
static constexpr float PID_D_ALPHA = 0.5f;        // 微分項低通濾波（雜訊大時調小）
static constexpr float PID_INTEGRAL_LIMIT_DEG = 3.0f;  // 積分項最多貢獻幾度
