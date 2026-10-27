// 全專案共用設定（腳位、感測器、機構尺寸、伺服校正、PID 預設值）。
// 單位：長度 mm、角度 度、週期 ms。球位置以盤面中心為原點。

#pragma once

#include <Arduino.h>

// 主控制迴圈。USE_IR_SENSOR 為 false 時改由序列埠手動輸入球位置。
static constexpr long SERIAL_BAUD = 115200;
static constexpr uint32_t CONTROL_PERIOD_MS = 20;
static constexpr uint32_t STAT_PERIOD_MS = 100;
static constexpr bool USE_IR_SENSOR = true;

// 開機後的預設姿態是水平（使用已儲存的原點），不會自動開始平衡；要平衡時
// 輸入 r（或按 GUI 的「自動平衡」）。設為 true 則開機後直接進入自動平衡。
static constexpr bool AUTO_BALANCE_AT_BOOT = false;

// I2C 匯流排：VL53L7CX 使用 Wire，PCA9685 使用獨立的 Wire1，避免伺服的雜訊
// 與傳輸時間影響感測器。
static constexpr uint8_t TOF_I2C_SDA_PIN = 22;
static constexpr uint8_t TOF_I2C_SCL_PIN = 21;
static constexpr uint32_t TOF_I2C_FREQ_HZ = 400000;
static constexpr uint8_t SERVO_I2C_SDA_PIN = 18;
static constexpr uint8_t SERVO_I2C_SCL_PIN = 19;
static constexpr uint32_t SERVO_I2C_FREQ_HZ = 400000;

// VL53L7CX。TOF_LPN_PIN 沒接線時為 -1；TOF_FOV_DEG 為單邊視角。
static constexpr uint8_t TOF_I2C_ADDRESS = 0x29;
static constexpr int TOF_LPN_PIN = -1;
static constexpr int MAX_GRID_SIZE = 8;
static constexpr int TOF_DEFAULT_GRID_SIZE = 4;
static constexpr uint8_t TOF_FREQ_4X4_HZ = 60;
static constexpr uint8_t TOF_FREQ_8X8_HZ = 15;
static constexpr float TOF_FOV_DEG = 60.0f;

// 銳化會把互相重疊的近距離目標標成狀態碼 12 而被篩選丟棄（透過壓克力板量測
// 時，板子與球靠得很近，預設值會讓大部分區域失效），需設為 0。
static constexpr uint8_t TOF_SHARPENER_PERCENT = 0;

// 區域有效性篩選：距離 (mm) 與 signal_per_spad。透過壓克力板量測時，板子的鏡面
// 反射強度可高達數萬、球約數千，所以強度上限放寬，由估測器處理尖峰；距離下限
// 要低於板子與球的實際距離（約 45 mm）。
static constexpr int TOF_FILTER_MIN_DISTANCE = 10;
static constexpr int TOF_FILTER_MAX_DISTANCE = 500;
static constexpr int TOF_FILTER_MIN_STRENGTH = 1;
static constexpr int TOF_FILTER_MAX_STRENGTH = 1000000;

// 球偵測（BallPositionEstimator）：強度取 log10(signal_per_spad)，
// 8x8 與 4x4 各一組，可分開調整。
// SPIKE_JUMP：比鄰居中位數高這麼多的單點，視為板子的鏡面尖峰。
// RISE：區塊門檻 = 背景 + RISE，背景取最低 1/8 的強度。
// MIN_PEAK：區塊門檻的下限，空板子的非尖峰區域都低於它。
// MIN_ZONES：最大區塊至少要幾個區域才算有球。
// 皆以空板子（水平與 ±10° 傾斜）與球在中心的實測驗證。4x4 空板子去除尖峰後
// 最強為 10^2.54，球為 10^2.88 以上，所以 4x4 的 MIN_PEAK 取 2.7。
static constexpr float BALL_SPIKE_JUMP_LOG10_8X8 = 1.0f;
static constexpr float BALL_RISE_LOG10_8X8 = 0.6f;
static constexpr float BALL_MIN_PEAK_LOG10_8X8 = 2.9f;
static constexpr int BALL_MIN_ZONES_8X8 = 3;
static constexpr float BALL_SPIKE_JUMP_LOG10_4X4 = 1.0f;
static constexpr float BALL_RISE_LOG10_4X4 = 0.6f;
static constexpr float BALL_MIN_PEAK_LOG10_4X4 = 2.7f;
static constexpr int BALL_MIN_ZONES_4X4 = 2;

// 感測器座標轉平台座標：先翻轉，再逆時針旋轉。
static constexpr float SENSOR_ROTATION_DEG = 0.0f;
static constexpr bool SENSOR_FLIP_X = true;
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

// neutral 統一為 90，各馬達的差異以 offset 補償。gain 為 -1：伺服角度增加時
// 連桿向下，與運動學的模型角方向相反，所以反轉輸出（不影響水平原點）。
static constexpr ServoCal SERVO_CAL_DEFAULT[3] = {
    {90.0f, -1.0f, 26.0f, 40.0f, 140.0f},
    {90.0f, -1.0f, 22.5f, 40.0f, 140.0f},
    {90.0f, -1.0f, 25.0f, 40.0f, 140.0f},
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
static constexpr float PID_KI = 0.00f;
static constexpr float PID_KD = 0.04f;
static constexpr float PID_OUTPUT_ALPHA = 0.85f;       // 輸出低通，1 = 不濾波。
static constexpr float PID_D_ALPHA = 0.7f;             // 微分項低通。
static constexpr float PID_INTEGRAL_LIMIT_DEG = 2.0f;  // 積分項貢獻上限。
