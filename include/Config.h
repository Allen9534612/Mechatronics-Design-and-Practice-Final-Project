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
// 以下距離上下限是在 PLATFORM_DEFAULT_HEIGHT_MM 這個高度量測校正的；感測器固定
// 在機構上方往下看，平台高度 (height) 數字增加＝平台降低＝感測器到平台的距離
// 變遠，兩者近似 1:1（mm 對 mm），所以實際門檻會依目前高度與這個基準高度的差
// 距平移，見 IRSensorArray::setHeight()。
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

// 伺服角度設定，三層由寬到窄：
//   SERVO_HARD_*  伺服本身的物理極限，只用於脈寬換算的刻度（0~180 對應
//                 SERVO_MIN_US~SERVO_MAX_US），不要當成可用範圍。
//   SERVO_SAFE_*  **安全角度**。ServoMotor::setAngle() 的最終夾限，任何模組
//                 （含 a / d 指令與未來新增的模式）送進來的角度都會被夾在這個
//                 範圍，是保護機構的最後一道防線。
//   SERVO_RANGE_* 運動學的工作範圍，填入 SERVO_CAL_DEFAULT 的 minDeg/maxDeg，
//                 用於判斷姿態是否可達。必須落在安全角度之內。
static constexpr float SERVO_HARD_MIN_DEG = 0.0f;
static constexpr float SERVO_HARD_MAX_DEG = 180.0f;
static constexpr float SERVO_SAFE_MIN_DEG = 20.0f;
static constexpr float SERVO_SAFE_MAX_DEG = 160.0f;
static constexpr float SERVO_RANGE_MIN_DEG = 20.0f;
static constexpr float SERVO_RANGE_MAX_DEG = 160.0f;

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
    {90.0f, -1.0f, 26.0f, SERVO_RANGE_MIN_DEG, SERVO_RANGE_MAX_DEG},
    {90.0f, -1.0f, 22.5f, SERVO_RANGE_MIN_DEG, SERVO_RANGE_MAX_DEG},
    {90.0f, -1.0f, 25.0f, SERVO_RANGE_MIN_DEG, SERVO_RANGE_MAX_DEG},
};

// 平台機構尺寸，符號對應 PlatformKinematics。MAX_TILT_DEG 會再與運動學
// 可達的最大傾角取小。
static constexpr double PLATFORM_TOP_RADIUS_MM = 71.25;  // lp
static constexpr double PLATFORM_TOP_ARM_MM = 62.0;      // l1
static constexpr double PLATFORM_BOTTOM_ARM_MM = 45.0;   // l2
static constexpr double PLATFORM_BASE_RADIUS_MM = 40.0;  // lb
static constexpr float PLATFORM_DEFAULT_HEIGHT_MM = 82.6f;
static constexpr float MAX_TILT_DEG = 15.0f;

// 平台高度上下限：**這裡是唯一來源**，main.cpp 的 minUsableHeight()/
// maxUsableHeight() 直接回傳這兩個值，不再被運動學或伺服掃描覆寫，GUI 也改為
// 從 STAT 讀取而不是自己抄一份。h 越大＝平台越高。
//
// 設定超出機構實際能力時韌體不會阻擋，只會在開機時警告，因為那無法自動判斷
// （Config 的機構尺寸可能與實體不符）。兩個已知的參考點：
//   * 機構奇異點 sqrt(l1^2 - (lb + l2 - lp)^2)（目前尺寸下約 60.5 mm）：擺臂
//     轉到水平，模型角達到 90 度的死點。低於它的高度在另一個解分支上，送出的
//     伺服角度同時也對應上分支的某個高度，平台會停在上分支那一個。
//   * 伺服角度：高度太高時水平姿態就會超出 SERVO_RANGE_*（目前約 96.9 mm）。
// 擺臂末端能不能擺到基座平面以下。false（預設）時模型角上限為 90 度（擺臂水
// 平），對應的平台高度約 sqrt(l1^2 - (lb + l2 - lp)^2)＝60.5 mm；低於它求解器
// 會跳到鏡像解，模型角與伺服角反向，實際高度與指令不符（擺臂水平本身在機構上
// 沒有特殊性，不是力學死點——死點是擺臂與連桿共線）。
//
// 設為 true 時取負根，擺臂可擺到水平線以下，模型角 > 90，平台能降到約 40 mm
// （伺服角仍在安全範圍內，不需要重新安裝舵機）。**開啟前必須先確認擺臂往下擺
// 時不會撞到底板或結構件**，可用 d 指令手動轉到對應角度檢查。
static constexpr bool ALLOW_ARM_BELOW_BASE = true;

// 高度下限。ALLOW_ARM_BELOW_BASE 為 false 時不要低於 62（60.5 是模型角 90 度
// 的交界，留約 2 度餘裕）；設為 true 後可往下放寬。
//
// 上限實際上有三層，由低到高（以目前的伺服安全角度與 NVS offset 計算）：
//   96.9  舊的 40~140 安全角度下的上限（現在放寬到 20~160 已非瓶頸）
//   101.67  computeMaxTilt() 歸零的高度——可達傾角不足
//   KINEMATICS_TILT_MARGIN_DEG
//           的安全餘裕時回傳 0，h 指令會報「這個高度無法保持水平」
//   102.33  機構硬極限 maxHeight()＝連桿與擺臂完全伸直成一直線
//           （sqrt((l1+l2)^2 - (lp-lb)^2)），再高運動學無解
// 下限則受伺服安全角度限制：目前 20~160 對應高度約 20.1 mm（h=20 時伺服角
// 19.9 度，差 0.1 度會被夾限）。
static constexpr float PLATFORM_MIN_HEIGHT_MM = 20.0f;
static constexpr float PLATFORM_MAX_HEIGHT_MM = 100.0f;
static constexpr float HEIGHT_SCAN_STEP_MM = 0.1f;

// 逆向運動學求最大傾角（computeMaxTilt）的二分搜尋範圍、收斂門檻、方向取樣
// 間隔與安全餘裕。
static constexpr float KINEMATICS_TILT_SEARCH_MAX_DEG = 25.0f;
static constexpr float KINEMATICS_TILT_TOLERANCE_DEG = 0.05f;
static constexpr int KINEMATICS_TILT_PHI_STEP_DEG = 10;
static constexpr float KINEMATICS_TILT_MARGIN_DEG = 0.5f;

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

// 開迴路驗證（w、v 指令，不讀球位置）：圓錐傾斜掃掠讓球沿盤緣繞圈，用來在調
// PID 之前確認座標系符號與伺服方向；高度振盪用來確認伺服是否快得足以把球彈
// 起來。
//
// 週期下限同時受伺服速度與控制週期限制：每個週期只有
// 週期/CONTROL_PERIOD_MS 步，掃掠 0.5 秒是 25 步/圈（每步 14 度），再短會變成
// 一格一格跳。振盪是單軸來回，步數可以少一些，所以下限分開設。
static constexpr float SWEEP_TILT_DEG = 5.0f;
static constexpr float SWEEP_PERIOD_S = 2.0f;
static constexpr float SWEEP_MIN_PERIOD_S = 0.5f;
static constexpr float HOP_AMPLITUDE_MM = 5.0f;
static constexpr float HOP_PERIOD_S = 0.5f;
static constexpr float HOP_MIN_PERIOD_S = 0.2f;

// 下 w / v 指令時檢查「整段軌跡是否都在伺服角度範圍內」的取樣間隔，以及被拒絕
// 時回報「目前可用到多少」所用的掃描步進與上限（見 main.cpp 的
// poseWithinServoRange / usableSweepTilt / usableHopAmplitude）。
static constexpr float SWEEP_PHI_CHECK_STEP_DEG = 1.0f;
static constexpr float RANGE_SCAN_TILT_STEP_DEG = 0.5f;
static constexpr float RANGE_SCAN_PHI_STEP_DEG = 5.0f;
static constexpr float RANGE_SCAN_AMP_STEP_MM = 0.5f;
static constexpr float RANGE_SCAN_AMP_MAX_MM = 40.0f;
