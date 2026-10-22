#include <Arduino.h>

#include "BalanceController.h"
#include "BallPositionEstimator.h"
#include "IRSensorArray.h"
#include "PlatformKinematics.h"
#include "ServoMotor.h"

// =====================================================================
// 硬體設定
// =====================================================================
const uint8_t I2C_SDA = 22;
const uint8_t I2C_SCL = 21;

// 腳 1 / 腳 2 / 腳 3 接在 PCA9685 的哪個 channel
// （從上往下看：腳 2 在 +X 軸，腳 1 在 120 度，腳 3 在 240 度，逆時針排列）
const uint8_t LEG_CHANNEL[3] = {0, 1, 2};

// =====================================================================
// 機構尺寸（mm）：與參考專案相同（原專案單位為 cm）
// =====================================================================
PlatformKinematics kinematics(71.25, 62.0, 45.0, 40.0);

// 平台工作高度（mm），原專案使用 8.26 cm
const float DEFAULT_HEIGHT = 82.6f;

// =====================================================================
// 伺服校正（每顆 MG996 都要自己量）
//   伺服角 = neutral + gain × (模型角 − 水平時的模型角) + offset
//   neutral：平台在 DEFAULT_HEIGHT 水平時，該伺服的角度
//   gain   ：+1 或 -1，馬達裝反時改成 -1
//   offset ：微調用
//   minDeg / maxDeg：保護機構的角度上下限
// =====================================================================
struct ServoCal {
    float neutral;
    float gain;
    float offset;
    float minDeg;
    float maxDeg;
};

ServoCal servoCal[3] = {
    // neutral 統一 90，各馬達落差用 offset 補：90 + offset = 實測水平角
    {90.0f, 1.0f, 0.0f, 40.0f, 140.0f},   // 腳 1（CH0）：90 + 11.5 = 101.5 度
    {90.0f, 1.0f, -4.5f, 40.0f, 140.0f},  // 腳 2（CH1）：90 - 2.5  = 87.5 度
    {90.0f, 1.0f, -4.0f, 40.0f, 140.0f},  // 腳 3（CH2）：90 + 1    = 91 度
};

// =====================================================================
// 感測器座標 → 平台座標
// 紅外線陣列裝好後，如果 X/Y 軸方向和平台不同，在這裡轉
// =====================================================================
const float SENSOR_ROTATION_DEG = 0.0f;  // 感測器座標相對平台逆時針旋轉幾度
const bool SENSOR_FLIP_X = false;
const bool SENSOR_FLIP_Y = false;

// false：手動輸入球位置，每輸入一次只算一次、轉到位就停
// true ：由紅外線陣列（readBallPosition）持續回傳位置，PID 以 50 Hz 連續控制
const bool USE_IR_SENSOR = true;

// 控制頻率（50 Hz）
const uint32_t CONTROL_PERIOD_MS = 20;

// 狀態列印間隔
const uint32_t PRINT_PERIOD_MS = 250;

// =====================================================================
// 全域物件
// =====================================================================
ServoMotor motors;
BalanceController pid;
IRSensorArray irSensor;
BallPositionEstimator estimator;

enum Mode {
    MODE_BALL,  // 依球位置自動控制（PID）
    MODE_TILT,  // 直接指定傾角
    MODE_RAW    // 直接指定三顆伺服角度
};

Mode mode = MODE_TILT;

float height = DEFAULT_HEIGHT;
float maxTilt = 0.0f;
float levelModel[3];  // 水平時三顆伺服的模型角（校正基準）
float lastServo[3] = {0, 0, 0};

float targetX = 0.0f, targetY = 0.0f;  // 希望球停的位置（mm）
float tiltTheta = 0.0f, tiltPhi = 0.0f;

// 模擬的球位置（之後由紅外線陣列取代）
struct BallReading {
    bool present;
    float x;  // mm，感測器座標
    float y;
};

BallReading manualBall = {false, 0.0f, 0.0f};
BallReading irBall = {false, 0.0f, 0.0f};

// =====================================================================
// 球位置來源
// 紅外線陣列做好後，只要改這個函式：回傳球是否在盤上與球的 x, y（mm）
// =====================================================================
bool readBallPosition(BallReading& ball) {
    ball = USE_IR_SENSOR ? irBall : manualBall;
    return ball.present;
}

// 感測器座標轉成平台座標
void sensorToPlatform(float sx, float sy, float& px, float& py) {
    if (SENSOR_FLIP_X) sx = -sx;
    if (SENSOR_FLIP_Y) sy = -sy;

    float r = radians(SENSOR_ROTATION_DEG);
    px = sx * cos(r) - sy * sin(r);
    py = sx * sin(r) + sy * cos(r);
}

// =====================================================================
// 輸出到伺服
// =====================================================================
float modelToServo(int leg, float modelDeg) {
    const ServoCal& c = servoCal[leg];
    float a = c.neutral + c.gain * (modelDeg - levelModel[leg]) + c.offset;
    return constrain(a, c.minDeg, c.maxDeg);
}

void writeServos(const float servoDeg[3]) {
    for (int i = 0; i < 3; i++) {
        motors.setAngle(LEG_CHANNEL[i], servoDeg[i]);
        lastServo[i] = servoDeg[i];
    }
}

// 指定平台姿態：往 phi 方向那一側往下傾 theta 度
bool applyTilt(float thetaDeg, float phiDeg) {
    float model[3];
    if (!kinematics.solveSpherical(thetaDeg, phiDeg, height, model))
        return false;  // 解不出來就維持上一個姿態

    float servo[3];
    for (int i = 0; i < 3; i++) servo[i] = modelToServo(i, model[i]);

    writeServos(servo);
    return true;
}

// =====================================================================
// 序列埠指令
// =====================================================================
void printHelp() {
    Serial.println();
    Serial.println("========== 指令（單位：mm、度）==========");
    Serial.println(
        "  x y          模擬球在 (x, y)，算出傾角並轉到位（只算一次），例：30 "
        "-20");
    Serial.println("  b x y        同上");
    Serial.println("  n            球離開盤面，回水平");
    Serial.println("  r            （紅外線模式）恢復自動控制");
    Serial.println("  g x y        設定目標點（預設 0 0 = 中心）");
    Serial.println("  t 傾角 方向   直接傾斜：往「方向」那側往下傾，例：t 5 0");
    Serial.println("  l            回水平");
    Serial.println(
        "  a a1 a2 a3   設定三顆伺服角度（會自動加上各自 offset，a 90 90 90 = "
        "水平）");
    Serial.println(
        "  c 腳 neutral [gain] [offset]   設定伺服校正，例：c 2 92 1 0");
    Serial.println("  p kp ki kd   設定 PID");
    Serial.println("  h 高度        設定平台高度");
    Serial.println("  s            顯示目前狀態");
    Serial.println("  ?            顯示這份說明");
    Serial.println("==========================================");
}

void printStatus() {
    Serial.println();
    Serial.printf("模式: %s\n", mode == MODE_BALL   ? "PID 自動"
                                : mode == MODE_TILT ? "指定傾角"
                                                    : "直接角度");
    Serial.printf("高度: %.1f mm  最大傾角: %.1f 度\n", height, maxTilt);
    Serial.printf("PID: kp=%.4f ki=%.4f kd=%.4f\n", pid.kp, pid.ki, pid.kd);
    Serial.printf("模擬球: %s (%.1f, %.1f)  目標: (%.1f, %.1f)\n",
                  manualBall.present ? "在盤上" : "不在", manualBall.x,
                  manualBall.y, targetX, targetY);
    Serial.printf("傾角: theta=%.2f phi=%.1f\n", tiltTheta, tiltPhi);
    for (int i = 0; i < 3; i++) {
        const ServoCal& c = servoCal[i];
        Serial.printf(
            "腳 %d (CH%d): 伺服 %.1f 度 | neutral=%.1f gain=%.0f offset=%.1f "
            "範圍 %.0f~%.0f\n",
            i + 1, LEG_CHANNEL[i], lastServo[i], c.neutral, c.gain, c.offset,
            c.minDeg, c.maxDeg);
    }
}

// 把一行指令切成字詞（空白或逗號分隔）
int splitTokens(char* line, char* tokens[], int maxTokens) {
    int n = 0;
    char* p = strtok(line, " ,\t");
    while (p && n < maxTokens) {
        tokens[n++] = p;
        p = strtok(nullptr, " ,\t");
    }
    return n;
}

bool parseNumber(const char* s, float& out) {
    char* end;
    out = strtof(s, &end);
    return end != s && *end == '\0' && !isnan(out) && !isinf(out);
}

// 從 tokens[from] 開始讀 count 個數字
bool parseNumbers(char* tokens[], int ntok, int from, int count, float out[]) {
    if (ntok - from < count) return false;
    for (int i = 0; i < count; i++)
        if (!parseNumber(tokens[from + i], out[i])) return false;
    return true;
}

// 手動輸入球位置：依目前誤差算一次傾角（只用 kp，沒有積分、微分），轉到位後停止
void setBall(float x, float y) {
    manualBall = {true, x, y};

    if (USE_IR_SENSOR) {
        Serial.println(
            ">> 目前使用紅外線陣列，手動位置不會生效（把 USE_IR_SENSOR 改成 "
            "false）");
        return;
    }

    float px, py;
    sensorToPlatform(x, y, px, py);
    float errX = px - targetX;
    float errY = py - targetY;

    // 平台往球的反方向傾（球那一側抬高），大小 = kp × 誤差距離
    float tx = -pid.kp * errX;
    float ty = -pid.kp * errY;
    float mag = sqrt(tx * tx + ty * ty);

    float theta = min(mag, maxTilt);
    float phi = (mag < 1e-4f) ? 0.0f : degrees(atan2(ty, tx));
    if (phi < 0) phi += 360.0f;

    mode = MODE_TILT;
    if (!applyTilt(theta, phi)) {
        Serial.println(">> 錯誤：此姿態超出機構可達範圍");
        return;
    }
    tiltTheta = theta;
    tiltPhi = phi;

    Serial.printf(
        ">> 球 (%.1f, %.1f) 誤差 (%.1f, %.1f) → 傾角 %.2f 度%s，往 %.1f "
        "度方向往下\n",
        px, py, errX, errY, theta, (mag > maxTilt) ? "（已達上限）" : "", phi);
    Serial.printf("   伺服 1 (CH%d): %.1f 度\n", LEG_CHANNEL[0], lastServo[0]);
    Serial.printf("   伺服 2 (CH%d): %.1f 度\n", LEG_CHANNEL[1], lastServo[1]);
    Serial.printf("   伺服 3 (CH%d): %.1f 度\n", LEG_CHANNEL[2], lastServo[2]);
}

void handleCommand(char* line) {
    char* tok[6];
    int n = splitTokens(line, tok, 6);
    if (n == 0) return;

    float v[4];

    // 只輸入兩個數字 → 球的位置
    if (n == 2 && parseNumbers(tok, n, 0, 2, v)) {
        setBall(v[0], v[1]);
        return;
    }

    char cmd = tolower(tok[0][0]);
    if (strlen(tok[0]) != 1) cmd = 0;

    switch (cmd) {
        case '4':
            irSensor.changeResolution(4);
            break;

        case '8':
            irSensor.changeResolution(8);
            break;

        case 'b':
            if (parseNumbers(tok, n, 1, 2, v))
                setBall(v[0], v[1]);
            else
                Serial.println(">> 格式：b x y");
            break;

        case 'n':
            manualBall.present = false;
            if (!USE_IR_SENSOR) {
                mode = MODE_TILT;
                tiltTheta = tiltPhi = 0;
                applyTilt(0, 0);
                Serial.printf(">> 球已移開，回水平 → 伺服 %.1f / %.1f / %.1f\n",
                              lastServo[0], lastServo[1], lastServo[2]);
            } else
                Serial.println(">> 球已移開，平台慢慢回水平");
            break;

        case 'r':
            if (USE_IR_SENSOR) {
                pid.reset();
                mode = MODE_BALL;
                Serial.println(">> 恢復紅外線陣列自動控制");
            } else
                Serial.println(">> 手動模式不需要 r，直接輸入球的位置即可");
            break;

        case 'g':
            if (parseNumbers(tok, n, 1, 2, v)) {
                targetX = v[0];
                targetY = v[1];
                Serial.printf(">> 目標點 (%.1f, %.1f) mm\n", targetX, targetY);
            } else
                Serial.println(">> 格式：g x y");
            break;

        case 't':
            if (parseNumbers(tok, n, 1, 2, v)) {
                if (v[0] < 0 || v[0] > maxTilt) {
                    Serial.printf(">> 錯誤：傾角需在 0 ~ %.1f 度之間\n",
                                  maxTilt);
                    break;
                }
                mode = MODE_TILT;
                tiltTheta = v[0];
                tiltPhi = v[1];
                if (applyTilt(tiltTheta, tiltPhi))
                    Serial.printf(
                        ">> 傾斜 %.1f 度，往 %.1f 度方向往下 → 伺服 %.1f / "
                        "%.1f / %.1f\n",
                        tiltTheta, tiltPhi, lastServo[0], lastServo[1],
                        lastServo[2]);
                else
                    Serial.println(">> 錯誤：此姿態超出機構可達範圍");
            } else
                Serial.println(">> 格式：t 傾角 方向");
            break;

        case 'l':
            mode = MODE_TILT;
            manualBall.present = false;
            tiltTheta = tiltPhi = 0;
            applyTilt(0, 0);
            Serial.printf(">> 回水平 → 伺服 %.1f / %.1f / %.1f\n", lastServo[0],
                          lastServo[1], lastServo[2]);
            break;

        case 'a':
            if (parseNumbers(tok, n, 1, 3, v)) {
                if (v[0] < 0 || v[0] > 180 || v[1] < 0 || v[1] > 180 ||
                    v[2] < 0 || v[2] > 180) {
                    Serial.println(">> 錯誤：角度需在 0 ~ 180 之間");
                    break;
                }
                mode = MODE_RAW;
                manualBall.present = false;

                // 輸入角度 + 各馬達 offset = 實際送出的角度
                // 例：offset 校正好後，a 90 90 90 就是水平
                float out[3];
                for (int i = 0; i < 3; i++)
                    out[i] = constrain(v[i] + servoCal[i].offset, 0.0f, 180.0f);

                writeServos(out);  // 不經過 minDeg/maxDeg 上下限，校正時請小心
                Serial.printf(
                    ">> 輸入 %.1f / %.1f / %.1f → 加 offset 後實際 %.1f / %.1f "
                    "/ %.1f（PCA9685 計數 %u / %u / %u）\n",
                    v[0], v[1], v[2], out[0], out[1], out[2],
                    motors.ticksOf(LEG_CHANNEL[0]),
                    motors.ticksOf(LEG_CHANNEL[1]),
                    motors.ticksOf(LEG_CHANNEL[2]));
            } else
                Serial.println(">> 格式：a 角度1 角度2 角度3");
            break;

        case 'c': {
            float leg, neutral;
            if (n < 3 || !parseNumber(tok[1], leg) ||
                !parseNumber(tok[2], neutral) || leg < 1 || leg > 3 ||
                leg != (int)leg) {
                Serial.println(">> 格式：c 腳(1~3) neutral [gain] [offset]");
                break;
            }
            ServoCal& c = servoCal[(int)leg - 1];
            c.neutral = neutral;
            float g, o;
            if (n >= 4 && parseNumber(tok[3], g))
                c.gain = (g < 0) ? -1.0f : 1.0f;
            if (n >= 5 && parseNumber(tok[4], o)) c.offset = o;
            Serial.printf(
                ">> 腳 %d：neutral=%.1f gain=%.0f offset=%.1f（記得抄回程式的 "
                "servoCal）\n",
                (int)leg, c.neutral, c.gain, c.offset);
            if (mode == MODE_TILT) applyTilt(tiltTheta, tiltPhi);
            break;
        }

        case 'p':
            if (parseNumbers(tok, n, 1, 3, v)) {
                pid.kp = v[0];
                pid.ki = v[1];
                pid.kd = v[2];
                pid.reset();
                Serial.printf(">> PID kp=%.4f ki=%.4f kd=%.4f\n", pid.kp,
                              pid.ki, pid.kd);
            } else
                Serial.println(">> 格式：p kp ki kd");
            break;

        case 'h':
            if (parseNumbers(tok, n, 1, 1, v)) {
                if (v[0] < kinematics.minHeight() ||
                    v[0] > kinematics.maxHeight()) {
                    Serial.printf(">> 錯誤：高度需在 %.1f ~ %.1f mm 之間\n",
                                  kinematics.minHeight(),
                                  kinematics.maxHeight());
                    break;
                }
                float newMax = kinematics.computeMaxTilt(v[0]);
                if (newMax <= 0) {
                    Serial.println(">> 錯誤：這個高度無法保持水平");
                    break;
                }
                height = v[0];
                maxTilt = newMax;
                if (mode == MODE_TILT)
                    applyTilt(min(tiltTheta, maxTilt), tiltPhi);
                Serial.printf(">> 高度 %.1f mm，最大傾角 %.1f 度\n", height,
                              maxTilt);
            } else
                Serial.println(">> 格式：h 高度");
            break;

        case 's':
            printStatus();
            break;

        case '?':
            printHelp();
            break;

        default:
            Serial.println(">> 看不懂這個指令，輸入 ? 查看說明");
            break;
    }
}

// 非阻塞讀取一整行
void pollSerial() {
    static char buf[64];
    static size_t len = 0;

    while (Serial.available() > 0) {
        char ch = Serial.read();
        if (len == 0 && (ch == '4' || ch == '8') && Serial.peek() == -1) {
            irSensor.changeResolution(ch - '0');
            continue;
        }
        if (ch == '\r') continue;
        if (ch == '\n') {
            buf[len] = '\0';
            if (len > 0) handleCommand(buf);
            len = 0;
        } else if (len < sizeof(buf) - 1) {
            buf[len++] = ch;
        }
    }
}

// =====================================================================
// setup / loop
// =====================================================================
void setup() {
    Serial.begin(115200);
    delay(1000);  // 等待序列埠連線穩定

    // 初始化 ESP32 與 PCA9685
    if (!motors.begin(I2C_SDA, I2C_SCL, 50)) {
        Serial.println("PCA9685 initialization failed!");
        while (1) delay(1000);
    }

    if (USE_IR_SENSOR) {
        irSensor.begin();
    }

    // 以預設高度水平姿態作為校正基準
    if (!kinematics.solveSpherical(0, 0, DEFAULT_HEIGHT, levelModel)) {
        Serial.println("逆向運動學在預設高度無解，請檢查機構尺寸");
        while (1) delay(1000);
    }
    maxTilt = kinematics.computeMaxTilt(height);

    Serial.println("Servo system ready!");
    Serial.printf("水平時模型角: %.2f / %.2f / %.2f，最大傾角 %.1f 度\n",
                  levelModel[0], levelModel[1], levelModel[2], maxTilt);

    // 開機先回水平
    applyTilt(0, 0);

    if (USE_IR_SENSOR) {
        pid.reset();
        mode = MODE_BALL;
    }

    printHelp();
}

void loop() {
    pollSerial();

    if (USE_IR_SENSOR) {
        float grid_strength[MAX_GRID_SIZE][MAX_GRID_SIZE] = {0};
        float grid_dist[MAX_GRID_SIZE][MAX_GRID_SIZE] = {0};
        int valid_zone_count = 0;

        if (irSensor.readData(grid_strength, grid_dist, valid_zone_count)) {
            float center_x = -1.0;
            float center_y = -1.0;

            estimator.estimate(grid_strength, irSensor.current_grid_size,
                               valid_zone_count, center_x, center_y);

            if (center_x != -1.0f && center_y != -1.0f) {
                irBall.present = true;
                irBall.x = center_x;
                irBall.y = center_y;
            } else {
                irBall.present = false;
            }

            Serial.print("DATA,");
            Serial.print(irSensor.current_grid_size);
            Serial.print(",");

            for (int y = 0; y < irSensor.current_grid_size; y++) {
                for (int x = 0; x < irSensor.current_grid_size; x++) {
                    Serial.print(grid_dist[y][x]);
                    Serial.print(",");
                }
            }
            Serial.print(center_x);
            Serial.print(",");
            Serial.println(center_y);
        }
    }

    static uint32_t lastControl = 0;
    static uint32_t lastPrint = 0;
    uint32_t now = millis();

    if (now - lastControl < CONTROL_PERIOD_MS) return;

    float dt = (now - lastControl) / 1000.0f;
    lastControl = now;

    // 手動模式不跑連續 PID（每次輸入在 setBall() 裡算一次就停）
    if (!USE_IR_SENSOR) return;

    // 下了 t / l / a 指令後暫停自動控制，輸入 r 恢復
    if (mode != MODE_BALL) return;

    BallReading ball;
    bool present = readBallPosition(ball);

    float px = 0, py = 0;
    if (present) sensorToPlatform(ball.x, ball.y, px, py);

    pid.update(present, px, py, targetX, targetY, dt, maxTilt, tiltTheta,
               tiltPhi);
    applyTilt(tiltTheta, tiltPhi);

    if (now - lastPrint >= PRINT_PERIOD_MS) {
        lastPrint = now;
        if (present)
            Serial.printf(
                "球 (%6.1f, %6.1f) 誤差 (%6.1f, %6.1f) → 傾角 %5.2f 方向 %5.1f "
                "→ 伺服 %6.1f %6.1f %6.1f\n",
                px, py, px - targetX, py - targetY, tiltTheta, tiltPhi,
                lastServo[0], lastServo[1], lastServo[2]);
        else
            Serial.printf("無球 → 傾角 %5.2f → 伺服 %6.1f %6.1f %6.1f\n",
                          tiltTheta, lastServo[0], lastServo[1], lastServo[2]);
    }
}