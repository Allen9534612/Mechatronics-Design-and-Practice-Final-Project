// 平衡球平台主程式：感測 -> 球位置估測 -> PID -> 逆向運動學 -> 伺服。
// 序列埠輸出 DATA（距離矩陣）與 STAT（狀態）供 script/heatmap.py 顯示，
// 並接受文字指令（輸入 ? 查看）。

#include <Arduino.h>
#include <Preferences.h>

#include "BalanceController.h"
#include "BallPositionEstimator.h"
#include "Config.h"
#include "IRSensorArray.h"
#include "PlatformKinematics.h"
#include "ServoMotor.h"

enum Mode {
    MODE_BALL,  // PID 自動平衡
    MODE_TILT,  // 指定傾角
    MODE_RAW    // 直接指定伺服角度
};

// 球位置，座標單位為 mm。
struct BallReading {
    bool present;
    float x;
    float y;
};

PlatformKinematics kinematics;
ServoMotor motors;
BalanceController pid;
IRSensorArray irSensor;
BallPositionEstimator estimator;

// 執行中可用 c 指令修改，改完需手動抄回 Config.h 的 SERVO_CAL_DEFAULT。
ServoCal servoCal[3] = {SERVO_CAL_DEFAULT[0], SERVO_CAL_DEFAULT[1],
                        SERVO_CAL_DEFAULT[2]};

Mode mode = MODE_TILT;
float height = PLATFORM_DEFAULT_HEIGHT_MM;
float maxTilt = 0.0f;
float levelModel[3];  // 水平姿態下三顆伺服的模型角，作為校正基準。
float lastServo[3] = {0, 0, 0};
float targetX = BALANCE_TARGET_X;  // 目標球位置，盤面中心為原點。
float targetY = BALANCE_TARGET_Y;
float tiltTheta = 0.0f;
float tiltPhi = 0.0f;

BallReading manualBall = {false, 0.0f, 0.0f};
BallReading irBall = {false, 0.0f, 0.0f};
bool servoReady = false;  // PCA9685 是否偵測到。
Preferences prefs;
float lastDistMm = 0.0f;  // 供 STAT 回報。
int lastValidZones = 0;

bool readBallPosition(BallReading& ball) {
    ball = USE_IR_SENSOR ? irBall : manualBall;
    return ball.present;
}

// 依 Config.h 的旋轉/翻轉設定，把感測器座標轉成平台座標。
void sensorToPlatform(float sx, float sy, float& px, float& py) {
    if (SENSOR_FLIP_X) sx = -sx;
    if (SENSOR_FLIP_Y) sy = -sy;

    float r = radians(SENSOR_ROTATION_DEG);
    px = sx * cos(r) - sy * sin(r);
    py = sx * sin(r) + sy * cos(r);
}

float modelToServo(int leg, float modelDeg) {
    const ServoCal& c = servoCal[leg];
    float a = c.neutral + c.gain * (modelDeg - levelModel[leg]) + c.offset;
    return constrain(a, c.minDeg, c.maxDeg);
}

void writeServos(const float servoDeg[3]) {
    for (int i = 0; i < 3; i++) {
        motors.setAngle(SERVO_CHANNEL[i], servoDeg[i]);
        lastServo[i] = servoDeg[i];
    }
}

// 平台往 phiDeg 方向那一側下傾 thetaDeg 度。無解時維持原姿態並回傳 false。
bool applyTilt(float thetaDeg, float phiDeg) {
    float model[3];
    if (!kinematics.solveSpherical(thetaDeg, phiDeg, height, model))
        return false;

    float servo[3];
    for (int i = 0; i < 3; i++) servo[i] = modelToServo(i, model[i]);

    writeServos(servo);
    return true;
}

// 平台水平時的伺服角度 = neutral + offset。offset 可寫入 ESP32 的非揮發記憶體
// （NVS），斷電後保留；沒有儲存值時使用 Config.h 的 SERVO_CAL_DEFAULT。
void loadServoOrigin() {
    if (!prefs.begin("servo", true)) return;
    bool loaded = false;
    for (int i = 0; i < 3; i++) {
        char key[8];
        snprintf(key, sizeof(key), "off%d", i);
        float off = prefs.getFloat(key, NAN);
        if (!isnan(off)) {
            servoCal[i].offset = off;
            loaded = true;
        }
    }
    prefs.end();
    if (loaded) Serial.println("已載入儲存的伺服原點");
}

bool saveServoOrigin() {
    if (!prefs.begin("servo", false)) return false;
    for (int i = 0; i < 3; i++) {
        char key[8];
        snprintf(key, sizeof(key), "off%d", i);
        prefs.putFloat(key, servoCal[i].offset);
    }
    prefs.end();
    return true;
}

void clearServoOrigin() {
    if (!prefs.begin("servo", false)) return;
    prefs.clear();
    prefs.end();
}

void printHelp() {
    Serial.println();
    Serial.println("========== 指令（單位：mm、度）==========");
    Serial.println(
        "  x y          模擬球在 (x, y)，算出傾角並轉到位（只算一次），例：30 "
        "-20");
    Serial.println("  b x y        同上");
    Serial.println("  n            球離開盤面，回水平");
    Serial.println("  r            （紅外線模式）恢復自動控制");
    Serial.println("  g x y        設定目標點（mm，盤面中心為原點）");
    Serial.println("  t 傾角 方向   直接傾斜：往「方向」那側往下傾，例：t 5 0");
    Serial.println("  l            回水平");
    Serial.println(
        "  a a1 a2 a3   設定三顆伺服角度（會加上各自 offset，a 90 90 90 = "
        "水平）");
    Serial.println(
        "  d a1 a2 a3   直接設定三顆伺服的實際角度（不加 offset），校正水平用");
    Serial.println(
        "  o            把目前 d 指令的角度寫入為水平原點（斷電保留）；o r "
        "還原");
    Serial.println(
        "  c 腳 neutral [gain] [offset]   設定伺服校正，例：c 2 92 1 0");
    Serial.println(
        "  p kp ki kd   設定 PID（kp:度/mm ki:度/(mm·s) kd:度/(mm/s)）");
    Serial.println("  h 高度        設定平台高度");
    Serial.println("  4 / 8        切換感測器解析度");
    Serial.println("  s            顯示目前狀態");
    Serial.println("  ?            顯示這份說明");
    Serial.println("==========================================");
}

void printStatus() {
    Serial.println();
    Serial.printf("模式: %s\n", mode == MODE_BALL   ? "PID 自動"
                                : mode == MODE_TILT ? "指定傾角"
                                                    : "直接角度");
    Serial.printf("伺服驅動: %s\n", servoReady ? "已連線" : "未偵測到");
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
            i + 1, SERVO_CHANNEL[i], lastServo[i], c.neutral, c.gain, c.offset,
            c.minDeg, c.maxDeg);
    }
}

// 輸出一行狀態，欄位順序需與 script/heatmap.py 的 STAT_FIELDS 一致：
// STAT,mode,present,x,y,theta,phi,s1,s2,s3,kp,ki,kd,targetX,targetY,height,
//      maxTilt,distMm,validZones,integX,integY
// mode 為 Mode 的數值，x、y 為平台座標（mm）。
void printStat(bool present, float px, float py) {
    Serial.printf(
        "STAT,%d,%d,%.2f,%.2f,%.2f,%.1f,%.1f,%.1f,%.1f,%.4f,%.4f,%.4f,%.1f,"
        "%.1f,%.1f,%.1f,%.1f,%d,%.2f,%.2f\n",
        (int)mode, present ? 1 : 0, px, py, tiltTheta, tiltPhi, lastServo[0],
        lastServo[1], lastServo[2], pid.kp, pid.ki, pid.kd, targetX, targetY,
        height, maxTilt, lastDistMm, lastValidZones, pid.integralX(),
        pid.integralY());
}

// 以空白或逗號切字，回傳字詞數。
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

// 從 tokens[from] 起讀 count 個數字。
bool parseNumbers(char* tokens[], int ntok, int from, int count, float out[]) {
    if (ntok - from < count) return false;
    for (int i = 0; i < count; i++)
        if (!parseNumber(tokens[from + i], out[i])) return false;
    return true;
}

// 手動模式：依目前誤差只用 kp 算一次傾角並轉到位，之後不再控制。
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

    // 球那一側要抬高，所以往球的反方向傾。
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
    Serial.printf("   伺服 1 (CH%d): %.1f 度\n", SERVO_CHANNEL[0],
                  lastServo[0]);
    Serial.printf("   伺服 2 (CH%d): %.1f 度\n", SERVO_CHANNEL[1],
                  lastServo[1]);
    Serial.printf("   伺服 3 (CH%d): %.1f 度\n", SERVO_CHANNEL[2],
                  lastServo[2]);
}

void handleCommand(char* line) {
    char* tok[6];
    int n = splitTokens(line, tok, 6);
    if (n == 0) return;

    float v[4];

    // 只有兩個數字視為球位置。
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

                // 實際角度 = 輸入角度 + 各自 offset。
                float out[3];
                for (int i = 0; i < 3; i++)
                    out[i] = constrain(v[i] + servoCal[i].offset, 0.0f, 180.0f);

                writeServos(out);  // 不受 minDeg/maxDeg 限制，校正時請小心。
                Serial.printf(
                    ">> 輸入 %.1f / %.1f / %.1f → 加 offset 後實際 %.1f / %.1f "
                    "/ %.1f（PCA9685 計數 %u / %u / %u）\n",
                    v[0], v[1], v[2], out[0], out[1], out[2],
                    motors.ticksOf(SERVO_CHANNEL[0]),
                    motors.ticksOf(SERVO_CHANNEL[1]),
                    motors.ticksOf(SERVO_CHANNEL[2]));
            } else
                Serial.println(">> 格式：a 角度1 角度2 角度3");
            break;

        case 'd':  // 直接設定三顆伺服的實際角度（不加 offset），校正水平用。
            if (parseNumbers(tok, n, 1, 3, v)) {
                for (int i = 0; i < 3; i++) {
                    if (v[i] < servoCal[i].minDeg ||
                        v[i] > servoCal[i].maxDeg) {
                        Serial.printf(
                            ">> 錯誤：腳 %d 角度需在 %.0f ~ %.0f 之間\n", i + 1,
                            servoCal[i].minDeg, servoCal[i].maxDeg);
                        return;
                    }
                }
                mode = MODE_RAW;
                manualBall.present = false;
                writeServos(v);
                Serial.printf(
                    ">> 直接角度 %.1f / %.1f / %.1f（PCA9685 計數 %u / %u / "
                    "%u）\n",
                    v[0], v[1], v[2], motors.ticksOf(SERVO_CHANNEL[0]),
                    motors.ticksOf(SERVO_CHANNEL[1]),
                    motors.ticksOf(SERVO_CHANNEL[2]));
            } else
                Serial.println(">> 格式：d 角度1 角度2 角度3");
            break;

        case 'o':  // 寫入平台水平的原點；o r 還原預設。
            if (n >= 2 && tolower(tok[1][0]) == 'r') {
                for (int i = 0; i < 3; i++)
                    servoCal[i].offset = SERVO_CAL_DEFAULT[i].offset;
                clearServoOrigin();
                if (mode == MODE_TILT) applyTilt(tiltTheta, tiltPhi);
                Serial.println(">> 已還原預設原點並清除儲存值");
            } else if (mode != MODE_RAW) {
                Serial.println(">> 請先用 d 指令把平台調到水平，再寫入原點");
            } else if (!servoReady) {
                Serial.println(">> 未偵測到 PCA9685，無法確認平台是否水平");
            } else {
                for (int i = 0; i < 3; i++)
                    servoCal[i].offset = lastServo[i] - servoCal[i].neutral;
                bool saved = saveServoOrigin();
                Serial.printf(
                    ">> 原點已寫入%s：水平角度 %.1f / %.1f / %.1f，offset %.1f "
                    "/ %.1f / %.1f（可抄回 Config.h 的 SERVO_CAL_DEFAULT）\n",
                    saved ? "（斷電保留）" : "（儲存失敗，僅本次有效）",
                    lastServo[0], lastServo[1], lastServo[2],
                    servoCal[0].offset, servoCal[1].offset, servoCal[2].offset);
            }
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
                ">> 腳 %d：neutral=%.1f gain=%.0f offset=%.1f（記得抄回 "
                "Config.h）\n",
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
                float newMax =
                    min(kinematics.computeMaxTilt(v[0]), MAX_TILT_DEG);
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

// 非阻塞讀取序列埠，收到完整一行才處理。單獨一個 '4' 或 '8'（無換行）
// 直接切換解析度。
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

void setup() {
    Serial.begin(SERIAL_BAUD);
    delay(1000);  // 等序列埠連線穩定。

    // 沒接 PCA9685 時只停用伺服輸出，感測、估測與 PID 照常執行。
    servoReady =
        motors.begin(SERVO_I2C_SDA_PIN, SERVO_I2C_SCL_PIN, SERVO_PWM_FREQ_HZ);
    if (!servoReady) {
        Serial.println("PCA9685 未偵測到，伺服輸出停用（仍可讀取感測器資料）");
    }

    if (USE_IR_SENSOR) {
        irSensor.begin();
    }

    if (!kinematics.solveSpherical(0, 0, height, levelModel)) {
        Serial.println("逆向運動學在預設高度無解，請檢查機構尺寸");
        while (1) delay(1000);
    }
    maxTilt = min(kinematics.computeMaxTilt(height), MAX_TILT_DEG);
    loadServoOrigin();

    Serial.println("Servo system ready!");
    Serial.printf("水平時模型角: %.2f / %.2f / %.2f，最大傾角 %.1f 度\n",
                  levelModel[0], levelModel[1], levelModel[2], maxTilt);

    // 開機先回水平；是否自動平衡由 AUTO_BALANCE_AT_BOOT 決定。
    tiltTheta = tiltPhi = 0;
    applyTilt(0, 0);
    pid.reset();
    mode = (USE_IR_SENSOR && AUTO_BALANCE_AT_BOOT) ? MODE_BALL : MODE_TILT;

    printHelp();
}

void loop() {
    pollSerial();

    if (USE_IR_SENSOR) {
        float grid_strength[MAX_GRID_SIZE][MAX_GRID_SIZE] = {0};
        float grid_dist[MAX_GRID_SIZE][MAX_GRID_SIZE] = {0};
        int valid_zone_count = 0;

        if (irSensor.readData(grid_strength, grid_dist, valid_zone_count)) {
            BallEstimate est = estimator.estimate(grid_strength, grid_dist,
                                                  irSensor.current_grid_size,
                                                  valid_zone_count);

            irBall.present = est.valid;
            lastDistMm = est.distMm;
            lastValidZones = valid_zone_count;
            if (est.valid) {
                irBall.x = est.xMm;
                irBall.y = est.yMm;
            }

            // DATA 的球位置用格點座標（heatmap.py 繪圖用），無球為 -1。
            float center_x = est.valid ? est.gridX : -1.0f;
            float center_y = est.valid ? est.gridY : -1.0f;

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
    static uint32_t lastStat = 0;
    uint32_t now = millis();

    if (now - lastControl < CONTROL_PERIOD_MS) return;

    float dt = (now - lastControl) / 1000.0f;
    lastControl = now;

    BallReading ball;
    bool present = readBallPosition(ball);

    float px = 0, py = 0;
    if (present) sensorToPlatform(ball.x, ball.y, px, py);

    if (now - lastStat >= STAT_PERIOD_MS) {
        lastStat = now;
        printStat(present, px, py);
    }

    // 手動模式與 t / l / a 之後都不跑 PID，輸入 r 恢復。
    if (!USE_IR_SENSOR || mode != MODE_BALL) return;

    pid.update(present, px, py, targetX, targetY, dt, maxTilt, tiltTheta,
               tiltPhi);
    applyTilt(tiltTheta, tiltPhi);
}
