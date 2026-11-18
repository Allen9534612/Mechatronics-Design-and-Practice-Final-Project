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

// 新增模式只能附加在最後面：script/heatmap.py 的 MODE_NAMES 是位置對應，且
// 硬寫死 mode==0（自動平衡按鈕）與 mode==2（寫入原點的保護）。
enum Mode {
    MODE_BALL,   // PID 自動平衡
    MODE_TILT,   // 指定傾角
    MODE_RAW,    // 直接指定伺服角度
    MODE_SWEEP,  // 開迴路圓錐掃掠（不讀球位置）
    MODE_HOP,    // 開迴路高度振盪（不讀球位置）
    MODE_ORBIT,  // 閉迴路球繞圈：PID + 會繞著目標點轉動的目標點
    MODE_BOUNCE  // 閉迴路彈跳：垂直正弦（自己鎖相）+ 水平 PID
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

// 開迴路掃掠/振盪（w、v 指令）的參數與起始時刻。
float sweepTheta = SWEEP_TILT_DEG;
float sweepPeriodS = SWEEP_PERIOD_S;
float hopAmplitude = HOP_AMPLITUDE_MM;
float hopPeriodS = HOP_PERIOD_S;
uint32_t sweepStartMs = 0;

// 閉迴路球繞圈（q 指令）的參數與起始時刻；圓心就是 targetX/targetY，可用
// g 指令隨時移動（即使繞圈中也可以）。
float orbitRadius = ORBIT_RADIUS_MM;
float orbitPeriodS = ORBIT_PERIOD_S;
uint32_t orbitStartMs = 0;

// 閉迴路彈跳。彈跳高度由球「偵測到→消失→再偵測到」的間隔推估：球飛高後角
// 尺寸變小、訊號變弱會偵測不到，所以不追蹤頂點，改量飛行時間（h = g*t^2/8）。
// bounceTargetMm 為 0 表示固定振幅、不做垂直調整。
float bounceAmp = BOUNCE_AMPLITUDE_MM;
float bouncePeriodS = BOUNCE_PERIOD_S;
float bounceTargetMm = 0.0f;
uint32_t bounceLostMs = 0;    // 球最後一次消失的時刻
float bounceFlightS = 0.0f;   // 最近一次量到的飛行時間
float bounceHeightMm = 0.0f;  // 由飛行時間推估的彈跳高度
bool bounceWasPresent = false;
uint32_t bounceFoundMs = 0;  // 球這一輪開始被偵測到的時刻
float bouncePhase = -1.0f;   // 最近一次量到的撞擊相位（0~1，-1 為尚未量到）
// 相位用累加器而不是 (now - start)/T：鎖相修正的是「頻率」，累加器才能讓高度
// 指令保持連續。直接平移時間原點會讓 sin 跳變數 mm，正好在球要落下時擾動平台。
float bouncePhaseAcc = 0.0f;    // 0~1
float bouncePhaseFound = 0.0f;  // 球被偵測到當下的相位
float bounceRateCorr = 0.0f;    // 相對頻率偏移，每次撞擊重設
// bounceMaxTilt() 要掃描傾角 x 方向 x 三個高度，振幅 14 時約 2000 次逆運動學
// （ESP32 上數 ms），絕對不能放在控制迴圈裡每週期算。快取起來，只有振幅真的
// 改變時才重算。
float bounceTiltCap = 0.0f;
float bounceCapAmp = -1.0f;  // 快取對應的振幅
// 上一次真的餵給 PID 的時刻。飛行中偵測不到球，PID 不更新，所以再次更新時
// 必須傳「真正經過的時間」——沿用控制週期會讓微分被放大成飛行時間/20ms 倍。
uint32_t bouncePidMs = 0;

BallReading manualBall = {false, 0.0f, 0.0f};
BallReading irBall = {false, 0.0f, 0.0f};
bool servoReady = false;  // PCA9685 是否偵測到。
Preferences prefs;
float lastDistMm = 0.0f;  // 供 STAT 回報。
// 控制週期的實測值。超過 CONTROL_PERIOD_MS 代表迴圈裡有東西太慢（例如把逆
// 運動學掃描放進迴圈），正弦類模式的取樣會變粗、時序會跑掉。
uint32_t dataPeriodMs = DATA_PERIOD_MS;  // m 指令可調整
uint32_t loopMaxMs = 0;
// 感測器讀取與 DATA 輸出各自的耗時，用來分辨控制週期被誰拖慢。
float sensorAvgMs = 0.0f;
uint32_t sensorMaxMs = 0;
float dataAvgMs = 0.0f;
float loopAvgMs = 0.0f;
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

// 未夾限的伺服角度。
float rawServoAngle(int leg, float modelDeg) {
    const ServoCal& c = servoCal[leg];
    return c.neutral + c.gain * (modelDeg - levelModel[leg]) + c.offset;
}

float modelToServo(int leg, float modelDeg) {
    const ServoCal& c = servoCal[leg];
    return constrain(rawServoAngle(leg, modelDeg), c.minDeg, c.maxDeg);
}

// 這個姿態的三顆伺服是否都落在 [minDeg, maxDeg] 內。運動學可達的範圍比伺服
// 角度允許的範圍大（例如預設高度下 IK 可到 16.3 度，但伺服只到 12 度），超出
// 時 modelToServo() 會夾限、實際姿態與指令不符。開迴路模式會連續停在極限附近，
// 所以下指令時先檢查整段軌跡。
// 低於擺臂水平的交界高度時要改取負根，解才會連續延伸（見 Config.h 的
// ALLOW_ARM_BELOW_BASE）。交界高度就是 kinematics.minHeight()。
bool armBelowBaseFor(float h) {
    return ALLOW_ARM_BELOW_BASE && h < (float)kinematics.minHeight();
}

bool poseWithinServoRange(float thetaDeg, float phiDeg, float h) {
    float model[3];
    if (!kinematics.solveSpherical(thetaDeg, phiDeg, h, model,
                                   armBelowBaseFor(h)))
        return false;
    for (int i = 0; i < 3; i++) {
        float a = rawServoAngle(i, model[i]);
        if (a < servoCal[i].minDeg || a > servoCal[i].maxDeg) return false;
    }
    return true;
}

// 平台高度的容許範圍：真正的限制是「水平姿態下三顆伺服都還在角度範圍內」，
// 不是 kinematics.minHeight()/maxHeight()。那兩個是解析式的保守估計
// （minHeight 算的是擺臂完全水平朝外那一刻，實際上求解器在更低的高度仍有解），
// 用它們當門檻會把可用範圍砍掉一大截。這裡改成從預設高度往兩邊掃描，找出水平
// 姿態不會被 modelToServo() 夾限的範圍，再與 Config.h 的設定取交集
// （PLATFORM_MIN/MAX_HEIGHT_MM 當硬性邊界，避免掃到機構會干涉的姿態）。
// 平台高度的可用範圍直接來自 Config.h，不再被運動學或伺服掃描覆寫
// （Config.h 的 PLATFORM_MIN/MAX_HEIGHT_MM 是唯一來源）。
float minUsableHeight() { return PLATFORM_MIN_HEIGHT_MM; }

float maxUsableHeight() { return PLATFORM_MAX_HEIGHT_MM; }

// 開機時檢查設定的高度範圍是否真的做得到，做不到只警告、不阻擋——無法自動
// 判斷 Config 的機構尺寸是否與實體相符，所以由使用者決定。
void warnHeightRange() {
    float crossing = (float)kinematics.minHeight();
    if (PLATFORM_MIN_HEIGHT_MM < crossing && !ALLOW_ARM_BELOW_BASE)
        Serial.printf(
            "警告：高度下限 %.1f mm 低於擺臂水平的交界 %.1f mm，但 "
            "ALLOW_ARM_BELOW_BASE 為 false。低於交界時求解器會跳到鏡像解，伺服"
            "角反向、實際高度與指令不符。請把下限提高到 %.1f 以上，或確認擺臂"
            "往下擺不會撞到底板後開啟 ALLOW_ARM_BELOW_BASE。\n",
            PLATFORM_MIN_HEIGHT_MM, crossing, crossing + 1.5f);
    if (ALLOW_ARM_BELOW_BASE)
        Serial.printf(
            "注意：ALLOW_ARM_BELOW_BASE 已開啟，低於 %.1f mm 時擺臂會擺到基座"
            "平面以下，請確認機構不會干涉。\n",
            crossing);

    for (int i = 0; i < 2; i++) {
        float h = i ? PLATFORM_MAX_HEIGHT_MM : PLATFORM_MIN_HEIGHT_MM;
        if (!poseWithinServoRange(0.0f, 0.0f, h))
            Serial.printf(
                "警告：高度%s %.1f mm 的水平姿態超出伺服角度範圍 %.0f~%.0f "
                "度，實際會被夾限。\n",
                i ? "上限" : "下限", h, SERVO_RANGE_MIN_DEG,
                SERVO_RANGE_MAX_DEG);
    }
}

// 目前高度下，全方向都不會夾限的最大傾角（0 表示連最小傾角都不行）。
float usableSweepTilt() {
    float usable = 0.0f;
    for (float th = RANGE_SCAN_TILT_STEP_DEG; th <= maxTilt;
         th += RANGE_SCAN_TILT_STEP_DEG) {
        bool all = true;
        for (float phi = 0.0f; phi < 360.0f; phi += RANGE_SCAN_PHI_STEP_DEG)
            if (!poseWithinServoRange(th, phi, height)) {
                all = false;
                break;
            }
        if (!all) break;
        usable = th;
    }
    return usable;
}

// 振幅 A 的彈跳中，全方向且高度上下極限都不夾限的最大傾角。彈跳時 PID 的傾角
// 會疊在振盪的高度上，只用 maxTilt 會讓極限附近被靜默夾限。
float bounceMaxTilt(float amp) {
    float best = 0.0f;
    for (float t = RANGE_SCAN_TILT_STEP_DEG; t <= maxTilt;
         t += RANGE_SCAN_TILT_STEP_DEG) {
        bool ok = true;
        for (float phi = 0.0f; phi < 360.0f && ok;
             phi += RANGE_SCAN_PHI_STEP_DEG) {
            if (!poseWithinServoRange(t, phi, height - amp) ||
                !poseWithinServoRange(t, phi, height) ||
                !poseWithinServoRange(t, phi, height + amp))
                ok = false;
        }
        if (!ok) break;
        best = t;
    }
    return best;
}

// 振幅變動超過半個掃描步進才重算傾角上限（見 bounceTiltCap 的註解）。
void updateBounceTiltCap(float amp) {
    if (fabsf(amp - bounceCapAmp) < 0.5f) return;
    bounceCapAmp = amp;
    bounceTiltCap = bounceMaxTilt(amp);
}

// 以目前高度為基準，上下都不會夾限的最大振幅。
float usableHopAmplitude() {
    float usable = 0.0f;
    for (float a = RANGE_SCAN_AMP_STEP_MM; a <= RANGE_SCAN_AMP_MAX_MM;
         a += RANGE_SCAN_AMP_STEP_MM) {
        if (!poseWithinServoRange(0.0f, 0.0f, height - a) ||
            !poseWithinServoRange(0.0f, 0.0f, height + a))
            break;
        usable = a;
    }
    return usable;
}

void writeServos(const float servoDeg[3]) {
    for (int i = 0; i < 3; i++) {
        // ServoMotor::setAngle() 內部會夾到
        // SERVO_SAFE_*，這裡同樣夾一次再記錄， 否則 lastServo（STAT 與 GUI
        // 的來源）會顯示實際上沒送出的角度。
        float a =
            constrain(servoDeg[i], SERVO_SAFE_MIN_DEG, SERVO_SAFE_MAX_DEG);
        motors.setAngle(SERVO_CHANNEL[i], a);
        lastServo[i] = a;
    }
}

// 平台往 phiDeg 方向那一側下傾 thetaDeg 度，h 為平台中心高度（預設為目前設定
// 的 height）。無解時維持原姿態並回傳 false。
bool applyTilt(float thetaDeg, float phiDeg, float h = height) {
    float model[3];
    if (!kinematics.solveSpherical(thetaDeg, phiDeg, h, model,
                                   armBelowBaseFor(h)))
        return false;

    float servo[3];
    for (int i = 0; i < 3; i++) servo[i] = modelToServo(i, model[i]);

    writeServos(servo);

    // ToF 的距離篩選窗要跟著平台實際高度走（見 IRSensorArray::setHeight()）。
    // MODE_HOP 的高度每個控制週期都在變，而 h 指令只在設定時呼叫一次，所以放在
    // 這裡——它是所有會改變平台高度的路徑唯一都會經過的地方，離開振盪模式時
    // （l / t / PID）也會自動還原成基準高度。
    if (USE_IR_SENSOR) irSensor.setHeight(h);
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

const char* modeName() {
    switch (mode) {
        case MODE_BALL:
            return "PID 自動";
        case MODE_TILT:
            return "指定傾角";
        case MODE_RAW:
            return "直接角度";
        case MODE_SWEEP:
            return "圓錐掃掠";
        case MODE_HOP:
            return "高度振盪";
        case MODE_BOUNCE:
            return "閉迴路彈跳";
        case MODE_ORBIT:
            return "球繞圈";
    }
    return "?";
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
        "  j 振幅 週期 [目標]   閉迴路彈跳：垂直正弦＋水平 PID，例：j 14 0.3");
    Serial.println(
        "  w 傾角 週期   開迴路圓錐掃掠（不讀球位置），傾斜方向繞一圈，例：w 5 "
        "2");
    Serial.println(
        "  v 振幅 週期   開迴路高度振盪（不讀球位置），平台上下動，例：v 5 "
        "0.5");
    Serial.println(
        "  q 半徑 週期   閉迴路球繞圈：PID 的目標點繞目前目標點轉一圈，例：q "
        "20 3");
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
    Serial.println(
        "  f 輸出α D的α 積分上限   設定 PID 濾波與積分上限（α "
        "0~1，上限單位度）");
    Serial.println("  h 高度        設定平台高度");
    Serial.println("  4 / 8        切換感測器解析度");
    Serial.println(
        "  m 毫秒        DATA 熱圖的最小輸出間隔（0=每幀），例：m 50");
    Serial.println("  s            顯示目前狀態");
    Serial.println("  ?            顯示這份說明");
    Serial.println("==========================================");
}

void printStatus() {
    Serial.println();
    Serial.printf("模式: %s\n", modeName());
    if (mode == MODE_SWEEP)
        Serial.printf("掃掠: 傾角 %.1f 度，週期 %.2f 秒\n", sweepTheta,
                      sweepPeriodS);
    if (mode == MODE_HOP)
        Serial.printf("振盪: %.1f ± %.1f mm，週期 %.2f 秒\n", height,
                      hopAmplitude, hopPeriodS);
    if (mode == MODE_BOUNCE) {
        Serial.printf(
            "彈跳: 振幅 %.1f mm 週期 %.2f 秒  目標 %.0f mm  量到 %.0f "
            "mm（飛行 %.0f ms）\n",
            bounceAmp, bouncePeriodS, bounceTargetMm, bounceHeightMm,
            bounceFlightS * 1000.0f);
        if (bouncePhase >= 0.0f)
            Serial.printf(
                "鎖相: 撞擊相位 %.3f（目標 %.3f，誤差 %+.3f）傾角上限 %.1f "
                "度\n",
                bouncePhase, BOUNCE_PHASE_TARGET,
                bouncePhase - BOUNCE_PHASE_TARGET, bounceTiltCap);
        else
            Serial.println("鎖相: 尚未量到撞擊相位");
    }
    if (mode == MODE_ORBIT)
        Serial.printf("繞圈: 圓心 (%.1f, %.1f)，半徑 %.1f mm，週期 %.2f 秒\n",
                      targetX, targetY, orbitRadius, orbitPeriodS);
    Serial.printf("控制週期: 平均 %.1f ms 最大 %u ms（目標 %d ms）%s\n",
                  loopAvgMs, loopMaxMs, CONTROL_PERIOD_MS,
                  loopMaxMs > 2 * CONTROL_PERIOD_MS ? "  <-- 偏慢" : "");
    Serial.printf(
        "  其中 感測器讀取 平均 %.1f ms 最大 %u ms；DATA 輸出 %.1f ms\n",
        sensorAvgMs, sensorMaxMs, dataAvgMs);
    loopMaxMs = 0;  // 讀過就重設，方便觀察某個動作造成的尖峰。
    sensorMaxMs = 0;
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
//      maxTilt,distMm,validZones,integX,integY,outAlpha,dAlpha,integLimit,
//      hMin,hMax,servoMin,servoMax
// mode 為 Mode 的數值，x、y 為平台座標（mm）。最後四欄是 Config.h 的高度與
// 伺服安全角度上下限，GUI 以此為準，不要在 GUI 另外寫死。
void printStat(bool present, float px, float py) {
    Serial.printf(
        "STAT,%d,%d,%.2f,%.2f,%.2f,%.1f,%.1f,%.1f,%.1f,%.4f,%.4f,%.4f,%.1f,"
        "%.1f,%.1f,%.1f,%.1f,%d,%.2f,%.2f,%.2f,%.2f,%.2f,%.1f,%.1f,%.1f,"
        "%.1f\n",
        (int)mode, present ? 1 : 0, px, py, tiltTheta, tiltPhi, lastServo[0],
        lastServo[1], lastServo[2], pid.kp, pid.ki, pid.kd, targetX, targetY,
        height, maxTilt, lastDistMm, lastValidZones, pid.integralX(),
        pid.integralY(), pid.outputAlpha, pid.dAlpha, pid.integralLimit,
        minUsableHeight(), maxUsableHeight(), SERVO_SAFE_MIN_DEG,
        SERVO_SAFE_MAX_DEG);
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

        case 'w':  // 開迴路圓錐掃掠：驗證運動學、伺服方向與座標系符號。
            if (parseNumbers(tok, n, 1, 2, v)) {
                if (v[0] < 0 || v[0] > maxTilt) {
                    Serial.printf(">> 錯誤：傾角需在 0 ~ %.1f 度之間\n",
                                  maxTilt);
                    break;
                }
                if (v[1] < SWEEP_MIN_PERIOD_S) {
                    Serial.printf(">> 錯誤：週期需 >= %.2f 秒（伺服跟不上）\n",
                                  SWEEP_MIN_PERIOD_S);
                    break;
                }
                bool reachable = true;
                for (float phi = 0.0f; phi < 360.0f;
                     phi += SWEEP_PHI_CHECK_STEP_DEG)
                    if (!poseWithinServoRange(v[0], phi, height)) {
                        reachable = false;
                        break;
                    }
                if (!reachable) {
                    Serial.printf(
                        ">> 錯誤：傾角 %.1f 度有部分方向超出伺服角度範圍（高度 "
                        "%.1f mm 下約 %.1f 度以內可用）\n",
                        v[0], height, usableSweepTilt());
                    break;
                }
                mode = MODE_SWEEP;
                manualBall.present = false;
                sweepTheta = v[0];
                sweepPeriodS = v[1];
                sweepStartMs = millis();
                Serial.printf(
                    ">> 圓錐掃掠：傾角 %.1f 度，每 %.2f 秒繞一圈（l 回水平）\n",
                    sweepTheta, sweepPeriodS);
            } else
                Serial.println(">> 格式：w 傾角 週期（秒）");
            break;

        case 'v':  // 開迴路高度振盪：驗證伺服是否快得足以把球彈起來。
            if (parseNumbers(tok, n, 1, 2, v)) {
                if (v[0] <= 0) {
                    Serial.println(">> 錯誤：振幅需大於 0");
                    break;
                }
                if (v[1] < HOP_MIN_PERIOD_S) {
                    Serial.printf(">> 錯誤：週期需 >= %.2f 秒（伺服跟不上）\n",
                                  HOP_MIN_PERIOD_S);
                    break;
                }
                if (!poseWithinServoRange(0.0f, 0.0f, height - v[0]) ||
                    !poseWithinServoRange(0.0f, 0.0f, height + v[0])) {
                    Serial.printf(
                        ">> 錯誤：%.1f ± %.1f mm 超出伺服角度範圍（約 ± %.1f "
                        "mm "
                        "以內可用）\n",
                        height, v[0], usableHopAmplitude());
                    break;
                }
                mode = MODE_HOP;
                manualBall.present = false;
                hopAmplitude = v[0];
                hopPeriodS = v[1];
                sweepStartMs = millis();
                tiltTheta = tiltPhi = 0;
                Serial.printf(
                    ">> 高度振盪：%.1f ± %.1f mm，週期 %.2f 秒（l 回水平）\n",
                    height, hopAmplitude, hopPeriodS);
            } else
                Serial.println(">> 格式：v 振幅 週期（秒）");
            break;

        case 'q':  // 閉迴路球繞圈：沿用 PID，目標點繞目前目標點轉動。
            if (!USE_IR_SENSOR) {
                Serial.println(
                    ">> 手動模式沒有連續的球位置回饋，q 指令只在紅外線模式下"
                    "有意義");
                break;
            }
            if (parseNumbers(tok, n, 1, 2, v)) {
                if (v[0] <= 0 || v[0] > ORBIT_MAX_RADIUS_MM) {
                    Serial.printf(">> 錯誤：半徑需在 0 ~ %.1f mm 之間\n",
                                  ORBIT_MAX_RADIUS_MM);
                    break;
                }
                if (v[1] < ORBIT_MIN_PERIOD_S) {
                    Serial.printf(
                        ">> 錯誤：週期需 >= %.2f 秒（PID 可能跟不上，這只是"
                        "保守的起始值）\n",
                        ORBIT_MIN_PERIOD_S);
                    break;
                }
                mode = MODE_ORBIT;
                manualBall.present = false;
                orbitRadius = v[0];
                orbitPeriodS = v[1];
                orbitStartMs = millis();
                pid.reset();
                Serial.printf(
                    ">> 閉迴路繞圈：圓心 (%.1f, %.1f)，半徑 %.1f mm，每 %.2f "
                    "秒一圈（l 回水平、g 可移動圓心）\n",
                    targetX, targetY, orbitRadius, orbitPeriodS);
            } else
                Serial.println(">> 格式：q 半徑 週期（秒）");
            break;

        case 'j':  // 閉迴路彈跳：垂直正弦 + 水平 PID。
            if (parseNumbers(tok, n, 1, 2, v)) {
                if (v[0] < BOUNCE_AMP_MIN_MM || v[0] > BOUNCE_AMP_MAX_MM) {
                    Serial.printf(">> 錯誤：振幅需在 %.0f ~ %.0f mm 之間\n",
                                  BOUNCE_AMP_MIN_MM, BOUNCE_AMP_MAX_MM);
                    break;
                }
                if (v[1] < BOUNCE_MIN_PERIOD_S) {
                    Serial.printf(">> 錯誤：週期需 >= %.2f 秒\n",
                                  BOUNCE_MIN_PERIOD_S);
                    break;
                }
                if (!poseWithinServoRange(0.0f, 0.0f, height - v[0]) ||
                    !poseWithinServoRange(0.0f, 0.0f, height + v[0])) {
                    Serial.printf(
                        ">> 錯誤：%.1f ± %.1f mm 超出伺服角度範圍（約 ± %.1f "
                        "mm 以內可用）\n",
                        height, v[0], usableHopAmplitude());
                    break;
                }
                float tgt = 0.0f;
                if (n >= 4 && !parseNumber(tok[3], tgt)) {
                    Serial.println(">> 格式：j 振幅 週期 [目標彈跳高度]");
                    break;
                }
                if (!USE_IR_SENSOR) {
                    Serial.println(
                        ">> 警告：USE_IR_SENSOR 為 false，水平 PID 不會作用");
                }
                float bt = bounceMaxTilt(v[0]);
                if (bt <= 0.0f) {
                    Serial.printf(
                        ">> 錯誤：振幅 %.1f mm 時無法同時傾斜，水平 PID 會失"
                        "效。請降低振幅或把高度調低（例如 h 70）\n",
                        v[0]);
                    break;
                }
                mode = MODE_BOUNCE;
                bounceAmp = v[0];
                bounceCapAmp = v[0];
                bounceTiltCap = bt;
                bouncePeriodS = v[1];
                bounceTargetMm = max(tgt, 0.0f);
                bounceWasPresent = false;
                bounceFoundMs = 0;
                bounceLostMs = 0;
                bouncePhase = -1.0f;
                bouncePhaseAcc = 0.0f;
                bouncePhaseFound = 0.0f;
                bounceRateCorr = 0.0f;
                bouncePidMs = 0;
                bounceFlightS = 0.0f;
                bounceHeightMm = 0.0f;
                pid.reset();
                Serial.printf(
                    ">> 閉迴路彈跳：%.1f ± %.1f mm，週期 %.2f 秒，水平 PID "
                    "維持在 (%.1f, %.1f)%s（l 停止）\n",
                    height, bounceAmp, bouncePeriodS, targetX, targetY,
                    (bounceTargetMm > 0 && BOUNCE_AMP_GAIN > 0)
                        ? "，振幅會依量到的彈跳高度調整"
                    : bounceTargetMm > 0
                        ? "，目標高度只會量測與回報（BOUNCE_AMP_GAIN 為 0）"
                        : "，固定振幅");
                Serial.printf(">> 此振幅下水平 PID 可用傾角 %.1f 度\n", bt);
            } else
                Serial.println(">> 格式：j 振幅 週期 [目標彈跳高度]");
            break;

        case 'a':
            if (parseNumbers(tok, n, 1, 3, v)) {
                bool outOfRange = false;
                for (int i = 0; i < 3; i++)
                    if (v[i] < SERVO_HARD_MIN_DEG || v[i] > SERVO_HARD_MAX_DEG)
                        outOfRange = true;
                if (outOfRange) {
                    Serial.printf(">> 錯誤：角度需在 %.0f ~ %.0f 之間\n",
                                  SERVO_HARD_MIN_DEG, SERVO_HARD_MAX_DEG);
                    break;
                }
                mode = MODE_RAW;
                manualBall.present = false;
                // 直接角度模式下平台高度無從得知，篩選窗還原成基準高度。
                if (USE_IR_SENSOR) irSensor.setHeight(height);

                // 實際角度 = 輸入角度 + 各自 offset。
                float out[3];
                for (int i = 0; i < 3; i++)
                    out[i] = constrain(v[i] + servoCal[i].offset,
                                       SERVO_HARD_MIN_DEG, SERVO_HARD_MAX_DEG);

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
                if (USE_IR_SENSOR) irSensor.setHeight(height);
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

        case 'f':  // 輸出低通、微分低通與積分貢獻上限；不清除積分。
            if (parseNumbers(tok, n, 1, 3, v)) {
                if (v[0] <= 0 || v[0] > 1 || v[1] <= 0 || v[1] > 1 ||
                    v[2] < 0) {
                    Serial.println(">> 錯誤：α 需在 (0, 1]，積分上限需 >= 0");
                    break;
                }
                pid.outputAlpha = v[0];
                pid.dAlpha = v[1];
                pid.integralLimit = v[2];
                Serial.printf(">> 輸出 α=%.2f  D α=%.2f  積分上限=%.2f 度\n",
                              pid.outputAlpha, pid.dAlpha, pid.integralLimit);
            } else
                Serial.println(">> 格式：f 輸出α D的α 積分上限");
            break;

        case 'h':
            if (parseNumbers(tok, n, 1, 1, v)) {
                if (v[0] < minUsableHeight() || v[0] > maxUsableHeight()) {
                    Serial.printf(">> 錯誤：高度需在 %.1f ~ %.1f mm 之間\n",
                                  minUsableHeight(), maxUsableHeight());
                    break;
                }
                float newMax =
                    min(kinematics.computeMaxTilt(v[0], armBelowBaseFor(v[0])),
                        MAX_TILT_DEG);
                if (newMax <= 0) {
                    Serial.println(">> 錯誤：這個高度無法保持水平");
                    break;
                }
                height = v[0];
                maxTilt = newMax;
                if (USE_IR_SENSOR) irSensor.setHeight(height);
                if (mode == MODE_TILT)
                    applyTilt(min(tiltTheta, maxTilt), tiltPhi);
                Serial.printf(">> 高度 %.1f mm，最大傾角 %.1f 度\n", height,
                              maxTilt);
            } else
                Serial.println(">> 格式：h 高度");
            break;

        case 'm':  // DATA 節流：序列埠阻塞會拖慢控制迴圈。
            if (parseNumbers(tok, n, 1, 1, v)) {
                if (v[0] < 0 || v[0] > 1000) {
                    Serial.println(">> 錯誤：間隔需在 0 ~ 1000 ms 之間");
                    break;
                }
                dataPeriodMs = (uint32_t)v[0];
                Serial.printf(">> DATA 間隔 %u ms（%s）\n", dataPeriodMs,
                              dataPeriodMs ? "節流中" : "每幀都送");
            } else
                Serial.printf(">> DATA 間隔 %u ms。格式：m 毫秒\n",
                              dataPeriodMs);
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
        irSensor.setHeight(height);
    }

    if (!kinematics.solveSpherical(0, 0, height, levelModel)) {
        Serial.println("逆向運動學在預設高度無解，請檢查機構尺寸");
        while (1) delay(1000);
    }
    maxTilt = min(kinematics.computeMaxTilt(height, armBelowBaseFor(height)),
                  MAX_TILT_DEG);
    loadServoOrigin();
    warnHeightRange();

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

        uint32_t tSensor = millis();
        bool gotFrame =
            irSensor.readData(grid_strength, grid_dist, valid_zone_count);
        if (gotFrame) {
            uint32_t sensorMs = millis() - tSensor;
            if (sensorAvgMs == 0.0f) sensorAvgMs = (float)sensorMs;
            sensorAvgMs += 0.05f * (sensorMs - sensorAvgMs);
            if (sensorMs > sensorMaxMs) sensorMaxMs = sensorMs;
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

            // DATA 只是給 GUI 畫熱圖，節流以免阻塞 loop()；估測不受影響。
            // 注意要用條件區塊而不是 return——return 會連同下面的控制更新一起
            // 跳過，等於把控制頻率降到 DATA 的頻率。
            static uint32_t lastData = 0;
            uint32_t nowMs = millis();
            if (!dataPeriodMs || nowMs - lastData >= dataPeriodMs) {
                lastData = nowMs;

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
                uint32_t dms = millis() - nowMs;
                if (dataAvgMs == 0.0f) dataAvgMs = (float)dms;
                dataAvgMs += 0.1f * (dms - dataAvgMs);
            }
        }
    }

    static uint32_t lastControl = 0;
    static uint32_t lastStat = 0;
    uint32_t now = millis();

    if (now - lastControl < CONTROL_PERIOD_MS) return;

    uint32_t period = now - lastControl;
    if (loopAvgMs == 0.0f) loopAvgMs = (float)period;
    loopAvgMs += 0.05f * (period - loopAvgMs);
    if (period > loopMaxMs) loopMaxMs = period;

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

    // 開迴路模式不讀球位置，所以放在紅外線檢查之前（沒接感測器也能驗證機構）。
    if (mode == MODE_SWEEP) {
        float t = (now - sweepStartMs) / 1000.0f;
        tiltTheta = sweepTheta;
        tiltPhi = fmodf(360.0f * t / sweepPeriodS, 360.0f);
        applyTilt(tiltTheta, tiltPhi);
        return;
    }

    if (mode == MODE_HOP) {
        float t = (now - sweepStartMs) / 1000.0f;
        // height 維持為基準高度，STAT 的高度欄位語意不變（GUI 的加減基準）。
        float h = height + hopAmplitude * sinf(TWO_PI * t / hopPeriodS);
        applyTilt(0.0f, 0.0f, h);
        return;
    }

    // 閉迴路彈跳：垂直正弦自己鎖相（實測開迴路的 v 就能維持），回授只負責水平
    // 不掉落與振幅調整，所以不需要抓個別撞擊時機。
    if (mode == MODE_BOUNCE) {
        // 以「球消失多久」量飛行時間；球在頂點會因角尺寸變小而偵測不到。
        if (bounceWasPresent && !present) {
            bounceLostMs = now;
            // 「有球」的區間包住撞擊，取中點的相位當撞擊相位。接觸窗很短，
            // 所以跨越 0/1 時把結束相位加 1 再平均即可。
            if (bounceFoundMs) {
                float end = bouncePhaseAcc;
                if (end < bouncePhaseFound) end += 1.0f;
                float ph = fmodf((bouncePhaseFound + end) * 0.5f, 1.0f);
                bouncePhase = ph;
                if (BOUNCE_PHASE_GAIN > 0.0f) {
                    float err = ph - BOUNCE_PHASE_TARGET;
                    while (err > 0.5f) err -= 1.0f;
                    while (err < -0.5f) err += 1.0f;
                    // 修正頻率而非跳相位：這個偏移維持到下次撞擊（約一個
                    // 週期），累積出來的相位移正好是 -GAIN*err。
                    bounceRateCorr =
                        constrain(-BOUNCE_PHASE_GAIN * err, -BOUNCE_RATE_LIMIT,
                                  BOUNCE_RATE_LIMIT);
                }
            }
        } else if (!bounceWasPresent && present && bounceLostMs) {
            uint32_t gap = now - bounceLostMs;
            if (gap >= BOUNCE_MIN_FLIGHT_MS && gap <= BOUNCE_MAX_FLIGHT_MS) {
                bounceFlightS = gap / 1000.0f;
                // h = g * t^2 / 8（t 為完整飛行時間，此處為低估值）。
                bounceHeightMm = 9810.0f * bounceFlightS * bounceFlightS / 8.0f;
                if (bounceTargetMm > 0.0f) {
                    bounceAmp +=
                        BOUNCE_AMP_GAIN * (bounceTargetMm - bounceHeightMm);
                    bounceAmp = constrain(bounceAmp, BOUNCE_AMP_MIN_MM,
                                          BOUNCE_AMP_MAX_MM);
                    // 振幅改變後仍要守住伺服角度範圍。
                    while (bounceAmp > BOUNCE_AMP_MIN_MM &&
                           (!poseWithinServoRange(0, 0, height - bounceAmp) ||
                            !poseWithinServoRange(0, 0, height + bounceAmp)))
                        bounceAmp -= 0.5f;
                    // 撞擊事件才發生，不在每個控制週期。
                    updateBounceTiltCap(bounceAmp);
                }
            }
            bounceLostMs = 0;
            bounceFoundMs = now;
            bouncePhaseFound = bouncePhaseAcc;
        }
        if (present && !bounceFoundMs) {
            bounceFoundMs = now;
            bouncePhaseFound = bouncePhaseAcc;
        }
        bounceWasPresent = present;

        // 水平：球不在視野時凍結傾角，而不是讓 PID 衰減——飛行中沒有新資訊，
        // 衰減只會讓平台在球落下前跑掉。
        if (present) {
            // 用距上次更新的真實間隔，而不是控制週期。BalanceController 會把
            // dt 夾在 0.005~0.1 秒，所以長飛行仍會被低估，但遠勝於用 20ms。
            float pidDt = bouncePidMs ? (now - bouncePidMs) / 1000.0f : dt;
            bouncePidMs = now;
            pid.update(true, px, py, targetX, targetY, pidDt, bounceTiltCap,
                       tiltTheta, tiltPhi);
        }

        bouncePhaseAcc += dt / bouncePeriodS * (1.0f + bounceRateCorr);
        bouncePhaseAcc -= floorf(bouncePhaseAcc);
        float h = height + bounceAmp * sinf(TWO_PI * bouncePhaseAcc);
        applyTilt(tiltTheta, tiltPhi, h);
        return;
    }

    // 手動模式與 t / l / a 之後都不跑 PID，輸入 r 恢復。
    if (!USE_IR_SENSOR || (mode != MODE_BALL && mode != MODE_ORBIT)) return;

    // MODE_ORBIT 跟 MODE_BALL 共用同一個 PID：目標點只是從固定的
    // (targetX, targetY) 換成繞著它轉動的點，圓心仍可用 g 指令移動。
    // BalanceController 的微分項取球的量測值變化（不是目標點變化），所以移動
    // 目標點不會造成微分項暴衝；控制週期沒變，延遲只來自原本就有的
    // CONTROL_PERIOD_MS，沒有額外引入大延遲。
    float tx = targetX, ty = targetY;
    if (mode == MODE_ORBIT) {
        float t = (now - orbitStartMs) / 1000.0f;
        float ang = TWO_PI * t / orbitPeriodS;
        tx += orbitRadius * cosf(ang);
        ty += orbitRadius * sinf(ang);
    }

    pid.update(present, px, py, tx, ty, dt, maxTilt, tiltTheta, tiltPhi);
    applyTilt(tiltTheta, tiltPhi);
}
