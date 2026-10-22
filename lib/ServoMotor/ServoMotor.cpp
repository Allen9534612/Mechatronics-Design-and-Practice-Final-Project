#include "ServoMotor.h"

ServoMotor::ServoMotor(uint8_t address) : pwm(address), i2cAddress(address) {
    for (int i = 0; i < 16; i++) {
        pulseMin[i] = SERVO_MIN_US;
        pulseMax[i] = SERVO_MAX_US;
    }
}

bool ServoMotor::begin(uint8_t sdaPin, uint8_t sclPin, uint16_t frequency) {
    Wire.begin(sdaPin, sclPin);

    // 確認 I2C 上真的有 PCA9685（pwm.begin() 本身不會檢查）
    Wire.beginTransmission(i2cAddress);
    if (Wire.endTransmission() != 0) return false;

    pwm.begin();
    pwm.setPWMFreq(frequency);
    pwmFrequency = frequency;  // 角度換算要用同一個頻率

    delay(10);
    initialized = true;

    return initialized;
}

bool ServoMotor::setAngle(uint8_t channel, float angle) {
    if (!initialized) return false;

    if (channel >= 16) return false;

    if (isnan(angle) || isinf(angle)) return false;

    // 限制角度在 0~180 度
    angle = constrain(angle, 0.0f, 180.0f);

    // 角度轉換成脈波寬度（保留小數，避免截斷）
    float pulseUs = pulseMin[channel] +
                    angle / 180.0f * (pulseMax[channel] - pulseMin[channel]);

    // 依據 PWM 頻率換算成 PCA9685 的 12-bit 計數值（四捨五入）
    // 50 Hz 時 1 tick ≈ 4.88 us，1 度 ≈ 11.1 us ≈ 2.3 tick
    float periodUs = 1000000.0f / pwmFrequency;

    long ticks = lroundf(pulseUs * 4096.0f / periodUs);
    ticks = constrain(ticks, 0L, 4095L);

    pwm.setPWM(channel, 0, (uint16_t)ticks);
    lastTicks[channel] = (uint16_t)ticks;

    return true;
}

bool ServoMotor::motor1(float angle) { return setAngle(0, angle); }

bool ServoMotor::motor2(float angle) { return setAngle(1, angle); }

bool ServoMotor::motor3(float angle) { return setAngle(2, angle); }

bool ServoMotor::setAll(float angle) {
    bool result1 = motor1(angle);
    bool result2 = motor2(angle);
    bool result3 = motor3(angle);

    return result1 && result2 && result3;
}

uint16_t ServoMotor::ticksOf(uint8_t channel) const {
    return channel < 16 ? lastTicks[channel] : 0;
}

bool ServoMotor::setPulseRange(uint8_t channel, uint16_t minUs,
                               uint16_t maxUs) {
    if (channel >= 16) return false;

    if (minUs == 0 || maxUs <= minUs) return false;

    pulseMin[channel] = minUs;
    pulseMax[channel] = maxUs;

    return true;
}