#include "ServoMotor.h"

ServoMotor::ServoMotor(uint8_t address, TwoWire& bus)
    : i2c(bus), pwm(address, bus), i2cAddress(address) {
    for (int i = 0; i < 16; i++) {
        pulseMin[i] = SERVO_MIN_US;
        pulseMax[i] = SERVO_MAX_US;
    }
}

bool ServoMotor::begin(uint8_t sdaPin, uint8_t sclPin, uint16_t frequency) {
    i2c.begin(sdaPin, sclPin);
    i2c.setClock(SERVO_I2C_FREQ_HZ);

    // pwm.begin() 不檢查裝置是否存在，所以先探測。
    i2c.beginTransmission(i2cAddress);
    if (i2c.endTransmission() != 0) {
        initialized = false;
        return false;
    }

    pwm.begin();
    pwm.setPWMFreq(frequency);
    pwmFrequency = frequency;

    delay(10);
    initialized = true;

    return initialized;
}

bool ServoMotor::setAngle(uint8_t channel, float angle) {
    if (!initialized) return false;

    if (channel >= 16) return false;

    if (isnan(angle) || isinf(angle)) return false;

    angle = constrain(angle, 0.0f, 180.0f);

    float pulseUs = pulseMin[channel] +
                    angle / 180.0f * (pulseMax[channel] - pulseMin[channel]);

    // 換算成 PCA9685 的 12-bit 計數值（50 Hz 時 1 tick ≈ 4.88 us）。
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