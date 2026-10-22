#ifndef SERVO_MOTOR_H
#define SERVO_MOTOR_H

#include <Adafruit_PWMServoDriver.h>
#include <Arduino.h>
#include <Wire.h>

#include "Config.h"

// 透過 PCA9685 驅動角度伺服。
class ServoMotor {
public:
    explicit ServoMotor(uint8_t address = PCA9685_ADDRESS);

    // 初始化 I2C 與 PCA9685。找不到裝置時回傳 false。
    bool begin(uint8_t sdaPin = I2C_SDA_PIN, uint8_t sclPin = I2C_SCL_PIN,
               uint16_t frequency = SERVO_PWM_FREQ_HZ);

    // 角度限制在 0~180 度。
    bool setAngle(uint8_t channel, float angle);

    bool motor1(float angle);
    bool motor2(float angle);
    bool motor3(float angle);
    bool setAll(float angle);

    // 最近一次送到該 channel 的 PCA9685 計數值，除錯用。
    uint16_t ticksOf(uint8_t channel) const;

    bool setPulseRange(uint8_t channel, uint16_t minUs, uint16_t maxUs);

private:
    Adafruit_PWMServoDriver pwm;
    uint8_t i2cAddress;
    bool initialized = false;
    uint16_t lastTicks[16] = {0};
    uint16_t pwmFrequency = SERVO_PWM_FREQ_HZ;  // 需與 setPWMFreq() 一致。
    uint16_t pulseMin[16];
    uint16_t pulseMax[16];
};

#endif
