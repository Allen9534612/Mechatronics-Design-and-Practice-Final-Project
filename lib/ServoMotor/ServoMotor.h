#ifndef SERVO_MOTOR_H
#define SERVO_MOTOR_H

#include <Adafruit_PWMServoDriver.h>
#include <Arduino.h>
#include <Wire.h>

class ServoMotor {
public:
    // 建構子：設定 PCA9685 I2C 位址
    explicit ServoMotor(uint8_t address = 0x40);

    // 初始化 PCA9685
    bool begin(uint8_t sdaPin = 21, uint8_t sclPin = 22,
               uint16_t frequency = 50);

    // 控制指定 Channel 的角度
    bool setAngle(uint8_t channel, float angle);

    // 三顆馬達的簡易介面
    bool motor1(float angle);
    bool motor2(float angle);
    bool motor3(float angle);

    // 同時設定三顆馬達
    bool setAll(float angle);

    // 最後一次送到該 Channel 的 PCA9685 計數值（除錯用）
    uint16_t ticksOf(uint8_t channel) const;

    // 設定單顆馬達的最小、最大脈波寬度
    bool setPulseRange(uint8_t channel, uint16_t minUs, uint16_t maxUs);

private:
    Adafruit_PWMServoDriver pwm;
    uint8_t i2cAddress;

    bool initialized = false;

    uint16_t lastTicks[16] = {0};

    // 必須與 setPWMFreq() 使用的頻率一致
    uint16_t pwmFrequency = 50;

    // 各 Channel 的預設脈波範圍
    uint16_t pulseMin[16] = {500, 500, 500, 500, 500, 500, 500, 500,
                             500, 500, 500, 500, 500, 500, 500, 500};

    uint16_t pulseMax[16] = {2500, 2500, 2500, 2500, 2500, 2500, 2500, 2500,
                             2500, 2500, 2500, 2500, 2500, 2500, 2500, 2500};
};

#endif