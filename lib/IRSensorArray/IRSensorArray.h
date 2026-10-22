#ifndef IR_SENSOR_ARRAY_H
#define IR_SENSOR_ARRAY_H

#include <Arduino.h>
#include <Wire.h>
#include <vl53l7cx_class.h>

// LPn, INT 不需要接
#define SDA_PIN 22
#define SCL_PIN 21
#define SENSOR_I2C_ADDRESS 0x29
#define I2C_CLOCK_SPEED 400000

#define MAX_GRID_SIZE 8

#define FILTER_MIN_DISTANCE 150
#define FILTER_MAX_DISTANCE 800

#define FILTER_MIN_STRENGTH 1
#define FILTER_MAX_STRENGTH 4000

class IRSensorArray {
public:
    int current_grid_size = 4;

    IRSensorArray();
    void begin();
    void changeResolution(int new_size);
    bool readData(float grid_strength[MAX_GRID_SIZE][MAX_GRID_SIZE],
                  float grid_dist[MAX_GRID_SIZE][MAX_GRID_SIZE],
                  int& valid_zone_count);

private:
    VL53L7CX sensor;
    VL53L7CX_ResultsData results;
    void halt(const char* msg, int status);
};

#endif