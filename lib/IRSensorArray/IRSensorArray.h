#ifndef IR_SENSOR_ARRAY_H
#define IR_SENSOR_ARRAY_H

#include <Arduino.h>
#include <Wire.h>
#include <vl53l7cx_class.h>

#include "Config.h"

class IRSensorArray {
public:
    int current_grid_size = TOF_DEFAULT_GRID_SIZE;

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
