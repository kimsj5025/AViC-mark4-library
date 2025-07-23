#ifndef AVIC_H
#define AVIC_H

#include <Wire.h>
#include "SD_MMC.h"
#include <Arduino.h>

class AViC {
public:
    // --- Public Member Variables (Pin Definitions) ---
    const int IGNITE_PIN = 7;
    const int PyroCh1 = 7;
    const int PyroCh2 = 8;
    const int PyroCh3 = 9;
    const int PyroCh4 = 10;
    const int LED_PIN = 15;
    
    // 도, 미, 솔 주파수(Hz)
    int NOTE_C = 1047;  // 도 (C6)
    int NOTE_E = 1319;  // 미 (E6)
    int NOTE_G = 1568;  // 솔 (G6)
    int BUZZER_PIN = 11;
    int BUZZER_CHANNEL = 0;
    int BUZZER_RESOLUTION = 8;
    bool isLedOn;
    int GPS_RX = 37;
    int GPS_TX = 38;

    int clk = 26;
    int cmd = 21;
    int d0 = 33;
    int d1 = 34;
    int d2 = 35;
    int d3 = 36;

    int ICM45686_ADDR = 0x68;
    int LIS2MDL_ADDR = 0x1E;
    int MS5607_ADDR = 0x76;

    int I2C_SDA = 5;
    int I2C_SCL = 6;

    // --- Constructor ---
    AViC();
    
    // --- Public Methods ---
    void initialize(bool beep = false); // Call this in setup() to configure the pins
    void ledSW();
    void playTone(int freq = 1047, int duration = 200);
    void Waring(int times);
    void PyroON(int num);
    void PyroOff(int num);

    void getAcceleData();
    void getMagnetData();
    void getBaroData();
    
private:
    // --- Private Member Variables ---
};

#endif