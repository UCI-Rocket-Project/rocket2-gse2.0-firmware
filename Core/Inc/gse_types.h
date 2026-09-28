#pragma once

#include "stm32f1xx_hal_i2c.h"
#include <cmath>
#include <cstdint>
#include <cstring>

// Define transmittable datastructures
#pragma pack(push, 1)
// ADC outputs connected to the STM32F105 directly
struct Stm32AdcData {
    uint32_t s4;
    uint32_t s5;
    uint32_t s6;
    uint32_t s7;
    uint32_t s8;
    uint32_t s9;
    uint32_t s10;
    uint32_t s11;
    uint32_t pwr0;
    uint32_t pwr1;    
    uint32_t s0;
    uint32_t s1;
    uint32_t s2;
    uint32_t s3;
};

struct GseCommand {
    uint32_t magicHeader = 0xDEADD00D;
    bool igniter0Fire;
    bool igniter1Fire;
    bool alarm;
    bool solenoidState0;
    bool solenoidState1;
    bool solenoidState2;
    bool solenoidState3;
    bool solenoidState4;
    bool solenoidState5;
    bool solenoidState6;
    bool solenoidState7;
    bool solenoidState8;
    bool solenoidState9;
    bool solenoidState10;
    bool solenoidState11;
    uint32_t crc;
};

struct GseData {
    uint32_t magicHeader = 0xDEADBEEF; // definition to discern that this is the start of the GSE data packet

    uint32_t timestamp;
    bool igniterArmed;
    bool igniter0Continuity;
    bool igniter1Continuity;

    bool igniterInternalState0;
    bool igniterInternalState1;
    bool alarmInternalState;
    bool solenoidInternalState0;
    bool solenoidInternalState1;
    bool solenoidInternalState2;
    bool solenoidInternalState3;
    bool solenoidInternalState4;
    bool solenoidInternalState5;
    bool solenoidInternalState6;
    bool solenoidInternalState7;
    bool solenoidInternalState8;
    bool solenoidInternalState9;
    bool solenoidInternalState10;
    bool solenoidInternalState11;

    float supplyVoltage0    = std::nanf("");
    float supplyVoltage1    = std::nanf("");
    float solenoidCurrent0  = std::nanf("");
    float solenoidCurrent1  = std::nanf("");
    float solenoidCurrent2  = std::nanf("");
    float solenoidCurrent3  = std::nanf("");
    float solenoidCurrent4  = std::nanf("");
    float solenoidCurrent5  = std::nanf("");
    float solenoidCurrent6  = std::nanf("");
    float solenoidCurrent7  = std::nanf("");
    float solenoidCurrent8  = std::nanf("");
    float solenoidCurrent9  = std::nanf("");
    float solenoidCurrent10 = std::nanf("");
    float solenoidCurrent11 = std::nanf("");

    uint32_t temperature0;
    uint32_t temperature1;
    uint32_t temperature2;

    // External ADC data
    float pressure0 = std::nanf("");
    float pressure1 = std::nanf("");
    float pressure2 = std::nanf("");
    float pressure3 = std::nanf("");
    float loadCellForce2 = std::nanf("");
    float loadCellForce3 = std::nanf("");
    float loadCellForce4 = std::nanf("");
    float loadCellForce5 = std::nanf("");

    uint32_t crc;
};
#pragma pack(pop)