#include "cpp_main.h"
#include "main.h" // Access to HAL handles (huart1, etc.)

#include <cmath>
#include <cstring>
#include <string>

#include "crc.h"
#include "tc_max31855_spi.h"

using namespace std; 

// Define transmittable datastructures
#pragma pack(push, 1)
struct AdcData {
    uint32_t s4;
    uint32_t s5;
    uint32_t s6;
    uint32_t s7;
    uint32_t s8;
    uint32_t pt0;
    uint32_t pt1;
    uint32_t pt2;
    uint32_t pwr0;
    uint32_t pwr1;
    uint32_t s0;
    uint32_t s1;
    uint32_t s2;
    uint32_t s3;
    uint32_t pt3;
    uint32_t pt4;
};

struct GseCommand {
    bool igniter0Fire;
    bool igniter1Fire;
    bool alarm;
    bool solenoidStateGn2Fill;
    bool solenoidStateGn2Vent;
    bool solenoidStateGn2Disconnect;
    bool solenoidStateMvasFill;
    bool solenoidStateMvasVent;
    bool solenoidStateMvasOpen;
    bool solenoidStateMvasClose;
    bool solenoidStateLoxVent;
    bool solenoidStateLngVent;
    uint32_t crc;
};

struct GseData {
    uint32_t timestamp;
    bool igniterArmed;
    bool igniter0Continuity;
    bool igniter1Continuity;
    bool igniterInternalState0;
    bool igniterInternalState1;
    bool alarmInternalState;
    bool solenoidInternalStateGn2Fill;
    bool solenoidInternalStateGn2Vent;
    bool solenoidInternalStateGn2Disconnect;
    bool solenoidInternalStateMvasFill;
    bool solenoidInternalStateMvasVent;
    bool solenoidInternalStateMvasOpen;
    bool solenoidInternalStateMvasClose;
    bool solenoidInternalStateLoxVent;
    bool solenoidInternalStateLngVent;
    float supplyVoltage0 = std::nanf("");
    float supplyVoltage1 = std::nanf("");
    float solenoidCurrentGn2Fill = std::nanf("");
    float solenoidCurrentGn2Vent = std::nanf("");
    float solenoidCurrentGn2Disconnect = std::nanf("");
    float solenoidCurrentMvasFill = std::nanf("");
    float solenoidCurrentMvasVent = std::nanf("");
    float solenoidCurrentMvasOpen = std::nanf("");
    float solenoidCurrentMvasClose = std::nanf("");
    float solenoidCurrentLoxVent = std::nanf("");
    float solenoidCurrentLngVent = std::nanf("");
    float temperatureLox = std::nanf("");
    float temperatureLng = std::nanf("");
    float pressureGn2 = std::nanf("");
    float pressureLoxInjTee = std::nanf("");
    float pressureVent = std::nanf("");
    float pressureLoxMvas = std::nanf("");
    uint32_t crc;
};
#pragma pack(pop)

// Add extern defined handles to peripheral interfaces defined in main.c
extern ADC_HandleTypeDef hadc1;

extern SPI_HandleTypeDef hspi3;

extern TIM_HandleTypeDef htim4;

extern TIM_HandleTypeDef htim5;

extern UART_HandleTypeDef huart3;

extern PCD_HandleTypeDef hpcd_USB_OTG_FS;

// TC handlers
// (TC0 INOPERABLE)
// (TC1 INOPERABLE)
TcMax31855Spi tc2(&hspi3, TC2_nCS_GPIO_Port, TC2_nCS_Pin, 100);
TcMax31855Spi tc3(&hspi3, TC3_nCS_GPIO_Port, TC3_nCS_Pin, 100);
TcMax31855Spi tc4(&hspi3, TC4_nCS_GPIO_Port, TC4_nCS_Pin, 100);
TcMax31855Spi tc5(&hspi3, TC5_nCS_GPIO_Port, TC5_nCS_Pin, 100);

// solenoid internal states, these are energized state, not open or closed
int solenoidState0;
int solenoidState1;
int solenoidState2;
int solenoidState3;
int solenoidState4;
int solenoidState5;
int solenoidState6;
int solenoidState7;
int solenoidState8;

// igniter internal states
int igniterState0;
int igniterState1;

// alarm
int alarmState;

// incoming command
bool newCommand = false;
uint8_t commandBuffer[sizeof(GseCommand)];
GseCommand command;

void cpp_main(void)
{
    HAL_GPIO_WritePin(ETH_nRST_GPIO_Port, ETH_nRST_Pin, GPIO_PIN_SET);

    HAL_GPIO_WritePin(TC2_nCS_GPIO_Port, TC2_nCS_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(TC3_nCS_GPIO_Port, TC3_nCS_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(TC4_nCS_GPIO_Port, TC4_nCS_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(TC5_nCS_GPIO_Port, TC5_nCS_Pin, GPIO_PIN_SET);

    solenoidState0 = 0;
    solenoidState1 = 0;
    solenoidState2 = 0;
    solenoidState3 = 0;
    solenoidState4 = 0;
    solenoidState5 = 0;
    solenoidState6 = 0;
    solenoidState7 = 0;
    solenoidState8 = 0;

    igniterState0 = 0;
    igniterState1 = 0;

    alarmState = 0;

    HAL_Delay(1000);

    HAL_TIM_Base_Start(&htim4);
    HAL_TIM_Base_Start(&htim5);

    HAL_UART_Receive_IT(&huart3, commandBuffer, sizeof(GseCommand));

    uint32_t usbBufferTimer = TIM5->CNT << 16 | TIM4->CNT;

    while (1) {
        GseData data;

        uint32_t timestamp = TIM5->CNT << 16 | TIM4->CNT;
        data.timestamp = timestamp;

        // update internal states
        if (newCommand) {
            newCommand = false;

            solenoidState0 = (int)(command.solenoidStateGn2Fill);
            solenoidState1 = (int)(command.solenoidStateGn2Vent);
            solenoidState2 = (int)(command.solenoidStateGn2Disconnect);
            solenoidState3 = (int)(command.solenoidStateMvasFill);
            solenoidState4 = (int)(command.solenoidStateMvasVent);
            solenoidState5 = (int)(command.solenoidStateMvasOpen);
            solenoidState6 = (int)(command.solenoidStateMvasClose);
            solenoidState7 = (int)(command.solenoidStateLoxVent);
            solenoidState8 = (int)(command.solenoidStateLngVent);
            igniterState0 = (int)command.igniter0Fire;
            igniterState1 = (int)command.igniter1Fire;
            alarmState = (int)command.alarm;
        }

        // internal states feedback
        data.solenoidInternalStateGn2Fill = (bool)solenoidState0;
        data.solenoidInternalStateGn2Vent = (bool)solenoidState1;
        data.solenoidInternalStateGn2Disconnect = (bool)solenoidState2;
        data.solenoidInternalStateMvasFill = (bool)solenoidState3;
        data.solenoidInternalStateMvasVent = (bool)solenoidState4;
        data.solenoidInternalStateMvasOpen = (bool)solenoidState5;
        data.solenoidInternalStateMvasClose = (bool)solenoidState6;
        data.solenoidInternalStateLoxVent = (bool)solenoidState7;
        data.solenoidInternalStateLngVent = (bool)solenoidState8;
        data.igniterInternalState0 = (bool)igniterState0;
        data.igniterInternalState1 = (bool)igniterState1;
        data.alarmInternalState = (bool)alarmState;

        // switch solenoids
        HAL_GPIO_WritePin(SOLENOID0_EN_GPIO_Port, SOLENOID0_EN_Pin, (GPIO_PinState)solenoidState0);
        HAL_GPIO_WritePin(SOLENOID1_EN_GPIO_Port, SOLENOID1_EN_Pin, (GPIO_PinState)solenoidState1);
        HAL_GPIO_WritePin(SOLENOID2_EN_GPIO_Port, SOLENOID2_EN_Pin, (GPIO_PinState)solenoidState2);
        HAL_GPIO_WritePin(SOLENOID3_EN_GPIO_Port, SOLENOID3_EN_Pin, (GPIO_PinState)solenoidState3);
        HAL_GPIO_WritePin(SOLENOID4_EN_GPIO_Port, SOLENOID4_EN_Pin, (GPIO_PinState)solenoidState4);
        HAL_GPIO_WritePin(SOLENOID5_EN_GPIO_Port, SOLENOID5_EN_Pin, (GPIO_PinState)solenoidState5);
        HAL_GPIO_WritePin(SOLENOID6_EN_GPIO_Port, SOLENOID6_EN_Pin, (GPIO_PinState)solenoidState6);
        HAL_GPIO_WritePin(SOLENOID7_EN_GPIO_Port, SOLENOID7_EN_Pin, (GPIO_PinState)solenoidState7);
        HAL_GPIO_WritePin(SOLENOID8_EN_GPIO_Port, SOLENOID8_EN_Pin, (GPIO_PinState)solenoidState8);

        // igniter
        data.igniterArmed = (bool)HAL_GPIO_ReadPin(ARMED_GPIO_Port, ARMED_Pin);
        data.igniter0Continuity = !(bool)HAL_GPIO_ReadPin(EMATCH0_CONT_GPIO_Port, EMATCH0_CONT_Pin);
        data.igniter1Continuity = !(bool)HAL_GPIO_ReadPin(EMATCH1_CONT_GPIO_Port, EMATCH1_CONT_Pin);

        if (data.igniterArmed && igniterState0) {
            HAL_GPIO_WritePin(EMATCH0_FIRE_GPIO_Port, EMATCH0_FIRE_Pin, GPIO_PIN_SET);
        } else {
            HAL_GPIO_WritePin(EMATCH0_FIRE_GPIO_Port, EMATCH0_FIRE_Pin, GPIO_PIN_RESET);
        }
        if (data.igniterArmed && igniterState1) {
            HAL_GPIO_WritePin(EMATCH1_FIRE_GPIO_Port, EMATCH1_FIRE_Pin, GPIO_PIN_SET);
        } else {
            HAL_GPIO_WritePin(EMATCH1_FIRE_GPIO_Port, EMATCH1_FIRE_Pin, GPIO_PIN_RESET);
        }

        // alarm
        HAL_GPIO_WritePin(ALARM_GPIO_Port, ALARM_Pin, (GPIO_PinState)alarmState);

        // ADC operations
        AdcData rawData = {0};
        for (int i = 0; i < 16; i++) {
            HAL_ADC_Start(&hadc1);
            HAL_ADC_PollForConversion(&hadc1, 10);
            uint32_t data = HAL_ADC_GetValue(&hadc1);
            *(((uint32_t *)&rawData) + i) += data;
        }

        data.pressureGn2 = 0.00128 * (float)rawData.pt0;                    // PT0 -> GN2 (5000 PSI)
        data.pressureLoxInjTee =  0.00128 * (float)rawData.pt1;             // PT1 -> LOX pressure at the Tee
        data.pressureVent = 0.00128 * (float)rawData.pt2;                   // PT2 -> Chamber/Vent pressure
        data.pressureLoxMvas = 0.00128 * (float)rawData.pt3;                // PT3 -> LOX pressure at MVAS
        (void)(0.00128f * (float)rawData.pt4);                              // UNUSED
        data.solenoidCurrentGn2Fill = 0.000817f * (float)rawData.s0;        // S0 -> GN2 Fill
        data.solenoidCurrentGn2Vent = 0.000817f * (float)rawData.s1;        // S1 -> GN2 Vent
        data.solenoidCurrentGn2Disconnect = 0.000817f * (float)rawData.s2;  // S2 -> GN2 QD
        data.solenoidCurrentMvasFill = 0.000817f * (float)rawData.s3;       // S3 -> MVAS Fill
        data.solenoidCurrentMvasVent = 0.000817f * (float)rawData.s4;       // S4 -> MVAS Vent
        data.solenoidCurrentMvasOpen = 0.000817f * (float)rawData.s5;       // S5 -> MVAS Open
        data.solenoidCurrentMvasClose = 0.000817f * (float)rawData.s6;      // S6 -> MVAS Close
        data.solenoidCurrentLoxVent = 0.000817f * (float)rawData.s7;        // S7 -> LOX Vent
        data.solenoidCurrentLngVent = 0.000817f * (float)rawData.s8;        // S8 -> LNG Vent
        data.supplyVoltage0 = 0.0062f * (float)rawData.pwr0 + 0.435f;
        data.supplyVoltage1 = 0.0062f * (float)rawData.pwr1 + 0.435f;

        // read thermocouples
        TcMax31855Spi::Data tcData;
        // (TC0 INOPERABLE)
        // (TC1 INOPERABLE)
        tcData = tc2.Read();
        if (tcData.valid) {
            data.temperatureLox = tcData.tcTemperature;  // TC2 -> LOX Temperature
        }
        tcData = tc3.Read();
        if (tcData.valid) {
            data.temperatureLng = tcData.tcTemperature;  // TC3 -> LNG Temperature
        }
        tcData = tc4.Read();
        if (tcData.valid) {
            (void)tcData.tcTemperature;  // UNUSED
        }
        tcData = tc5.Read();
        if (tcData.valid) {
            (void)tcData.tcTemperature;  // UNUSED
        }

        // USB
        /*
        char buffer[1024] = {0};
        // if(data.timestamp - usbBufferTimer > 500)
        {
            sprintf(buffer,
                "Timestamp: %08X\r\n"
                "IGNITER\r\n"
                "Armed: %d  Igniter 0 Continuity: %d  Igniter 1 Continuity: %d\r\n"
                "(G)Igniter 1 Fire: %d  (H)Igniter 2 Fire: %d\r\n"
                "\r\n"
                "SOLENOID\r\n"
                "(1)GN2 Fill: %d-%04d  (2)GN2 Vent: %d-%04d  (3)GN2 QD: %d-%04d  (4)MVAS Fill: %d-%04d  (5)MVAS Vent: %d-%04d  "
                "(6)MVAS Open: %d-%04d  (7)MVAS Close: %d-%04d  (8)LOX Vent: %d-%04d  (9)LNG Vent: %d-%04d\r\n"
                "\r\n"
                "PRESSURE & TEMPERATURE & VOLTAGE\r\n"
                "Supply 0: %02d  Supply 1: %02d \r\n"
                "GN2 Pressure: %04d  Chamber/Vent Pressure: %04d LOX Injector Tee Pressure %04d LOX MVAS Pressure %04d \r\n"
                "LOX Temperature: %03d  LNG Temperature: %03d\r\n"
                "(A)Alarm: %d\r\n"
                "---------------------\r\n",
                (unsigned int)data.timestamp, (int)data.igniterArmed, (int)data.igniter0Continuity, (int)data.igniter1Continuity, (int)data.igniterInternalState0, (int)data.igniterInternalState1,
                (int)data.solenoidInternalStateGn2Fill, (int)(data.solenoidCurrentGn2Fill * 1000), (int)data.solenoidInternalStateGn2Vent, (int)(data.solenoidCurrentGn2Vent * 1000),
                (int)data.solenoidInternalStateGn2Disconnect, (int)(data.solenoidCurrentGn2Disconnect * 1000), (int)data.solenoidInternalStateMvasFill, (int)(data.solenoidCurrentMvasFill * 1000),
                (int)data.solenoidInternalStateMvasVent, (int)(data.solenoidCurrentMvasVent * 1000), (int)data.solenoidInternalStateMvasOpen, (int)(data.solenoidCurrentMvasOpen * 1000),
                (int)data.solenoidInternalStateMvasClose, (int)(data.solenoidCurrentMvasClose * 1000), (int)data.solenoidInternalStateLoxVent, (int)(data.solenoidCurrentLoxVent * 1000),
                (int)data.solenoidInternalStateLngVent, (int)(data.solenoidCurrentLngVent * 1000), (int)data.supplyVoltage0, (int)data.supplyVoltage1, 
                (int)(data.pressureGn2 * 1000), (int)(data.pressureLoxInjTee * 1000), (int)(data.pressureVent * 1000), (int)(data.pressureLoxMvas * 1000), 
                (int)data.temperatureLox, (int)data.temperatureLng, (int)data.alarmInternalState);
            CDC_Transmit_FS((uint8_t *)buffer, strlen(buffer));

            //update buffer timer
            usbBufferTimer = data.timestamp;
        }
        */

        // ethernet
        uint32_t crc = Crc32((uint8_t *)&data, sizeof(GseData) - 4);
        data.crc = crc;
        HAL_UART_Transmit(&huart3, (uint8_t *)&data, sizeof(GseData), 100);

        // loop time control (the timestamp rolls over after 49 hours, should be ok)
        while ((TIM5->CNT << 16 | TIM4->CNT) - timestamp < 100) {
        }
    }
}