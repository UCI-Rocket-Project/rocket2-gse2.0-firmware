#include "cpp_main.h"
#include "main.h" // Access to HAL handles (huart1, etc.)

#include "crc.h"
#include "stm32f1xx_hal_i2c.h"
#include "tc_max31855_spi.h"
#include "adc_max11614_i2c.h"

#include <cmath>
#include <cstdint>
#include <cstring>

using namespace std; 

// Define transmittable datastructures
#pragma pack(push, 1)
//ADC outputs connected to the STM32F105 directly
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

    uint32_t crc;
};
#pragma pack(pop)

bool newCommand = false;
GseCommand command;
uint8_t commandBuffer[sizeof(GseCommand)];
GseData data;

// Add extern defined handles to peripheral interfaces defined in main.c
extern ADC_HandleTypeDef hadc1;
extern SPI_HandleTypeDef hspi3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim5;
extern UART_HandleTypeDef huart3;
extern I2C_HandleTypeDef hi2c1;
// extern PCD_HandleTypeDef hpcd_USB_OTG_FS;

void cpp_main(void)
{
    

    HAL_GPIO_WritePin(ETH_RST_GPIO_Port, ETH_RST_Pin, GPIO_PIN_RESET);
    HAL_Delay(10); // Hold in reset
    HAL_GPIO_WritePin(ETH_RST_GPIO_Port, ETH_RST_Pin, GPIO_PIN_SET);
    HAL_Delay(250); // Wait for XPort's internal 200ms reset to finish

    // Thermocpuple setup
    TcMax31855Spi::Data tcData;

    TcMax31855Spi tc0(&hspi3, TC0_CS_GPIO_Port, TC0_CS_Pin, 100);
    TcMax31855Spi tc1(&hspi3, TC1_CS_GPIO_Port, TC1_CS_Pin, 100);
    TcMax31855Spi tc2(&hspi3, TC2_CS_GPIO_Port, TC2_CS_Pin, 100);

    tc0.Init();
    tc1.Init();
    tc2.Init();

    // External ADC setup
    AdcMax11614i2c external_adc(&hi2c1, EXT_ADC_SCL_GPIO_Port, EXT_ADC_SCL_Pin);
    external_adc.Init();

    /* init state stuff*/
    bool solenoidState0  = 0;
    bool solenoidState1  = 0;
    bool solenoidState2  = 0;
    bool solenoidState3  = 0;
    bool solenoidState4  = 0;
    bool solenoidState5  = 0;
    bool solenoidState6  = 0;
    bool solenoidState7  = 0;
    bool solenoidState8  = 0;
    bool solenoidState9  = 0;
    bool solenoidState10 = 0;
    bool solenoidState11 = 0;

    bool igniterState0 = 0;
    bool igniterState1 = 0;

    bool alarmState = 0;


    HAL_TIM_Base_Start(&htim4);
    HAL_TIM_Base_Start(&htim5);

    HAL_UART_Receive_IT(&huart3, commandBuffer, sizeof(GseCommand));

    // uint32_t usbBufferTimer = TIM5->CNT << 16 | TIM4->CNT;

    while (1) {
        /*** Update time ***/
        uint32_t timestamp = TIM5->CNT << 16 | TIM4->CNT;
        data.timestamp = timestamp;

        /*** Command GSE from ethernet ***/
        // update internal states from new ethernet command
        if (newCommand) {
            newCommand = false;

            igniterState0       = command.igniter0Fire;
            igniterState1       = command.igniter1Fire;
            alarmState          = command.alarm;
            solenoidState0      = command.solenoidState0;
            solenoidState1      = command.solenoidState1;
            solenoidState2      = command.solenoidState2;
            solenoidState3      = command.solenoidState3;
            solenoidState4      = command.solenoidState4;
            solenoidState5      = command.solenoidState5;
            solenoidState6      = command.solenoidState6;
            solenoidState7      = command.solenoidState7;
            solenoidState8      = command.solenoidState8;
            solenoidState9      = command.solenoidState9;
            solenoidState10     = command.solenoidState10;
            solenoidState11     = command.solenoidState11;
        }

        // update internal states feedback
        data.igniterInternalState0      = igniterState0;
        data.igniterInternalState1      = igniterState1;     
        data.alarmInternalState         = alarmState;
        data.solenoidInternalState0     = solenoidState0;
        data.solenoidInternalState1     = solenoidState1;
        data.solenoidInternalState2     = solenoidState2;
        data.solenoidInternalState3     = solenoidState3;
        data.solenoidInternalState4     = solenoidState4;
        data.solenoidInternalState5     = solenoidState5;
        data.solenoidInternalState6     = solenoidState6;
        data.solenoidInternalState7     = solenoidState7;
        data.solenoidInternalState8     = solenoidState8;
        data.solenoidInternalState9     = solenoidState9;
        data.solenoidInternalState10    = solenoidState10;
        data.solenoidInternalState11    = solenoidState11;

        // switch solenoids
        HAL_GPIO_WritePin(SOLENOID0_EN_GPIO_Port,   SOLENOID0_EN_Pin,   (GPIO_PinState)solenoidState0);
        HAL_GPIO_WritePin(SOLENOID1_EN_GPIO_Port,   SOLENOID1_EN_Pin,   (GPIO_PinState)solenoidState1);
        HAL_GPIO_WritePin(SOLENOID2_EN_GPIO_Port,   SOLENOID2_EN_Pin,   (GPIO_PinState)solenoidState2);
        HAL_GPIO_WritePin(SOLENOID3_EN_GPIO_Port,   SOLENOID3_EN_Pin,   (GPIO_PinState)solenoidState3);
        HAL_GPIO_WritePin(SOLENOID4_EN_GPIO_Port,   SOLENOID4_EN_Pin,   (GPIO_PinState)solenoidState4);
        HAL_GPIO_WritePin(SOLENOID5_EN_GPIO_Port,   SOLENOID5_EN_Pin,   (GPIO_PinState)solenoidState5);
        HAL_GPIO_WritePin(SOLENOID6_EN_GPIO_Port,   SOLENOID6_EN_Pin,   (GPIO_PinState)solenoidState6);
        HAL_GPIO_WritePin(SOLENOID7_EN_GPIO_Port,   SOLENOID7_EN_Pin,   (GPIO_PinState)solenoidState7);
        HAL_GPIO_WritePin(SOLENOID8_EN_GPIO_Port,   SOLENOID8_EN_Pin,   (GPIO_PinState)solenoidState8);
        HAL_GPIO_WritePin(SOLENOID9_EN_GPIO_Port,   SOLENOID9_EN_Pin,   (GPIO_PinState)solenoidState9);
        HAL_GPIO_WritePin(SOLENOID10_EN_GPIO_Port,  SOLENOID10_EN_Pin,  (GPIO_PinState)solenoidState10);
        HAL_GPIO_WritePin(SOLENOID11_EN_GPIO_Port,  SOLENOID11_EN_Pin,  (GPIO_PinState)solenoidState11);
        
        // igniter read what has continuity/power
        data.igniterArmed       =  (bool)HAL_GPIO_ReadPin(ARMED_GPIO_Port, ARMED_Pin);
        data.igniter0Continuity = !(bool)HAL_GPIO_ReadPin(EMATCH0_CONT_GPIO_Port, EMATCH0_CONT_Pin);
        data.igniter1Continuity = !(bool)HAL_GPIO_ReadPin(EMATCH1_CONT_GPIO_Port, EMATCH1_CONT_Pin);

        // fire ignitors
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


        /*** Read sensors ***/
        // All sensing pins are preconfigured to always send data, no matter if it's actually used during a test/launch

        // STM32 ADC operations
        Stm32AdcData rawData = {0};
        for (int i = 0; i < 14; i++) {
            HAL_ADC_Start(&hadc1);
            HAL_ADC_PollForConversion(&hadc1, 10);
            uint32_t data = HAL_ADC_GetValue(&hadc1);
            *(((uint32_t *)&rawData) + i) += data;
        }

        // update transmit data struct
        data.supplyVoltage0     = 0.0062f * (float)rawData.pwr0 + 0.435f;
        data.supplyVoltage1     = 0.0062f * (float)rawData.pwr1 + 0.435f;
        data.solenoidCurrent0   = 0.000817f * (float)rawData.s0;
        data.solenoidCurrent1   = 0.000817f * (float)rawData.s1;
        data.solenoidCurrent2   = 0.000817f * (float)rawData.s0;
        data.solenoidCurrent3   = 0.000817f * (float)rawData.s1;
        data.solenoidCurrent4   = 0.000817f * (float)rawData.s0;
        data.solenoidCurrent5   = 0.000817f * (float)rawData.s1;
        data.solenoidCurrent6   = 0.000817f * (float)rawData.s0;
        data.solenoidCurrent7   = 0.000817f * (float)rawData.s1;
        data.solenoidCurrent8   = 0.000817f * (float)rawData.s0;
        data.solenoidCurrent9   = 0.000817f * (float)rawData.s1;
        data.solenoidCurrent10  = 0.000817f * (float)rawData.s0;
        data.solenoidCurrent11  = 0.000817f * (float)rawData.s1;
        
        // read thermocouples
        tcData = tc0.Read();
        if (tcData.valid) {
            data.temperature0 = tcData.tcTemperature;
        }
        tcData = tc1.Read();
        if (tcData.valid) {
            data.temperature1 = tcData.tcTemperature;
        }
        tcData = tc2.Read();
        if (tcData.valid) {
            data.temperature2 = tcData.tcTemperature;
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

// function is defined as a __weak function inside the HAL driver files
// overwritten version for our use acase
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    uint32_t crc = Crc32(commandBuffer, sizeof(GseCommand) - 4);
    if (crc == ((GseCommand *)commandBuffer)->crc) {
        memcpy((uint8_t *)&command, commandBuffer, sizeof(GseCommand));
        newCommand = true;
    }
    HAL_UART_Receive_IT(huart, commandBuffer, sizeof(GseCommand));
}