#pragma once

#include "main.h"
#include "gse_types.h"
#include "task_scheduler.h"
#include "adc_max11614_i2c.h"
#include "tc_max31855_spi.h"

// The main state/task owner, responsible for containing all GSE tasks through the task scheduler
class GseController {
  public:
    // The globally accessible instance to access the controller at, making it compatible with
    // interrupts from the STM32's C-based HAL
    static GseController* instance;

    GseController();
    void Init();
    void Run();

    // Hardware interrupt routers, internal logic to attach to extern C block
    void OnUartRx(UART_HandleTypeDef *huart);
    void OnI2cRx(I2C_HandleTypeDef *hi2c);

  private:
    // Internal state variables
    bool newCommand = false;
    GseCommand command;
    uint8_t commandBuffer[sizeof(GseCommand)];
    GseData data;

    bool solenoidState0 = 0, solenoidState1 = 0, solenoidState2 = 0, solenoidState3 = 0;
    bool solenoidState4 = 0, solenoidState5 = 0, solenoidState6 = 0, solenoidState7 = 0;
    bool solenoidState8 = 0, solenoidState9 = 0, solenoidState10 = 0, solenoidState11 = 0;
    bool igniterState0 = 0, igniterState1 = 0;
    bool alarmState = 0;

    // Hardware object handles
    AdcMax11614i2c external_adc;
    TcMax31855Spi tc0;
    TcMax31855Spi tc1;
    TcMax31855Spi tc2;

    // Task scheduler setup
    TaskScheduler<10, 3> scheduler;
    size_t cmdTaskId, solenoidTaskId, igniterTaskId, alarmTaskId;
    size_t internalAdcTaskId, initExtAdcTaskId, tcTaskId, fetchExtAdcTaskId, ethTaskId;
    size_t testTaskId;

    // Task callbacks
    static void ProcessCommandsTask();
    static void SwitchSolenoidsTask();
    static void FireIgnitersTask();
    static void SetAlarmTask();
    static void ReadInternalAdcTask();
    static void InitiateExternalAdcReadTask();
    static void ReadThermocouplesTask();
    static void FetchExternalAdcTask();
    static void TransmitEthernetTask();
    static void TestCounterTask();
};