#include "gse_controller.h"
#include "crc.h"
#include <cstddef>

// Define external HAL handles
extern ADC_HandleTypeDef hadc1;
extern SPI_HandleTypeDef hspi3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim5;
extern UART_HandleTypeDef huart3;
extern I2C_HandleTypeDef hi2c1;

// Allocate memory for the static pointer
GseController* GseController::instance = nullptr;
size_t counter;
float timestamp;
float hz;

// --- Profiling global variables ---
float avgEthTaskTime = 0.0f;
float avgTestTaskTime = 0.0f;
float avgCrcTime = 0.0f;
float avgUartTime = 0.0f;
float avgCmdTaskTime = 0.0f;
float avgSolenoidTaskTime = 0.0f;
float avgIgniterTaskTime = 0.0f;
float avgAlarmTaskTime = 0.0f;
float avgInternalAdcTaskTime = 0.0f;
float avgInternalAdcInitTaskTime = 0.0f;
float avgInternalAdcFetchTaskTime = 0.0f;
float avgExternalAdcInitTaskTime = 0.0f;
float avgExternalAdcFetchTaskTime = 0.0f;
float avgThermocoupleTaskTime = 0.0f;

// --- Task run counters (inspectable in Live Watch to confirm loop execution) ---
volatile uint32_t cmdTaskRunCount = 0;
volatile uint32_t solenoidTaskRunCount = 0;
volatile uint32_t igniterTaskRunCount = 0;
volatile uint32_t alarmTaskRunCount = 0;
volatile uint32_t internalAdcTaskRunCount = 0;
volatile uint32_t tcTaskRunCount = 0;
volatile uint32_t ethTaskRunCount = 0;
volatile uint32_t ethTxRunCount = 0;
volatile uint32_t externalAdcFetchRunCount = 0;

// --- Cumulative task run times (in ms) for true average calculation ---
uint64_t totalEthTaskTime = 0;
uint64_t totalTestTaskTime = 0;
uint64_t totalCrcTime = 0;
uint64_t totalUartTime = 0;
uint64_t totalCmdTaskTime = 0;
uint64_t totalSolenoidTaskTime = 0;
uint64_t totalIgniterTaskTime = 0;
uint64_t totalAlarmTaskTime = 0;
uint64_t totalInternalAdcTaskTime = 0;
uint64_t totalThermocoupleTaskTime = 0;
uint64_t totalExternalAdcFetchTaskTime = 0;

/**
 * @brief Safely reads the 32-bit hardware timer, preventing rollover glitches.
 */
inline uint32_t GetCurrentTime() {
    uint32_t high1 = TIM5->CNT;
    uint32_t low = TIM4->CNT;
    uint32_t high2 = TIM5->CNT;
    if (high1 != high2) {
        low = TIM4->CNT;
    }
    return (high2 << 16) | low;
}

GseController::GseController()
    : external_adc(&hi2c1, EXT_ADC_SCL_GPIO_Port, EXT_ADC_SCL_Pin, EXT_ADC_SDA_GPIO_Port, EXT_ADC_SDA_Pin),
      tc0(&hspi3, TC0_CS_GPIO_Port, TC0_CS_Pin, 100),
      tc1(&hspi3, TC1_CS_GPIO_Port, TC1_CS_Pin, 100),
      tc2(&hspi3, TC2_CS_GPIO_Port, TC2_CS_Pin, 100) 
{
    instance = this; 
}

void GseController::Init() {
    HAL_GPIO_WritePin(ETH_RST_GPIO_Port, ETH_RST_Pin, GPIO_PIN_RESET);
    HAL_Delay(10); 
    HAL_GPIO_WritePin(ETH_RST_GPIO_Port, ETH_RST_Pin, GPIO_PIN_SET);
    HAL_Delay(250); 

    tc0.Init();
    tc1.Init();
    tc2.Init();

    AdcMax11614i2c::Config adcConfig;
    external_adc.Init(adcConfig);

    // Reconfigure ADC1 for continuous scan mode with circular DMA:
    // All 14 channels are converted continuously in hardware into adcDmaBuffer.
    hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
    hadc1.Init.ContinuousConvMode = ENABLE;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.NbrOfConversion = 14;
    if (HAL_ADC_Init(&hadc1) != HAL_OK) {
        Error_Handler();
    }
    HAL_ADCEx_Calibration_Start(&hadc1);

    HAL_TIM_Base_Start(&htim4);
    HAL_TIM_Base_Start(&htim5);

    HAL_UART_Receive_IT(&huart3, commandBuffer, sizeof(GseCommand));

    // --- Task ID registration ---
    // Note the number of dependencies for each task
    // Command processing task
    cmdTaskId             = *scheduler.AddTask(ProcessCommandsTask, 0);

    // Hardware actuation
    solenoidTaskId        = *scheduler.AddTask(SwitchSolenoidsTask, 1);
    igniterTaskId         = *scheduler.AddTask(FireIgnitersTask, 1);
    alarmTaskId           = *scheduler.AddTask(SetAlarmTask, 1);

    // Sensor reading, requires actuation first
    initInternalAdcTaskId = *scheduler.AddTask(InitiateInternalAdcTask, 0);
    internalAdcTaskId     = *scheduler.AddTask(ReadInternalAdcTask, 0);
    initExtAdcTaskId      = *scheduler.AddTask(InitiateExternalAdcReadTask, 0);
    tcTaskId              = *scheduler.AddTask(ReadThermocouplesTask, 0);

    // Special task called by interrupt after external ADC reads data async
    fetchExtAdcTaskId     = *scheduler.AddTask(FetchExternalAdcTask, 0); 

    // Ethernet task always sends data even if sensors didn't all read properly
    ethTaskId             = *scheduler.AddTask(TransmitEthernetTask, 0);

    // Test task that counts up every time it's scheduled
    testTaskId            = *scheduler.AddTask(TestCounterTask, 0);

    // On startup, schedule all tasks to run immediately
    uint32_t startTime = GetCurrentTime();
    scheduler.ScheduleTask(testTaskId, startTime);
    scheduler.ScheduleTask(cmdTaskId, startTime);
    scheduler.ScheduleTask(solenoidTaskId, startTime);
    scheduler.ScheduleTask(igniterTaskId, startTime);
    scheduler.ScheduleTask(alarmTaskId, startTime);
    scheduler.ScheduleTask(initInternalAdcTaskId, startTime);
    scheduler.ScheduleTask(internalAdcTaskId, startTime);
    // WARNING currently breaks other tasks
    scheduler.ScheduleTask(initExtAdcTaskId, startTime);
    scheduler.ScheduleTask(tcTaskId, startTime);
    scheduler.ScheduleTask(ethTaskId, startTime);
}

void GseController::Run() {
    while (1) {
        scheduler.Run(GetCurrentTime());
    }
}

// Hardware interrupt routers, internal logic to attach to extern C block
void GseController::OnUartRx(UART_HandleTypeDef *huart) {
    uint32_t crc = Crc32(commandBuffer, sizeof(GseCommand) - 4);
    if (crc == ((GseCommand *)commandBuffer)->crc) {
        memcpy((uint8_t *)&command, commandBuffer, sizeof(GseCommand));
        newCommand = true;
    }
    HAL_UART_Receive_IT(huart, commandBuffer, sizeof(GseCommand));
}

void GseController::OnUartTx(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART3) {
        // Callback router for USART3 TX DMA completion
    }
}

void GseController::OnUartError(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART3) {
        // Clear error flags and re-arm command reception
        __HAL_UART_CLEAR_PEFLAG(huart);
        __HAL_UART_CLEAR_FEFLAG(huart);
        __HAL_UART_CLEAR_NEFLAG(huart);
        __HAL_UART_CLEAR_OREFLAG(huart);
        huart->ErrorCode = HAL_UART_ERROR_NONE;
        huart->RxState = HAL_UART_STATE_READY;
        huart->gState = HAL_UART_STATE_READY;
        HAL_UART_Receive_IT(huart, commandBuffer, sizeof(GseCommand));
    }
}

void GseController::OnI2cRx(I2C_HandleTypeDef *hi2c) {
    if (hi2c->Instance == I2C1) {
        scheduler.ScheduleTask(fetchExtAdcTaskId, GetCurrentTime());
    }
}

void GseController::OnAdcDmaComplete(ADC_HandleTypeDef *hadc) {
    // Unused in circular DMA mode (DMA continuously writes to adcDmaBuffer)
}

// --- Static task callbacks ---
void GseController::ProcessCommandsTask() {
    cmdTaskRunCount++;
    uint32_t taskStartTime = GetCurrentTime();

    static uint32_t nextRun = 0;
    if (nextRun == 0) nextRun = GetCurrentTime(); // Init on first run

    // Update internal states from new ethernet command
    if (instance->newCommand) {
        instance->newCommand = false;
        instance->igniterState0       = instance->command.igniter0Fire;
        instance->igniterState1       = instance->command.igniter1Fire;
        instance->alarmState          = instance->command.alarm;
        instance->solenoidState0      = instance->command.solenoidState0;
        instance->solenoidState1      = instance->command.solenoidState1;
        instance->solenoidState2      = instance->command.solenoidState2;
        instance->solenoidState3      = instance->command.solenoidState3;
        instance->solenoidState4      = instance->command.solenoidState4;
        instance->solenoidState5      = instance->command.solenoidState5;
        instance->solenoidState6      = instance->command.solenoidState6;
        instance->solenoidState7      = instance->command.solenoidState7;
        instance->solenoidState8      = instance->command.solenoidState8;
        instance->solenoidState9      = instance->command.solenoidState9;
        instance->solenoidState10     = instance->command.solenoidState10;
        instance->solenoidState11     = instance->command.solenoidState11;
    }

    // update internal states feedback
    instance->data.igniterInternalState0      = instance->igniterState0;
    instance->data.igniterInternalState1      = instance->igniterState1;     
    instance->data.alarmInternalState         = instance->alarmState;
    instance->data.solenoidInternalState0     = instance->solenoidState0;
    instance->data.solenoidInternalState1     = instance->solenoidState1;
    instance->data.solenoidInternalState2     = instance->solenoidState2;
    instance->data.solenoidInternalState3     = instance->solenoidState3;
    instance->data.solenoidInternalState4     = instance->solenoidState4;
    instance->data.solenoidInternalState5     = instance->solenoidState5;
    instance->data.solenoidInternalState6     = instance->solenoidState6;
    instance->data.solenoidInternalState7     = instance->solenoidState7;
    instance->data.solenoidInternalState8     = instance->solenoidState8;
    instance->data.solenoidInternalState9     = instance->solenoidState9;
    instance->data.solenoidInternalState10    = instance->solenoidState10;
    instance->data.solenoidInternalState11    = instance->solenoidState11;

    // Trigger downstream Actuation Tasks
    instance->scheduler.SetDependency(instance->solenoidTaskId, 0, true);
    instance->scheduler.SetDependency(instance->igniterTaskId, 0, true);
    instance->scheduler.SetDependency(instance->alarmTaskId, 0, true);

    nextRun += 10;
    instance->scheduler.ScheduleTask(instance->cmdTaskId, nextRun);

    uint32_t taskDelta = GetCurrentTime() - taskStartTime;
    totalCmdTaskTime += taskDelta;
    if (cmdTaskRunCount > 0) {
        avgCmdTaskTime = (float)totalCmdTaskTime / (float)cmdTaskRunCount;
    }
}

void GseController::SwitchSolenoidsTask() {
    solenoidTaskRunCount++;
    uint32_t taskStartTime = GetCurrentTime();

    static uint32_t nextRun = 0;
    if (nextRun == 0) nextRun = GetCurrentTime();

    // Switch solenoids
    HAL_GPIO_WritePin(SOLENOID0_EN_GPIO_Port,   SOLENOID0_EN_Pin,   (GPIO_PinState)instance->solenoidState0);
    HAL_GPIO_WritePin(SOLENOID1_EN_GPIO_Port,   SOLENOID1_EN_Pin,   (GPIO_PinState)instance->solenoidState1);
    HAL_GPIO_WritePin(SOLENOID2_EN_GPIO_Port,   SOLENOID2_EN_Pin,   (GPIO_PinState)instance->solenoidState2);
    HAL_GPIO_WritePin(SOLENOID3_EN_GPIO_Port,   SOLENOID3_EN_Pin,   (GPIO_PinState)instance->solenoidState3);
    HAL_GPIO_WritePin(SOLENOID4_EN_GPIO_Port,   SOLENOID4_EN_Pin,   (GPIO_PinState)instance->solenoidState4);
    HAL_GPIO_WritePin(SOLENOID5_EN_GPIO_Port,   SOLENOID5_EN_Pin,   (GPIO_PinState)instance->solenoidState5);
    HAL_GPIO_WritePin(SOLENOID6_EN_GPIO_Port,   SOLENOID6_EN_Pin,   (GPIO_PinState)instance->solenoidState6);
    HAL_GPIO_WritePin(SOLENOID7_EN_GPIO_Port,   SOLENOID7_EN_Pin,   (GPIO_PinState)instance->solenoidState7);
    HAL_GPIO_WritePin(SOLENOID8_EN_GPIO_Port,   SOLENOID8_EN_Pin,   (GPIO_PinState)instance->solenoidState8);
    HAL_GPIO_WritePin(SOLENOID9_EN_GPIO_Port,   SOLENOID9_EN_Pin,   (GPIO_PinState)instance->solenoidState9);
    HAL_GPIO_WritePin(SOLENOID10_EN_GPIO_Port,  SOLENOID10_EN_Pin,  (GPIO_PinState)instance->solenoidState10);
    HAL_GPIO_WritePin(SOLENOID11_EN_GPIO_Port,  SOLENOID11_EN_Pin,  (GPIO_PinState)instance->solenoidState11);

    nextRun += 10;
    instance->scheduler.ScheduleTask(instance->solenoidTaskId, nextRun);

    uint32_t taskDelta = GetCurrentTime() - taskStartTime;
    totalSolenoidTaskTime += taskDelta;
    if (solenoidTaskRunCount > 0) {
        avgSolenoidTaskTime = (float)totalSolenoidTaskTime / (float)solenoidTaskRunCount;
    }
}

void GseController::FireIgnitersTask() {
    igniterTaskRunCount++;
    uint32_t taskStartTime = GetCurrentTime();

    static uint32_t nextRun = 0;
    if (nextRun == 0) nextRun = GetCurrentTime();

    // Igniter read what has continuity/power
    instance->data.igniterArmed       =  (bool)HAL_GPIO_ReadPin(ARMED_GPIO_Port, ARMED_Pin);
    instance->data.igniter0Continuity = !(bool)HAL_GPIO_ReadPin(EMATCH0_CONT_GPIO_Port, EMATCH0_CONT_Pin);
    instance->data.igniter1Continuity = !(bool)HAL_GPIO_ReadPin(EMATCH1_CONT_GPIO_Port, EMATCH1_CONT_Pin);

    // Fire igniters
    if (instance->data.igniterArmed && instance->igniterState0) {
        HAL_GPIO_WritePin(EMATCH0_FIRE_GPIO_Port, EMATCH0_FIRE_Pin, GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(EMATCH0_FIRE_GPIO_Port, EMATCH0_FIRE_Pin, GPIO_PIN_RESET);
    }
    if (instance->data.igniterArmed && instance->igniterState1) {
        HAL_GPIO_WritePin(EMATCH1_FIRE_GPIO_Port, EMATCH1_FIRE_Pin, GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(EMATCH1_FIRE_GPIO_Port, EMATCH1_FIRE_Pin, GPIO_PIN_RESET);
    }

    nextRun += 10;
    instance->scheduler.ScheduleTask(instance->igniterTaskId, nextRun);

    uint32_t taskDelta = GetCurrentTime() - taskStartTime;
    totalIgniterTaskTime += taskDelta;
    if (igniterTaskRunCount > 0) {
        avgIgniterTaskTime = (float)totalIgniterTaskTime / (float)igniterTaskRunCount;
    }
}

void GseController::SetAlarmTask() {
    alarmTaskRunCount++;
    uint32_t taskStartTime = GetCurrentTime();

    static uint32_t nextRun = 0;
    if (nextRun == 0) nextRun = GetCurrentTime();

    // alarm
    HAL_GPIO_WritePin(ALARM_GPIO_Port, ALARM_Pin, (GPIO_PinState)instance->alarmState);

    nextRun += 10;
    instance->scheduler.ScheduleTask(instance->alarmTaskId, nextRun);

    uint32_t taskDelta = GetCurrentTime() - taskStartTime;
    totalAlarmTaskTime += taskDelta;
    if (alarmTaskRunCount > 0) {
        avgAlarmTaskTime = (float)totalAlarmTaskTime / (float)alarmTaskRunCount;
    }
}

void GseController::InitiateInternalAdcTask() {
    uint32_t taskStartTime = GetCurrentTime();

    // Start continuous circular DMA transfer for all 14 ADC channels (32-bit words).
    // In circular mode, hardware continuously streams conversions into adcDmaBuffer
    // with zero CPU intervention.
    HAL_ADC_Start_DMA(&hadc1, instance->adcDmaBuffer, 14);

    // Note: Does NOT self-reschedule. This one-time task completes and remains dormant.
    // ReadInternalAdcTask periodically reads the latest values from adcDmaBuffer.

    uint32_t taskDelta = GetCurrentTime() - taskStartTime;
    avgInternalAdcInitTaskTime = (avgInternalAdcInitTaskTime == 0.0f) ? (float)taskDelta : (avgInternalAdcInitTaskTime * 0.9f) + ((float)taskDelta * 0.1f);
}

void GseController::ReadInternalAdcTask() {
    internalAdcTaskRunCount++;
    uint32_t taskStartTime = GetCurrentTime();

    static uint32_t nextRun = 0;
    if (nextRun == 0) nextRun = GetCurrentTime();

    // DMA continuously updates adcDmaBuffer with all 14 channels in rank order.
    // Map the buffer through the Stm32AdcData struct for named access.
    Stm32AdcData rawData;
    memcpy(&rawData, instance->adcDmaBuffer, sizeof(Stm32AdcData));

    // Update transmit data struct
    instance->data.supplyVoltage0     = 0.0062f * (float)rawData.pwr0 + 0.435f;
    instance->data.supplyVoltage1     = 0.0062f * (float)rawData.pwr1 + 0.435f;
    instance->data.solenoidCurrent0   = 0.000817f * (float)rawData.s0;
    instance->data.solenoidCurrent1   = 0.000817f * (float)rawData.s1;
    instance->data.solenoidCurrent2   = 0.000817f * (float)rawData.s2;
    instance->data.solenoidCurrent3   = 0.000817f * (float)rawData.s3;
    instance->data.solenoidCurrent4   = 0.000817f * (float)rawData.s4;
    instance->data.solenoidCurrent5   = 0.000817f * (float)rawData.s5;
    instance->data.solenoidCurrent6   = 0.000817f * (float)rawData.s6;
    instance->data.solenoidCurrent7   = 0.000817f * (float)rawData.s7;
    instance->data.solenoidCurrent8   = 0.000817f * (float)rawData.s8;
    instance->data.solenoidCurrent9   = 0.000817f * (float)rawData.s9;
    instance->data.solenoidCurrent10  = 0.000817f * (float)rawData.s10;
    instance->data.solenoidCurrent11  = 0.000817f * (float)rawData.s11;

    nextRun += 10;
    instance->scheduler.ScheduleTask(instance->internalAdcTaskId, nextRun);

    uint32_t taskDelta = GetCurrentTime() - taskStartTime;
    totalInternalAdcTaskTime += taskDelta;
    if (internalAdcTaskRunCount > 0) {
        avgInternalAdcTaskTime = (float)totalInternalAdcTaskTime / (float)internalAdcTaskRunCount;
        avgInternalAdcFetchTaskTime = avgInternalAdcTaskTime;
    }
}

void GseController::InitiateExternalAdcReadTask() {
    uint32_t taskStartTime = GetCurrentTime();

    static uint32_t nextRun = 0;
    if (nextRun == 0) nextRun = GetCurrentTime();

    // Begin async read of external ADC for all 8 channels (0xFF) which is default config
    bool result = instance->external_adc.StartReadAsync();
    instance->data.solenoidInternalState0 = result;

    // Should only reschedule this task if the channel selection changes (e.g. from GUI inputs)

    uint32_t taskDelta = GetCurrentTime() - taskStartTime;
    avgExternalAdcInitTaskTime = (avgExternalAdcInitTaskTime == 0.0f) ? (float)taskDelta : (avgExternalAdcInitTaskTime * 0.9f) + ((float)taskDelta * 0.1f);
}

void GseController::ReadThermocouplesTask() {
    tcTaskRunCount++;
    uint32_t taskStartTime = GetCurrentTime();

    static uint32_t nextRun = 0;
    if (nextRun == 0) nextRun = GetCurrentTime();

    // Read thermocouples
    TcMax31855Spi::Data tcData;
    tcData = instance->tc0.Read();
    if (tcData.valid) {
        instance->data.temperature0 = tcData.tcTemperature;
    }
    tcData = instance->tc1.Read();
    if (tcData.valid) {
        instance->data.temperature1 = tcData.tcTemperature;
    }
    tcData = instance->tc2.Read();
    if (tcData.valid) {
        instance->data.temperature2 = tcData.tcTemperature;
    }

    nextRun += 10;
    instance->scheduler.ScheduleTask(instance->tcTaskId, nextRun);

    uint32_t taskDelta = GetCurrentTime() - taskStartTime;
    totalThermocoupleTaskTime += taskDelta;
    if (tcTaskRunCount > 0) {
        avgThermocoupleTaskTime = (float)totalThermocoupleTaskTime / (float)tcTaskRunCount;
    }
}

void GseController::FetchExternalAdcTask() {
    uint32_t taskStartTime = GetCurrentTime();

    // Read external ADC if interrupt finished
    if (instance->external_adc.IsDataReady()) {
        auto optData = instance->external_adc.FetchData(); // Calling this resets driver to IDLE state
        
        // For overall data structure, only use if we got a successful read
        if (optData.has_value()) {
            auto externalADCData = optData.value();
            
            // Pass along valid channel bitmask
            instance->data.externalAdcValidBitmask = externalADCData.channelValidBitmask;

            // For each channel, if we got valid data, convert from 0-4095 raw output to 0-5V digital voltage
            if (externalADCData.channelValidBitmask & (1 << 0)) {
                instance->data.pressure0 = (externalADCData.channelOutput0 / (float)AdcMax11614i2c::maxChannelOutput) * 5.0f;
            }
            if (externalADCData.channelValidBitmask & (1 << 1)) {
                instance->data.pressure1 = (externalADCData.channelOutput1 / (float)AdcMax11614i2c::maxChannelOutput) * 5.0f;
            }
            if (externalADCData.channelValidBitmask & (1 << 2)) {
                instance->data.pressure2 = (externalADCData.channelOutput2 / (float)AdcMax11614i2c::maxChannelOutput) * 5.0f;
            }
            if (externalADCData.channelValidBitmask & (1 << 3)) {
                instance->data.pressure3 = (externalADCData.channelOutput3 / (float)AdcMax11614i2c::maxChannelOutput) * 5.0f;
            }
            // Load cells
            if (externalADCData.channelValidBitmask & (1 << 4)) {
                instance->data.loadCellForce2 = (externalADCData.channelOutput4 / (float)AdcMax11614i2c::maxChannelOutput) * 5.0f;
            }
            if (externalADCData.channelValidBitmask & (1 << 5)) {
                instance->data.loadCellForce3 = (externalADCData.channelOutput5 / (float)AdcMax11614i2c::maxChannelOutput) * 5.0f;
            }
            if (externalADCData.channelValidBitmask & (1 << 6)) {
                instance->data.loadCellForce4 = (externalADCData.channelOutput6 / (float)AdcMax11614i2c::maxChannelOutput) * 5.0f;
            }
            if (externalADCData.channelValidBitmask & (1 << 7)) {
                instance->data.loadCellForce5 = (externalADCData.channelOutput7 / (float)AdcMax11614i2c::maxChannelOutput) * 5.0f;
            }
        }
    }
    // Note: Does NOT self-reschedule. This task remains dormant until the ADC read completes and the ISR
    // schedules it manually again

    externalAdcFetchRunCount++;
    uint32_t taskDelta = GetCurrentTime() - taskStartTime;
    totalExternalAdcFetchTaskTime += taskDelta;
    if (externalAdcFetchRunCount > 0) {
        avgExternalAdcFetchTaskTime = (float)totalExternalAdcFetchTaskTime / (float)externalAdcFetchRunCount;
    }
}

void GseController::TransmitEthernetTask() {
    ethTaskRunCount++;
    uint32_t taskStartTime = GetCurrentTime();

    static uint32_t nextRun = 0;
    if (nextRun == 0) nextRun = GetCurrentTime();

    static uint32_t busyTicks = 0;

    // Only start a new transmission if the previous DMA transfer has completed
    if (huart3.gState == HAL_UART_STATE_READY) {
        busyTicks = 0;

        // Ethernet - send freshest data regardless of dependencies
        instance->data.timestamp = GetCurrentTime(); 

        // Profile CRC execution
        uint32_t crcStartTime = GetCurrentTime();
        uint32_t crc = Crc32((uint8_t *)&instance->data, sizeof(GseData) - 4);
        uint32_t crcDelta = GetCurrentTime() - crcStartTime;

        instance->data.crc = crc;
        // Set last two bytes to 0x67
        instance->data.crc = 0x67676767;

        // Snapshot to separate TX buffer to prevent data tearing and CRC mismatch during background DMA
        memcpy(&instance->ethTxBuffer, &instance->data, sizeof(GseData));

        // Profile UART Transmit DMA initiation
        uint32_t uartStartTime = GetCurrentTime();
        if (HAL_UART_Transmit_DMA(&huart3, (uint8_t *)&instance->ethTxBuffer, sizeof(GseData)) != HAL_OK) {
            HAL_UART_AbortTransmit(&huart3);
        }
        uint32_t uartDelta = GetCurrentTime() - uartStartTime;

        ethTxRunCount++;
        totalCrcTime += crcDelta;
        avgCrcTime = (float)totalCrcTime / (float)ethTxRunCount;
        totalUartTime += uartDelta;
        avgUartTime = (float)totalUartTime / (float)ethTxRunCount;
    } else {
        // Watchdog: If UART remains busy for >50 ms (expected ~1.4 ms), abort stuck transfer to recover
        busyTicks++;
        if (busyTicks >= 5) {
            HAL_UART_AbortTransmit(&huart3);
            busyTicks = 0;
        }
    }

    nextRun += 10;
    instance->scheduler.ScheduleTask(instance->ethTaskId, nextRun);

    // Profile total task execution
    uint32_t taskDelta = GetCurrentTime() - taskStartTime;
    totalEthTaskTime += taskDelta;
    if (ethTaskRunCount > 0) {
        avgEthTaskTime = (float)totalEthTaskTime / (float)ethTaskRunCount;
    }
}

void GseController::TestCounterTask() {
    uint32_t taskStartTime = GetCurrentTime();
    
    static uint32_t nextRun = 0;
    if (nextRun == 0) nextRun = GetCurrentTime(); // Ensure timer aligns with boot time

    ++counter;
    timestamp = (float)GetCurrentTime();
    
    // Timestamp is in milliseconds (guarded against division by zero)
    if (timestamp > 0.0f) {
        hz = (float)counter / (timestamp / 1000.0f);
    }

    nextRun += 10;
    instance->scheduler.ScheduleTask(instance->testTaskId, nextRun);

    // Profile total task execution
    uint32_t taskDelta = GetCurrentTime() - taskStartTime;
    totalTestTaskTime += taskDelta;
    if (counter > 0) {
        avgTestTaskTime = (float)totalTestTaskTime / (float)counter;
    }
}

// --- Global C callback routers ---
extern "C" {
    void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
        if (GseController::instance) {
            GseController::instance->OnUartRx(huart);
        }
    }

    void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
        if (GseController::instance) {
            GseController::instance->OnUartTx(huart);
        }
    }

    void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
        if (GseController::instance) {
            GseController::instance->OnUartError(huart);
        }
    }

    void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c) {
        AdcMax11614i2c::HAL_RxCpltCallback(hi2c);
        if (GseController::instance) {
            GseController::instance->OnI2cRx(hi2c);
        }
    }

    void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c) {
        AdcMax11614i2c::HAL_ErrorCallback(hi2c);
    }

    void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
        if (GseController::instance) {
            GseController::instance->OnAdcDmaComplete(hadc);
        }
    }
}