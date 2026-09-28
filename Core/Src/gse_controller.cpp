#include "gse_controller.h"
#include "crc.h"

// Define external HAL handles
extern ADC_HandleTypeDef hadc1;
extern SPI_HandleTypeDef hspi3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim5;
extern UART_HandleTypeDef huart3;
extern I2C_HandleTypeDef hi2c1;

// Allocate memory for the static pointer
GseController* GseController::instance = nullptr;

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

    // tc0.Init();
    // tc1.Init();
    // tc2.Init();

    AdcMax11614i2c::Config adcConfig;
    external_adc.Init(adcConfig);

    HAL_TIM_Base_Start(&htim4);
    HAL_TIM_Base_Start(&htim5);

    HAL_UART_Receive_IT(&huart3, commandBuffer, sizeof(GseCommand));

    // --- Task ID registration ---
    // Note the number of dependencies for each task
    // Command processing task
    cmdTaskId         = *scheduler.AddTask(ProcessCommandsTask, 0);

    // Hardware actuation
    solenoidTaskId    = *scheduler.AddTask(SwitchSolenoidsTask, 1);
    igniterTaskId     = *scheduler.AddTask(FireIgnitersTask, 1);
    alarmTaskId       = *scheduler.AddTask(SetAlarmTask, 1);

    // Sensor reading, requires actuation first
    internalAdcTaskId = *scheduler.AddTask(ReadInternalAdcTask, 3);
    initExtAdcTaskId  = *scheduler.AddTask(InitiateExternalAdcReadTask, 3);
    tcTaskId          = *scheduler.AddTask(ReadThermocouplesTask, 3);

    // Special task called by interrupt after external ADC reads data async
    fetchExtAdcTaskId = *scheduler.AddTask(FetchExternalAdcTask, 0); 

    // Ethernet task always sends data even if sensors didn't all read properly
    ethTaskId         = *scheduler.AddTask(TransmitEthernetTask, 0);

    // On startup, schedule all tasks to run immediately
    uint32_t startTime = GetCurrentTime();
    scheduler.ScheduleTask(cmdTaskId, startTime);
    scheduler.ScheduleTask(solenoidTaskId, startTime);
    scheduler.ScheduleTask(igniterTaskId, startTime);
    scheduler.ScheduleTask(alarmTaskId, startTime);
    scheduler.ScheduleTask(internalAdcTaskId, startTime);
    scheduler.ScheduleTask(initExtAdcTaskId, startTime);
    scheduler.ScheduleTask(tcTaskId, startTime);
    scheduler.ScheduleTask(ethTaskId, startTime);

    // Note: fetchExtAdcTaskId is intentionally not scheduled here; it waits for the I2C interrupt
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

void GseController::OnI2cRx(I2C_HandleTypeDef *hi2c) {
    if (hi2c->Instance == I2C1) {
        scheduler.ScheduleTask(fetchExtAdcTaskId, GetCurrentTime());
    }
}

// --- Static task callbacks ---
void GseController::ProcessCommandsTask() {
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

    nextRun += 100;
    instance->scheduler.ScheduleTask(instance->cmdTaskId, nextRun);
}

void GseController::SwitchSolenoidsTask() {
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

    // Notify sensor tasks that solenoids have been switched (dependency 0)
    instance->scheduler.SetDependency(instance->internalAdcTaskId, 0, true);
    instance->scheduler.SetDependency(instance->initExtAdcTaskId, 0, true);
    instance->scheduler.SetDependency(instance->tcTaskId, 0, true);

    nextRun += 100;
    instance->scheduler.ScheduleTask(instance->solenoidTaskId, nextRun);
}

void GseController::FireIgnitersTask() {
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

    // Notify sensor tasks that igniters have been fired (dependency 1)
    instance->scheduler.SetDependency(instance->internalAdcTaskId, 1, true);
    instance->scheduler.SetDependency(instance->initExtAdcTaskId, 1, true);
    instance->scheduler.SetDependency(instance->tcTaskId, 1, true);

    nextRun += 100;
    instance->scheduler.ScheduleTask(instance->igniterTaskId, nextRun);
}

void GseController::SetAlarmTask() {
    static uint32_t nextRun = 0;
    if (nextRun == 0) nextRun = GetCurrentTime();

    // alarm
    HAL_GPIO_WritePin(ALARM_GPIO_Port, ALARM_Pin, (GPIO_PinState)instance->alarmState);

    // Trigger Sensor Tasks (Dep 2)
    instance->scheduler.SetDependency(instance->internalAdcTaskId, 2, true);
    instance->scheduler.SetDependency(instance->initExtAdcTaskId, 2, true);
    instance->scheduler.SetDependency(instance->tcTaskId, 2, true);

    nextRun += 100;
    instance->scheduler.ScheduleTask(instance->alarmTaskId, nextRun);
}

void GseController::ReadInternalAdcTask() {
    static uint32_t nextRun = 0;
    if (nextRun == 0) nextRun = GetCurrentTime();

    // STM32 internal ADC operations
    Stm32AdcData rawData = {0};
    for (int i = 0; i < 14; i++) {
        HAL_ADC_Start(&hadc1);
        HAL_ADC_PollForConversion(&hadc1, 10);
        uint32_t val = HAL_ADC_GetValue(&hadc1);
        *(((uint32_t *)&rawData) + i) += val;
    }

    // update transmit data struct
    instance->data.supplyVoltage0     = 0.0062f * (float)rawData.pwr0 + 0.435f;
    instance->data.supplyVoltage1     = 0.0062f * (float)rawData.pwr1 + 0.435f;
    instance->data.solenoidCurrent0   = 0.000817f * (float)rawData.s0;
    instance->data.solenoidCurrent1   = 0.000817f * (float)rawData.s1;
    instance->data.solenoidCurrent2   = 0.000817f * (float)rawData.s0;
    instance->data.solenoidCurrent3   = 0.000817f * (float)rawData.s1;
    instance->data.solenoidCurrent4   = 0.000817f * (float)rawData.s0;
    instance->data.solenoidCurrent5   = 0.000817f * (float)rawData.s1;
    instance->data.solenoidCurrent6   = 0.000817f * (float)rawData.s0;
    instance->data.solenoidCurrent7   = 0.000817f * (float)rawData.s1;
    instance->data.solenoidCurrent8   = 0.000817f * (float)rawData.s0;
    instance->data.solenoidCurrent9   = 0.000817f * (float)rawData.s1;
    instance->data.solenoidCurrent10  = 0.000817f * (float)rawData.s0;
    instance->data.solenoidCurrent11  = 0.000817f * (float)rawData.s1;

    nextRun += 100;
    instance->scheduler.ScheduleTask(instance->internalAdcTaskId, nextRun);
}

void GseController::InitiateExternalAdcReadTask() {
    static uint32_t nextRun = 0;
    if (nextRun == 0) nextRun = GetCurrentTime();

    // Begin async read of external ADC for all 8 channels (0xFF)
    instance->external_adc.StartReadAsync(0xFF);

    nextRun += 100;
    instance->scheduler.ScheduleTask(instance->initExtAdcTaskId, nextRun);
}

void GseController::ReadThermocouplesTask() {
    static uint32_t nextRun = 0;
    if (nextRun == 0) nextRun = GetCurrentTime();

    // Read thermocouples (commented out for now)
    // TcMax31855Spi::Data tcData;
    // tcData = instance->tc0.Read();
    // if (tcData.valid) {
    //     instance->data.temperature0 = tcData.tcTemperature;
    // }
    // tcData = instance->tc1.Read();
    // if (tcData.valid) {
    //     instance->data.temperature1 = tcData.tcTemperature;
    // }
    // tcData = instance->tc2.Read();
    // if (tcData.valid) {
    //     instance->data.temperature2 = tcData.tcTemperature;
    // }

    nextRun += 100;
    instance->scheduler.ScheduleTask(instance->tcTaskId, nextRun);
}

void GseController::FetchExternalAdcTask() {
    // Read external ADC if interrupt finished
    if (instance->external_adc.IsDataReady()) {
        auto optData = instance->external_adc.FetchData(); // Calling this resets driver to IDLE state
        
        // For overall data structure, only use if we got a successful read
        if (optData.has_value()) {
            auto externalADCData = optData.value();
            
            // For each channel, if we got data, convert from 0-4095 raw output to 0-5V digital voltage
            if (externalADCData.channelOutput0 != -1) {
                instance->data.pressure0 = (externalADCData.channelOutput0 / (float)AdcMax11614i2c::maxChannelOutput) * 5.0f;
            }
            if (externalADCData.channelOutput1 != -1) {
                instance->data.pressure1 = (externalADCData.channelOutput1 / (float)AdcMax11614i2c::maxChannelOutput) * 5.0f;
            }
            if (externalADCData.channelOutput2 != -1) {
                instance->data.pressure2 = (externalADCData.channelOutput2 / (float)AdcMax11614i2c::maxChannelOutput) * 5.0f;
            }
            if (externalADCData.channelOutput3 != -1) {
                instance->data.pressure3 = (externalADCData.channelOutput3 / (float)AdcMax11614i2c::maxChannelOutput) * 5.0f;
            }
            // Load cells
            if (externalADCData.channelOutput4 != -1) {
                instance->data.loadCellForce2 = (externalADCData.channelOutput4 / (float)AdcMax11614i2c::maxChannelOutput) * 5.0f;
            }
            if (externalADCData.channelOutput5 != -1) {
                instance->data.loadCellForce3 = (externalADCData.channelOutput5 / (float)AdcMax11614i2c::maxChannelOutput) * 5.0f;
            }
            if (externalADCData.channelOutput6 != -1) {
                instance->data.loadCellForce4 = (externalADCData.channelOutput6 / (float)AdcMax11614i2c::maxChannelOutput) * 5.0f;
            }
            if (externalADCData.channelOutput7 != -1) {
                instance->data.loadCellForce5 = (externalADCData.channelOutput7 / (float)AdcMax11614i2c::maxChannelOutput) * 5.0f;
            }
        }
    }
    // Note: Does NOT self-reschedule. This task remains dormant until the ADC read completes and the ISR
    // schedules it manually again
}

void GseController::TransmitEthernetTask() {
    static uint32_t nextRun = 0;
    if (nextRun == 0) nextRun = GetCurrentTime();

    // Ethernet - send freshest data regardless of dependencies
    instance->data.timestamp = GetCurrentTime(); 
    uint32_t crc = Crc32((uint8_t *)&instance->data, sizeof(GseData) - 4);
    instance->data.crc = crc;
    HAL_UART_Transmit(&huart3, (uint8_t *)&instance->data, sizeof(GseData), 100);

    nextRun += 100;
    instance->scheduler.ScheduleTask(instance->ethTaskId, nextRun);
}

// --- Global C callback routers ---
extern "C" {
    void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
        if (GseController::instance) {
            GseController::instance->OnUartRx(huart);
        }
    }

    void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c) {
        AdcMax11614i2c::HAL_RxCpltCallback(hi2c);
        if (GseController::instance) {
            GseController::instance->OnI2cRx(hi2c);
        }
    }

    void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c) {
        AdcMax11614i2c::HAL_TxCpltCallback(hi2c);
    }

    void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c) {
        AdcMax11614i2c::HAL_ErrorCallback(hi2c);
    }
}