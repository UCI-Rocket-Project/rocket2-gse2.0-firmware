#pragma once

#include "stm32f1xx_hal.h"
#include "gse_types.h"

// DMA controls the write index, while receiver is the reader of the buffer
class UartCommandReceiver {
  public:
    static constexpr uint32_t bufferSize = 1024;

    explicit UartCommandReceiver(UART_HandleTypeDef* uart) : uart(uart) {}

    bool Start();
    // Returns at most one validated command. On failure, leaves buffer unchanged
    bool ReadCommand(GseCommand& command);

    void OnRxComplete();
    
    void OnError();

  private:
    uint32_t WriteCount() const;

    UART_HandleTypeDef* uart;
    volatile uint8_t buffer[bufferSize] = {};
    volatile uint32_t dmaWraps = 0;
    volatile bool restartPending = true;
    uint32_t readCount = 0; // Monotonically increasing read counter
};
