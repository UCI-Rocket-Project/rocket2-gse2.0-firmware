#pragma once

#include "main.h"

// UART/DMA simulator/mock implementation used by UartCommandReceiver. The tests simulate DMA progress
// and pending transfer-complete interrupts independently of task execution.
struct DMA_HandleTypeDef {
    uint32_t remaining = 0;
    bool transferComplete = false;
    bool halfInterruptEnabled = true;
};

struct UART_HandleTypeDef {
    DMA_HandleTypeDef* hdmarx;
    uint32_t gState = 0;
    uint32_t RxState = 0;
};

constexpr uint32_t DMA_IT_HT = 1;

extern "C" {
HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef* uart);
HAL_StatusTypeDef HAL_UART_Receive_DMA(UART_HandleTypeDef* uart, uint8_t* data, uint16_t size);
uint32_t __get_PRIMASK();
void __disable_irq();
void __set_PRIMASK(uint32_t primask);
uint32_t MockDmaCounter(DMA_HandleTypeDef* dma);
void MockClearUartErrors(UART_HandleTypeDef* uart);
}

#define __HAL_DMA_GET_COUNTER(dma) MockDmaCounter(dma)
#define __HAL_DMA_GET_TC_FLAG_INDEX(dma) 1
#define __HAL_DMA_GET_FLAG(dma, flag) ((dma)->transferComplete)
#define __HAL_DMA_DISABLE_IT(dma, interrupt) ((dma)->halfInterruptEnabled = false)
#define __HAL_UART_CLEAR_OREFLAG(uart) MockClearUartErrors(uart)
