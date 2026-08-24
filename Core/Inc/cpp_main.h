#ifndef CPP_MAIN_H
#define CPP_MAIN_H

#include "main.h"

#ifdef __cplusplus
extern "C" { // Disables C++ name mangling so C code can link to this
#endif

// Bridge from C startup code into your C++ application
void cpp_main(void);

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);

void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c);

void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c);

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c);

#ifdef __cplusplus
} // Closes extern "C" block
#endif

#endif // CPP_MAIN_H
