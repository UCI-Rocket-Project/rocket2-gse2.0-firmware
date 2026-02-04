#include "cpp_main.h"
#include "main.h" // Access to HAL handles (huart1, etc.)

#include <cmath>

using namespace std; 

// Define transmittable datastructures
#pragma pack(push, 1)
struct Data {
  uint32_t data; 
};

#pragma pack(pop)

// Add extern defined handles to peripheral interfaces defined in main.c
extern ADC_HandleTypeDef hadc1;

extern SPI_HandleTypeDef hspi3;

extern UART_HandleTypeDef huart3;

extern PCD_HandleTypeDef hpcd_USB_OTG_FS;

void cpp_main(void)
{
  while (1)
  {
  }
}