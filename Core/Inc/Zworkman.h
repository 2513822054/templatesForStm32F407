#ifndef __WORKMAN_H_
#define __WORKMAN_H_
#include "ad9959.h"
#include "ADS8688.h"
#include "DAC8563.h"
#include "outputdata.h"
#include "tft.h"
#include "Zdosomething.h"
#include "Zresponse.h"
#include "W25Q128.h"
#include "WIFI.h"
#include "main.h"
#include "arm_math.h"
#include "Ztest.h"
#include "Zfir.h"

extern ADC_HandleTypeDef hadc1;
extern DMA_HandleTypeDef hdma_adc1;

extern DAC_HandleTypeDef hdac;
extern DMA_HandleTypeDef hdma_dac1;

extern SPI_HandleTypeDef hspi1;
extern SPI_HandleTypeDef hspi3;

extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim7;

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart6;

extern uint32_t count3;
extern uint16_t ad_dma_value;
extern uint32_t da_dma_value;

void get_ads_alldata();

//初始化自己外设及while循环前的函数
void init_start();
//while循环里做的函数
void while_do();




#endif
