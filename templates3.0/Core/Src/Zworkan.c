#include "Zworkman.h"


uint32_t count3=0;
void storage_float(uint32_t place,float num)
{

	uint32_t storagenum=(int)num;


	W25QXX_Write(&storagenum,place,4);
}

float read_float(uint32_t place)
{
	int storagenum;
	float num;
	W25QXX_Read(&storagenum,place,4);

	num = *(float*)&storagenum;

	return num;
}


void delay_us(uint16_t us)
{
	TIM14->CNT = 0;
	while(TIM14->CNT < us);
}



uint16_t ad_dma_value;
uint32_t da_dma_value=2000;
void init_start()
{


	//用什么初始化什么
	//W25QXX_Init();
	//ADS8688_Init(&ads, &hspi3, SPI3_CS_GPIO_Port, SPI3_CS_Pin);
	//Init_AD9959();
	//DAC8563_Init();
	//HAL_TIM_Base_Start(&htim6);
	//HAL_DAC_Start_DMA(&hdac,DAC1_CHANNEL_1,(uint32_t*)&da_dma_value,1,DAC_ALIGN_12B_R);
	//iir_init();
	//Fir_Realtime_Init();
	//HAL_ADC_Start_DMA(&hadc1,(uint32_t*)&ad_dma_value,1);
	//HAL_TIM_Base_Start_IT(&htim3);
	//WIFI_TCP_Server_Init(1);


	//HAL_TIM_Base_Start_IT(&htim3);
	//HAL_TIM_Base_Start(&htim14);

}


void while_do()
{
	uint32_t i;
	//for(i=0;i<=139999;i++);
	i=0;
	//delay_us(10000);
	HAL_GPIO_TogglePin(LED1_GPIO_Port,LED1_Pin);
	HAL_GPIO_TogglePin(LED2_GPIO_Port,LED2_Pin);
}
