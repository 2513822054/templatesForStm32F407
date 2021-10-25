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

uint16_t ad_dma_value;
uint32_t da_dma_value=2000;
void init_start()
{
	uint8_t buf1[4]={0x11,0x22,0x33,0x44};
	uint8_t buf2[4]={0};
	uint16_t ads_data[4];
	float num,storagenum = -222.3334;

	//用什么初始化什么
	//W25QXX_Init();
	//ADS8688_Init(&ads, &hspi3, SPI3_CS_GPIO_Port, SPI3_CS_Pin);
	//Init_AD9959();
	//DAC8563_Init();
	//HAL_TIM_Base_Start(&htim6);
	//HAL_DAC_Start_DMA(&hdac,DAC1_CHANNEL_1,(uint32_t*)&da_dma_value,1,DAC_ALIGN_12B_R);
	//iir_init();
	//HAL_ADC_Start_DMA(&hadc1,(uint32_t*)&ad_dma_value,1);
	//HAL_TIM_Base_Start_IT(&htim3);



}


void while_do()
{
	uint32_t i;
	for(i=0;i<=49999;i++);
	HAL_GPIO_TogglePin(LED1_GPIO_Port,LED1_Pin);
	HAL_GPIO_TogglePin(LED2_GPIO_Port,LED2_Pin);
}
