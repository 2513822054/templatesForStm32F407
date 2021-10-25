#include "Zresponse.h"



void ads_get_all()
{
	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);
	  HAL_SPI_TransmitReceive(&hspi3, txbuf, rxbuf, 2, 10);
	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
	  ads_data[0] = *(uint16_t *)(&rxbuf[2]);

}

void resTIM3()
{

}

