#include "SPI.h"
void SPI::Error_Handler(Error error)
{
	while (1)
	{
		
	}
}
void SPI::init(void)
{
	if (HAL_SPI_Init(&hspi) != HAL_OK)
	{
		Error_Handler(SPI::Error::INIT_ERROR);
	}

}

void SPI::deInit(void)
{
	if (HAL_SPI_DeInit(&hspi) != HAL_OK)
	{
		Error_Handler(SPI::Error::DEINIT_ERROR);
	}
}
inline void SPI::transmitUInt8(uint8_t buff, uint32_t timeout)
{
	HAL_SPI_Transmit(&hspi, &buff, sizeof(buff), timeout);
}
inline void SPI::transmitBuffUInt8(uint8_t *txbuff, uint16_t size, uint32_t timeout) 
{
	HAL_SPI_Transmit(&hspi, txbuff, size, timeout);
}
inline void SPI::transmitRecieveUint8(uint8_t tx, uint8_t* rx, uint32_t timeout)
{
	HAL_SPI_TransmitReceive(&hspi, &tx, rx, 1, timeout);
}
SPI::SPI(SPI_TypeDef *instance, SPI_InitTypeDef initStruct)
{
	hspi.Instance = instance;
	hspi.Init = initStruct;
	
}