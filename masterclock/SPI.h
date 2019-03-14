#pragma once
#include "cmsis_os.h"
#include "stm32f4xx_hal.h"
#include "iPeriph.h"
class SPI: public IPeriph

{
	SPI_HandleTypeDef hspi;
	enum class Error
	{
		NOERROR,
		INIT_ERROR,
		DEINIT_ERROR
	}; 
	static void	Error_Handler(Error);

  public:
	
	SPI(const SPI_HandleTypeDef& _hspi) : hspi(_hspi) {};
	SPI(SPI_TypeDef*, SPI_InitTypeDef);

	void init() override;
	void deInit() override;
	void transmitUInt8(uint8_t buff, uint32_t timeout=1000) override;
	void transmitBuffUInt8(uint8_t *buff, uint16_t length, uint32_t timeout) override;
	void transmitRecieveUint8(uint8_t tx, uint8_t* rx, uint32_t timeout = 1000) override;
};

