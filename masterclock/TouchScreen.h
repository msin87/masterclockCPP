#pragma once
#include "iPeriph.h"
#include "functional"
#include "stm32f4xx_hal.h"

constexpr uint32_t RDX = 0X90; //0B10010000
constexpr uint32_t RDY = 0XD0; //0B11010000
constexpr uint8_t FILTERBUFFERSIZE = 10;
constexpr uint8_t TOLERANCE = 20;
constexpr uint32_t RESCALE_FACTOR = 1000000;
namespace Touch
{
	typedef struct
	{
		uint16_t X, Y;
	} Coords;
	typedef struct
	{
		uint32_t A, B, C, D, E, F;
	} CalibrCoeffStruct;
	typedef struct
	{
		GPIO_TypeDef *port;
		uint8_t pin;
	} GPIOStruct;
	enum class State
	{
		SELECTED,
		NOTSELECTED
	};
	enum class Calibrated
	{
		CALIBRATED,
		NOTCALIBRATED
	};
} // namespace Touch

class TouchScreen
{

	Touch::Coords dirtyXY, filteredXY;

	Touch::GPIOStruct touchCS;
	Touch::GPIOStruct touchIRQ;
	Touch::CalibrCoeffStruct calibrCoeffs;
	IPeriph *hardware;
	uint16_t Xdirty, Ydirty;
	bool swapXY;
	bool calibrated;
	bool reverseX;
	bool reverseY;
	bool readed;
	bool readDirtyXY();
	void setCS(Touch::State state);
	bool isTouchPressed() { return  HAL_GPIO_ReadPin(touchIRQ.port, touchIRQ.pin); }
	bool medianFilter();
	void applyCalibration();
public:
	TouchScreen(IPeriph *iPeriph, GPIO_TypeDef *CSport, uint8_t CSpin, GPIO_TypeDef *IRQport, uint8_t IRQpin);
	void startOn(IPeriph&);
	void setReverseX(bool reverse) { reverseX = reverse; }
	void setReverseY(bool reverse) { reverseY = reverse; }
	const Touch::Coords* const getXY(Touch::Calibrated calibrated);
	void setSwapXY(bool swap) { swapXY = swap; }
	bool isCoordsOk() { return readed; }
	void setCalibrCoeffs(Touch::CalibrCoeffStruct calibrCoeffsStruct);
	~TouchScreen();
};

