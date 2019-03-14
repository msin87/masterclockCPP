#include "TouchScreen.h"

using namespace Touch;

TouchScreen::TouchScreen(IPeriph* iPeriph, GPIO_TypeDef *CSport, uint8_t CSpin, GPIO_TypeDef *IRQport, uint8_t IRQpin)
{
	hardware = iPeriph;
	touchCS.pin = CSpin;
	touchCS.port = CSport;
	touchIRQ.pin = IRQpin;
	touchIRQ.port = IRQport;
	calibrCoeffs = CalibrCoeffStruct{ 0, 0, 0, 0, 0, 0 };


}


TouchScreen::~TouchScreen()
{
}


void TouchScreen::startOn(IPeriph& iPeriph)
{
	hardware = &iPeriph;
	hardware->init();
}


void TouchScreen::setCS(State state)
{
	switch (state)
	{
	case State::SELECTED:
		touchCS.port->BSRR = touchCS.pin << 16; //GPIO_BSRR_BR сброс в 0
		break;
	case State::NOTSELECTED:
		touchCS.port->BSRR = touchCS.pin; //GPIO_BSRR_BS установка в 1
		break;
	}

}


bool TouchScreen::readDirtyXY()
{
	uint8_t rxByte = 0;
	uint16_t rxBuff[2];
	uint16_t X;
	uint16_t Y;
	//i=0 => reads hardware X, i=1 => reads hardware Y
	for (uint8_t i = 0; i < 2; i++)
	{
		setCS(State::SELECTED);
		osDelay(1);
		hardware->transmitUInt8(RDX);
		osDelay(1);
		hardware->transmitRecieveUint8(0, &rxByte);
		rxBuff[i] = rxByte << 8;
		hardware->transmitRecieveUint8(0, &rxByte);
		rxBuff[i] |= rxByte;
		osDelay(1);
		setCS(State::NOTSELECTED);
		osDelay(1);
	}
	X = 0xFFF * reverseX - rxBuff[0 + swapXY]; //0xFFF - макс значение
	Y = 0xFFF * reverseY - rxBuff[1 - swapXY];
	//	if (X < 100) 
	//	{
	//		X = 0; //лишнее условие, лучше приравнять к 101 и протесить.
	//	}
	if (X > 100 && Y > 100 && X < 4000 && Y < 4000)
	{
		dirtyXY.X = X;
		dirtyXY.Y = Y;
		return 1;
	}
	return 0;
}
const Coords* const TouchScreen::getXY(Calibrated calibrated)
{
	if (!medianFilter())
	{
		readed = false;
	}
	if (calibrated == Calibrated::CALIBRATED)applyCalibration();
	readed = true;
	return &filteredXY;
}

bool TouchScreen::medianFilter()
{
	uint8_t count = 0;
	bool sorted = false;
	uint16_t buffer_x[FILTERBUFFERSIZE];
	uint16_t buffer_y[FILTERBUFFERSIZE];
	while (isTouchPressed() && count < FILTERBUFFERSIZE)
	{
		if (readDirtyXY())
		{
			buffer_x[count] = dirtyXY.X;
			buffer_y[count] = dirtyXY.Y;
			count++;
		}
	}
	if (count == 10)
	{
		do
		{
			sorted = false;
			for (uint8_t count2 = 0; count2 < count - 1; count2++)
			{
				if (buffer_x[count2] > buffer_x[count2 + 1])
				{
					uint16_t temp = buffer_x[count2 + 1];
					buffer_x[count2 + 1] = buffer_x[count2];
					buffer_x[count2] = temp;
					sorted = true;
				}
			}
		} while (sorted);

		do
		{
			sorted = false;
			for (uint8_t count2 = 0; count2 < count - 1; count2++)
			{
				if (buffer_y[count2] > buffer_y[count2 + 1])
				{
					uint16_t temp = buffer_y[count2 + 1];
					buffer_y[count2 + 1] = buffer_y[count2];
					buffer_y[count2] = temp;
					sorted = true;
				}
			}
		} while (sorted);
		uint16_t middleX1, middleX2, middleY1, middleY2;
		middleX1 = buffer_x[3];
		middleX2 = buffer_x[4];
		middleY1 = buffer_y[3];
		middleY2 = buffer_y[4];
		if (((middleX1 > middleX2) && (middleX1 > middleX2 + TOLERANCE)) || ((middleX2 > middleX1) && (middleX2 > middleX1 + TOLERANCE)) || ((middleY1 > middleY2) && (middleY1 > middleY2 + TOLERANCE)) || ((middleY2 > middleY1) && (middleY2 > middleY1 + TOLERANCE)));
		else
		{
			uint16_t medianX = (buffer_x[3] + buffer_x[4]) / 2;
			uint16_t medianY = (buffer_y[3] + buffer_y[4]) / 2;
			if (medianX <= 0xFFF && medianY <= 0xFFF)
			{
				filteredXY.X = medianX;
				filteredXY.Y = medianY;
				return true;
			}
		}
		return false;
	}
	return false;
}


void TouchScreen::applyCalibration()
{
	filteredXY.X = (calibrCoeffs.A * filteredXY.X + calibrCoeffs.B * filteredXY.Y + calibrCoeffs.C) / RESCALE_FACTOR;
	filteredXY.Y = (calibrCoeffs.D * filteredXY.X + calibrCoeffs.E * filteredXY.Y + calibrCoeffs.F) / RESCALE_FACTOR;
}


void TouchScreen::setCalibrCoeffs(CalibrCoeffStruct calibrCoeffsStruct)
{
	calibrCoeffs = calibrCoeffsStruct;
}
