/*
 * DigitalIoPin.cpp
 *
 *  Created on: 31.1.2016
 *      Author: krl
 */

#include "DigitalIoPin.h"
#include "chip.h"

const int DigitalIoPin::dPort[] = { 0, 0, 0, 1, 1, 0,  //Arduino Analog GPIO row
		1, 1, 0, 0, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 	//Arduino Digital GPIO row
		1, 1, 1, 1, 0, 							//LPC right Extended GPIO row
		1, 0, 0, 0									//LPC left Extended GPIO row
		};
const int DigitalIoPin::dPin[] = { 7, 6, 5, 8, 6, 8, //Arduino Analog GPIO row
		10, 9, 29, 9, 10, 16, 3, 0, 24, 0, 27, 28, 12, 14, 23, 22,//Arduino Digital GPIO row
		7, 4, 5, 1, 3,							//LPC right Extended GPIO row
		2, 4, 1, 26								//LPC left Extended GPIO row
		};

/*
 * DigitalIoPin[0][0] : PIO0_7
 * DigitalIoPin[1][1] : PIO0_6
 * DigitalIoPin[2][2] : PIO0_5
 * DigitalIoPin[3][3] : PIO1_8
 * DigitalIoPin[4][4] : PIO1_6
 * DigitalIoPin[5][5] : PIO0_8
 * DigitalIoPin[6][6] : PIO1_10
 * DigitalIoPin[7][7] : PIO1_9
 * DigitalIoPin[8][8] : PIO0_29
 * DigitalIoPin[9][9] : PIO0_9
 * DigitalIoPin[10][10] : PIO0_10
 * DigitalIoPin[11][11] : PIO0_16
 * DigitalIoPin[12][12] : PIO1_3
 * DigitalIoPin[13][13] : PIO0_0
 * DigitalIoPin[14][14] : PIO0_24
 * DigitalIoPin[15][15] : PIO1_0
 * DigitalIoPin[16][16] : PIO0_27
 * DigitalIoPin[17][17] : PIO0_28
 * DigitalIoPin[18][18] : PIO0_12
 * DigitalIoPin[19][19] : PIO0_14
 * DigitalIoPin[20][20] : PIO0_23
 * DigitalIoPin[21][21] : PIO0_22
 * DigitalIoPin[22][22] : PIO1_7
 * DigitalIoPin[23][23] : PIO1_4
 * DigitalIoPin[24][24] : PIO1_5
 * DigitalIoPin[25][25] : PIO1_1
 * DigitalIoPin[26][26] : PIO1_3
 * DigitalIoPin[27][27] : PIO1_2
 * DigitalIoPin[28][28] : PIO0_4
 * DigitalIoPin[29][29] : PIO0_1
 * DigitalIoPin[30][30] : PIO0_26
 *
 *
 */

#define RESERVED 0x80
DigitalIoPin::DigitalIoPin(int arduinoPin, pinMode mode, bool invert) {
	// ugly but at this point I don't want to throw exceptions
	if (arduinoPin < 0 || arduinoPin > 30)
		arduinoPin = 0;

	pin = dPin[arduinoPin];
	port = dPort[arduinoPin];
	if (mode == output) {
		Chip_IOCON_PinMuxSet(LPC_IOCON, port, pin,
				IOCON_MODE_INACT | IOCON_DIGMODE_EN);
		Chip_GPIO_SetPinDIROutput(LPC_GPIO, port, pin);
	} else {
		uint32_t pm = IOCON_DIGMODE_EN;

		if (invert)
			pm |= IOCON_INV_EN;

		if (mode == pullup) {
			pm |= IOCON_MODE_PULLUP;
		} else if (mode == pulldown) {
			pm |= IOCON_MODE_PULLDOWN;
		}

		Chip_IOCON_PinMuxSet(LPC_IOCON, port, pin, pm);
		Chip_GPIO_SetPinDIRInput(LPC_GPIO, port, pin);
	}
}

DigitalIoPin::~DigitalIoPin() {
	// TODO Auto-generated destructor stub
}

bool DigitalIoPin::read() {
	return Chip_GPIO_GetPinState(LPC_GPIO, port, pin);
}

void DigitalIoPin::write(bool value) {
	return Chip_GPIO_SetPinState(LPC_GPIO, port, pin, value);
}

