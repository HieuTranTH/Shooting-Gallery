/*
 * lcd_port.cpp
 *
 *  Created on: 26.1.2016
 *      Author: krl
 */

/* compatibility layer */
#include "board.h"
#include "lcd_port.h"
#include "Game_FreeRTOSv2.h"
#include "FreeRTOS.h"
#include "semphr.h"

#define LCD_bits_Mask 0xE0E0

static volatile bool RIT_flag;
xSemaphoreHandle sbRIT = xSemaphoreCreateBinary();
static volatile uint16_t lastState = 0x0000;

static const int dPinBit[] = { 13, 14, 15, 5, 6, 7 };
static const uint16_t dPinMask[] = { 0x2000, 0x4000, 0x8000, 0x0020, 0x0040,
		0x0080 };

static uint16_t mo4_buff[] = { 0x0000, 0x0000 };
static uint16_t mi4_buff[sizeof(mo4_buff) / sizeof(mo4_buff[0])];

extern "C" {
void RIT_IRQHandler(void) {
	// todo: implement RIT ISR that signals main task that timer has expired

	// This used to check if a context switch is required
	portBASE_TYPE xHigherPriorityWoken = pdFALSE;

	// Acknowledge interrupt by clearing RIT interrupt flag
	Chip_RIT_ClearIntStatus(LPC_RITIMER);

	// set a flag to notify main program
	//RIT_flag = true;

	Chip_RIT_Disable(LPC_RITIMER); // disable timer
	// Give semaphore and set context switch flag if a higher priority task was woken up
	xSemaphoreGiveFromISR(sbRIT, &xHigherPriorityWoken);
	// End the ISR and (possibly) do a context switch
	portEND_SWITCHING_ISR(xHigherPriorityWoken);
}
}

void digitalWrite(uint8_t pin, uint8_t val) {
	lastState = (lastState & ~dPinMask[pin])
			| ((val << dPinBit[pin]) & dPinMask[pin]);
	/*
	 * Register 0x12, 0x13: GPIO: GENERAL PURPOSE I/O PORT REGISTER
	 * Write GPIO outputs to both GPIOA and GPIOB
	 */
	mo4_buff[0] = 0x4012;
	mo4_buff[1] = lastState;
	/* Write simple message over SPI */
	WriteSpi1Mssg(mo4_buff, sizeof(mo4_buff) / sizeof(mo4_buff[0]), 0, mi4_buff);
}

void pinMode(uint8_t pin, uint8_t mode) {

}

void delayMicroseconds(int us) {
	// todo: implement accurate waiting using RIT-timer

	// calculate compare value
	uint64_t cmp_value = (uint64_t) Chip_Clock_GetSystemClockRate();
	cmp_value = cmp_value * (uint64_t) us / 1000000;
	// disable RIT – compare value may only be changed when RIT is disabled
	Chip_RIT_Disable(LPC_RITIMER);
	//clear wait flag
	//RIT_flag = false;
	//set compare value
	Chip_RIT_SetCompareValue(LPC_RITIMER, cmp_value);
	//clear RIT counter
	Chip_RIT_SetCounter(LPC_RITIMER, 0);
	//enable RIT
	Chip_RIT_Enable(LPC_RITIMER);

	// Enable the interrupt signal in NVIC (the interrupt controller)
	NVIC_EnableIRQ(RITIMER_IRQn);

	// wait for ISR to tell that we're done
	if (xSemaphoreTake(sbRIT, portMAX_DELAY) == pdTRUE) {
		// Disable the interrupt signal in NVIC (the interrupt controller)
		NVIC_DisableIRQ(RITIMER_IRQn);
	} else {
		// unexpected error
	}
}
