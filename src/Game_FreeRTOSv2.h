/*
 * Game_FreeRTOSv2.h
 *
 *  Created on: Feb 13, 2018
 *      Author: macos
 */

#ifndef GAME_FREERTOSV2_H_
#define GAME_FREERTOSV2_H_

// TODO: insert other include files here
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"
#include "timers.h"
#include "DigitalIoPin.h"
#include "LiquidCrystal.h"
#include "BarGraph.h"
#include "PropertyEdit.h"
#include "IntegerEdit.h"
#include "StatusEdit.h"
#include "SaveEdit.h"
#include "MenuItem.h"
#include "SimpleMenu.h"
#include "StateMachine.h"
#include "eepromIO.h"
#include "ITM_write.h"
#include <string>
#include <cstdlib>
//#include <cstring>

// TODO: insert other definitions and declarations here

/*****************************************************************************
 * Private types/enumerations/variables
 ****************************************************************************/

/*****************************************************************************
 * Public types/enumerations/variables
 ****************************************************************************/
extern int currentScore, highScore, timeLeft, currentPreset;
extern unsigned int currentTarget, lastTarget;
extern bool canVal;

/*Game parameters*/
extern uint8_t gameLengthV, litBeforeHitV, litAfterHitV,
		numberOfTargetsV, smallTargetPointV, mediumTargetPointV,
		largeTargetPointV, highScoreToggleV, timeLeftToggleV,
		soundToggleV;

/* FreeRTOS timer */
extern TimerHandle_t xTimer1;

/*****************************************************************************
 * Private functions
 ****************************************************************************/

/* Initializes pin muxing for UART interface - note that SystemInit() may
 already setup your pin muxing at system startup */
void Init_UART_PinMux(void);
void UART_Setup(void);

/* Initializes pin muxing for SPI interface - note that SystemInit() may
 already setup your pin muxing at system startup */
void Init_SPI0_PinMux(void);
void Init_SPI1_PinMux(void);

/* Turn on onboard Blue LED to indicate an SPI error */
void errorSPI(void);

/* Setup SPI handle and parameters */
void setupSpi0Master(void);
void setupSpi1Master(void);

/* Master SPI transmit in polling mode */
void WriteSpi0Mssg(uint16_t *xferPtr, uint32_t xferSize, uint32_t ssel,
		uint16_t *rferPtr);
void WriteSpi1Mssg(uint16_t *xferPtr, uint32_t xferSize, uint32_t ssel,
		uint16_t *rferPtr);

/*PININT handler setup*/
void Init_PIN_INT_PinMux(void);

/*IO Expander 0*/
void setup_IO_Expander_CurrentScoreDisplay(void);
/*IO Expander 1*/
void setup_IO_Expander_TimeDisplay(void);
/*IO Expander 2*/
void setup_IO_Expander_HighScoreDisplay(void);
/*IO Expander 4*/
void setup_IO_Expander_Targets(void);
/*IO Expander 5*/
void setup_IO_Expander_Buttons(void);

void clear_Pin_Int0(void);
void clear_Pin_Int1(void);

void SCT_Init(void);

void loadConfig(void);
void bootUpLoad(void);

void greenLED(unsigned int target, bool state);
void redLED(unsigned int target, bool state);

void soundStart(void);
void soundHit(void);
void soundMiss(void);
void soundEnd(void);

void increaseScore(unsigned int target);
void saveHighScore(void);

/*
 * Increase the score according to the size of the target that hit
 * Target 1-3: Small size (highest score)
 * Target 4-6: Medium size (middle score)
 * Target 7-10: Large size (smallest score)
 */
void increaseScore(int target);
/*****************************************************************************
 * Public functions
 ****************************************************************************/

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief	Handle interrupt from GPIO pin or GPIO pin mapped to PININT
 * @return	Nothing
 */
void PIN_INT0_IRQHandler(void);

/**
 * @brief	Handle interrupt from GPIO pin or GPIO pin mapped to PININT
 * @return	Nothing
 */
void PIN_INT1_IRQHandler(void);

#ifdef __cplusplus
}
#endif

#endif /* GAME_FREERTOSV2_H_ */
