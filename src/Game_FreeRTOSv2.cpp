/*
 ===============================================================================
 Name        : main.c
 Author      : $(author)
 Version     :
 Copyright   : $(copyright)
 Description : main definition
 ===============================================================================
 */

#if defined (__USE_LPCOPEN)
#if defined(NO_BOARD_LIB)
#include "chip.h"
#else
#include "board.h"
#endif
#endif

#include <cr_section_macros.h>

// TODO: insert other include files here
#include "Game_FreeRTOSv2.h"

// TODO: insert other definitions and declarations here

/*****************************************************************************
 * Private types/enumerations/variables
 ****************************************************************************/
/* Transmit and Receive Buffers */
/*SPI0*/
//Master out buffer - IO Expander 0
static uint16_t mo0_buff[] = { 0x0000, 0x0000 };
//Master in buffer - IO Expander 0
static uint16_t mi0_buff[sizeof(mo0_buff) / sizeof(mo0_buff[0])];

//Master out buffer - IO Expander 1
static uint16_t mo1_buff[] = { 0x0000, 0x0000 };
//Master in buffer - IO Expander 1
static uint16_t mi1_buff[sizeof(mo1_buff) / sizeof(mo1_buff[0])];

//Master out buffer - IO Expander 2
static uint16_t mo2_buff[] = { 0x0000, 0x0000 };
//Master in buffer - IO Expander 2
static uint16_t mi2_buff[sizeof(mo2_buff) / sizeof(mo2_buff[0])];

/*SPI1*/
//Master out buffer - IO Expander 4
static uint16_t mo4_buff[] = { 0x0000, 0x0000 };
//Master in buffer - IO Expander 4
static uint16_t mi4_buff[sizeof(mo4_buff) / sizeof(mo4_buff[0])];

//Master out buffer - IO Expander 5
static uint16_t mo5_buff[] = { 0x0000, 0x0000 };
//Master in buffer - IO Expander 5
static uint16_t mi5_buff[sizeof(mo5_buff) / sizeof(mo5_buff[0])];

/* SPI Transfer Setup */
static SPI_DATA_SETUP_T XferSetup;

/* Transmit and receive ring buffers */
STATIC RINGBUFF_T txring, rxring;

/* Ring buffer size */
#define UART_RB_SIZE 64

/* Set the default UART, IRQ number, and IRQ handler name */
#define LPC_USART       LPC_USART0
#define LPC_IRQNUM      UART0_IRQn
#define LPC_UARTHNDLR   UART0_IRQHandler

/* Default baudrate for testing */
#define UART_TEST_DEFAULT_BAUDRATE 115200

/* Transmit and receive buffers */
static uint8_t rxbuff[UART_RB_SIZE], txbuff[UART_RB_SIZE];

/*****************************************************************************
 * Public types/enumerations/variables
 ****************************************************************************/

struct QueueItem {
	enum itemType {
		Tick, Button, Target
	};
	itemType type;
	uint16_t value;
};

TickType_t systicks = 0;
int currentScore = 0, highScore = 0, timeLeft = 0, currentPreset = 1;
unsigned int currentTarget = 0, lastTarget = 0;
bool canVal = false;

/* PinINT Semaphore to unblock ClearPinINT task */
static volatile xSemaphoreHandle clearPININT0 = xSemaphoreCreateBinary();
static volatile xSemaphoreHandle clearPININT1 = xSemaphoreCreateBinary();
/* Uart Semaphore that given from Uart ISR to unblock Uart transmission task */
static volatile xSemaphoreHandle uartSem = xSemaphoreCreateBinary();
/* Sound tasks Semaphore */
static volatile xSemaphoreHandle soundStartSem = xSemaphoreCreateBinary();
static volatile xSemaphoreHandle soundHitSem = xSemaphoreCreateBinary();
static volatile xSemaphoreHandle soundMissSem = xSemaphoreCreateBinary();
static volatile xSemaphoreHandle soundEndSem = xSemaphoreCreateBinary();

/* SPI Mutex to protect SPI0 transmission */
static volatile xSemaphoreHandle SpiMutex = xSemaphoreCreateMutex();
static volatile xSemaphoreHandle SoundMutex = xSemaphoreCreateMutex();

/*Input queue */
static volatile QueueHandle_t xQueue = xQueueCreate(10, sizeof(QueueItem));

/* FreeRTOS timer */
static volatile TimerHandle_t xTimer1;

/*Game parameters*/
uint8_t gameLengthV = 100, litBeforeHitV = 10, litAfterHitV = 2,
		numberOfTargetsV = 3, smallTargetPointV = 5, mediumTargetPointV = 2,
		largeTargetPointV = 1, highScoreToggleV = 1, timeLeftToggleV = 1,
		soundToggleV = 1;

/*****************************************************************************
 * Private functions
 ****************************************************************************/
/* Sets up system hardware */
static void prvSetupHardware(void) {
#if defined (__USE_LPCOPEN)
	// Read clock settings and update SystemCoreClock variable
	SystemCoreClockUpdate();
#if !defined(NO_BOARD_LIB)
	// Set up and initialize all required blocks and
	// functions related to the board hardware
	Board_Init();
	// Set the LED to the state of "On"
	Board_LED_Set(0, true);
#endif
#endif

	ITM_init();

	Init_UART_PinMux();
	UART_Setup();

	eeprom_Setup();

	/*******************SPI0 setup***************************************************/
	/* Setup SPI pin muxing */
	Init_SPI0_PinMux();
	/* Allocate SPI handle, setup rate, and initialize clocking */
	setupSpi0Master();

	/* Setup SPI pin muxing */
	Init_SPI1_PinMux();
	/* Allocate SPI handle, setup rate, and initialize clocking */
	setupSpi1Master();

	/*************************************************************************************/
	setup_IO_Expander_CurrentScoreDisplay();
	setup_IO_Expander_TimeDisplay();
	setup_IO_Expander_HighScoreDisplay();

	setup_IO_Expander_Targets();
	setup_IO_Expander_Buttons();
	/*************************************************************************************/

	Init_PIN_INT_PinMux();

	/*************************************************************************************/
	SCT_Init();

	/*************************************************************************************/
	// initialize RIT (= enable clocking etc.)
	Chip_RIT_Init(LPC_RITIMER);
	// set the priority level of the interrupt
	// The level must be equal or lower than the maximum priority specified in FreeRTOS config // Note that in a Cortex-M3 a higher number indicates lower interrupt priority
	NVIC_SetPriority(RITIMER_IRQn,
	configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY + 1);

	/*
	 * Load first preset and variables from EEPROM
	 */
	bootUpLoad();
}

void Init_UART_PinMux(void) {

	/* UART signals on pins PIO0_13 (FUNC0, U0_TXD) and PIO0_18 (FUNC0, U0_RXD) */
	Chip_IOCON_PinMuxSet(LPC_IOCON, 0, 13,
			(IOCON_FUNC0 | IOCON_MODE_INACT | IOCON_DIGMODE_EN));
	Chip_IOCON_PinMuxSet(LPC_IOCON, 0, 18,
			(IOCON_FUNC0 | IOCON_MODE_INACT | IOCON_DIGMODE_EN));

	/* UART signal muxing via SWM */
	Chip_SWM_MovablePortPinAssign(SWM_UART0_RXD_I, 0, 13);
	Chip_SWM_MovablePortPinAssign(SWM_UART0_TXD_O, 0, 18);

}

void UART_Setup(void) {
	/* Use main clock rate as base for UART baud rate divider */
	Chip_Clock_SetUARTBaseClockRate(Chip_Clock_GetMainClockRate(), false);

	/* Use 128x expected UART baud rate for fractional baud mode. */
	Chip_Clock_SetUARTBaseClockRate((115200 * 128), true);

	/* Setup UART */
	Chip_UART_Init(LPC_USART);
	Chip_UART_ConfigData(LPC_USART,
	UART_CFG_DATALEN_8 | UART_CFG_PARITY_NONE | UART_CFG_STOPLEN_1);
	Chip_UART_SetBaud(LPC_USART, UART_TEST_DEFAULT_BAUDRATE);
	Chip_UART_Enable(LPC_USART);
	Chip_UART_TXEnable(LPC_USART);

	/* Before using the ring buffers, initialize them using the ring
	 buffer init function */
	RingBuffer_Init(&rxring, rxbuff, 1, UART_RB_SIZE);
	RingBuffer_Init(&txring, txbuff, 1, UART_RB_SIZE);

	/* Enable receive data and line status interrupt */
	Chip_UART_IntEnable(LPC_USART, UART_INTEN_RXRDY);
	Chip_UART_IntDisable(LPC_USART, UART_INTEN_TXRDY); /* May not be needed */

	// set the priority level of the interrupt
	// The level must be equal or lower than the maximum priority specified in FreeRTOS config
	// Note that in a Cortex-M3 a higher number indicates lower interrupt priority
	NVIC_SetPriority(LPC_IRQNUM,
	configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY + 1);

	/* Enable UART interrupt */
	NVIC_EnableIRQ(LPC_IRQNUM);
}

void Init_SPI0_PinMux(void) {
#if (defined(BOARD_NXP_LPCXPRESSO_1549))

	/* Enable the clock to the Switch Matrix */
	Chip_Clock_EnablePeriphClock(SYSCTL_CLOCK_SWM);
	/*
	 * Initialize SPI0 pins connect
	 * SCK0: PINASSIGN3[15:8]: Select P0.8
	 * MOSI0: PINASSIGN3[23:16]: Select P1.6
	 * MISO0: PINASSIGN3[31:24] : Select P1.8
	 * SSEL0: PINASSIGN4[7:0]: Select P0.5
	 * SSEL1: PINASSIGN4[7:0]: Select P0.6
	 * SSEL2: PINASSIGN4[7:0]: Select P0.7
	 */
	Chip_IOCON_PinMuxSet(LPC_IOCON, 0, 8,
			(IOCON_MODE_INACT | IOCON_DIGMODE_EN));
	Chip_IOCON_PinMuxSet(LPC_IOCON, 1, 6,
			(IOCON_MODE_INACT | IOCON_DIGMODE_EN));
	Chip_IOCON_PinMuxSet(LPC_IOCON, 1, 8,
			(IOCON_MODE_INACT | IOCON_DIGMODE_EN));
	Chip_IOCON_PinMuxSet(LPC_IOCON, 0, 5,
			(IOCON_MODE_INACT | IOCON_DIGMODE_EN));
	Chip_IOCON_PinMuxSet(LPC_IOCON, 0, 6,
			(IOCON_MODE_INACT | IOCON_DIGMODE_EN));
	Chip_IOCON_PinMuxSet(LPC_IOCON, 0, 7,
			(IOCON_MODE_INACT | IOCON_DIGMODE_EN));

	Chip_SWM_MovablePortPinAssign(SWM_SPI0_SCK_IO, 0, 8); /* P0.8 */
	Chip_SWM_MovablePortPinAssign(SWM_SPI0_MOSI_IO, 1, 6);/* P1.6 */
	Chip_SWM_MovablePortPinAssign(SWM_SPI0_MISO_IO, 1, 8);/* P1.8 */
	Chip_SWM_MovablePortPinAssign(SWM_SPI0_SSELSN_0_IO, 0, 5); /* P0.5 */
	Chip_SWM_MovablePortPinAssign(SWM_SPI0_SSELSN_1_IO, 0, 6); /* P0.6 */
	Chip_SWM_MovablePortPinAssign(SWM_SPI0_SSELSN_2_IO, 0, 7); /* P0.7 */

	/* Disable the clock to the Switch Matrix to save power */
	Chip_Clock_DisablePeriphClock(SYSCTL_CLOCK_SWM);
#else
	/* Configure your own SPI pin muxing here if needed */
#warning "No SPI pin muxing defined"
#endif
}

void Init_SPI1_PinMux(void) {
#if (defined(BOARD_NXP_LPCXPRESSO_1549))

	/* Enable the clock to the Switch Matrix */
	Chip_Clock_EnablePeriphClock(SYSCTL_CLOCK_SWM);
	/*
	 * Initialize SPI1 pins connect
	 * SCK1: PINASSIGN3[15:8]: Select P1.3
	 * MOSI1: PINASSIGN3[23:16]: Select P0.16
	 * MISO1: PINASSIGN3[31:24] : Select P0.10
	 * SSEL0: PINASSIGN4[7:0]: Select P0.9
	 * SSEL1: PINASSIGN4[7:0]: Select P0.29
	 */
	Chip_IOCON_PinMuxSet(LPC_IOCON, 1, 3,
			(IOCON_MODE_INACT | IOCON_DIGMODE_EN));
	Chip_IOCON_PinMuxSet(LPC_IOCON, 0, 16,
			(IOCON_MODE_INACT | IOCON_DIGMODE_EN));
	Chip_IOCON_PinMuxSet(LPC_IOCON, 0, 10,
			(IOCON_MODE_INACT | IOCON_DIGMODE_EN));
	Chip_IOCON_PinMuxSet(LPC_IOCON, 0, 9,
			(IOCON_MODE_INACT | IOCON_DIGMODE_EN));
	Chip_IOCON_PinMuxSet(LPC_IOCON, 0, 29,
			(IOCON_MODE_INACT | IOCON_DIGMODE_EN));

	Chip_SWM_MovablePortPinAssign(SWM_SPI1_SCK_IO, 1, 3); /* P1.3 */
	Chip_SWM_MovablePortPinAssign(SWM_SPI1_MOSI_IO, 0, 16);/* P0.16 */
	Chip_SWM_MovablePortPinAssign(SWM_SPI1_MISO_IO, 0, 10);/* P0.10 */
	Chip_SWM_MovablePortPinAssign(SWM_SPI1_SSELSN_0_IO, 0, 9); /* P0.9 */
	Chip_SWM_MovablePortPinAssign(SWM_SPI1_SSELSN_1_IO, 0, 29); /* P0.29 */

	/* Disable the clock to the Switch Matrix to save power */
	Chip_Clock_DisablePeriphClock(SYSCTL_CLOCK_SWM);
#else
	/* Configure your own SPI pin muxing here if needed */
#warning "No SPI pin muxing defined"
#endif
}

void errorSPI(void) {
	Board_LED_Set(2, true);
	while (1) {
	}
}

void setupSpi0Master(void) {
	SPI_CFG_T spiCfg;
	SPI_DELAY_CONFIG_T spiDelayCfg;
	/* Initialize SPI Block */
	Chip_SPI_Init(LPC_SPI0);
	/* Set SPI Config register */
	spiCfg.ClkDiv = 0x0024; /* Set Clock divider to maximum */
	spiCfg.Mode = SPI_MODE_MASTER; /* Enable Master Mode */
	spiCfg.ClockMode = SPI_CLOCK_MODE0; /* Enable Mode 0 */
	spiCfg.DataOrder = SPI_DATA_MSB_FIRST; /* Transmit MSB first */
	/* Slave select polarity is active low */
	spiCfg.SSELPol = (SPI_CFG_SPOL0_LO | SPI_CFG_SPOL1_LO | SPI_CFG_SPOL2_LO
			| SPI_CFG_SPOL3_LO);
	Chip_SPI_SetConfig(LPC_SPI0, &spiCfg);
	/* Set Delay register */
	spiDelayCfg.PreDelay = 0;
	spiDelayCfg.PostDelay = 0;
	spiDelayCfg.FrameDelay = 0;
	spiDelayCfg.TransferDelay = 0;
	Chip_SPI_DelayConfig(LPC_SPI0, &spiDelayCfg);
	/* Enable SPI0 */
	Chip_SPI_Enable(LPC_SPI0);
}

/* Setup SPI handle and parameters */
void setupSpi1Master(void) {
	SPI_CFG_T spiCfg;
	SPI_DELAY_CONFIG_T spiDelayCfg;
	/* Initialize SPI Block */
	Chip_SPI_Init(LPC_SPI1);
	/* Set SPI Config register */
	spiCfg.ClkDiv = 0x0024; /* Set Clock divider to maximum */
	spiCfg.Mode = SPI_MODE_MASTER; /* Enable Master Mode */
	spiCfg.ClockMode = SPI_CLOCK_MODE0; /* Enable Mode 0 */
	spiCfg.DataOrder = SPI_DATA_MSB_FIRST; /* Transmit MSB first */
	/* Slave select polarity is active low */
	spiCfg.SSELPol = (SPI_CFG_SPOL0_LO | SPI_CFG_SPOL1_LO);
	Chip_SPI_SetConfig(LPC_SPI1, &spiCfg);
	/* Set Delay register */
	spiDelayCfg.PreDelay = 2;
	spiDelayCfg.PostDelay = 2;
	spiDelayCfg.FrameDelay = 2;
	spiDelayCfg.TransferDelay = 2;
	Chip_SPI_DelayConfig(LPC_SPI1, &spiDelayCfg);
	/* Enable SPI1 */
	Chip_SPI_Enable(LPC_SPI1);
}

void WriteSpi0Mssg(uint16_t *xferPtr, uint32_t xferSize, uint32_t ssel,
		uint16_t *rferPtr) {
	if (xSemaphoreTake(SpiMutex, portMAX_DELAY) == pdTRUE) {
		/* Setup Transfer structure, this data should be retained for the entire transmission */
		XferSetup.pTx = xferPtr; /* Transmit Buffer */
		XferSetup.pRx = rferPtr;/* Receive Buffer */
		XferSetup.DataSize = sizeof(uint16_t) * 8; /* Data size in bits */
		XferSetup.Length = xferSize; /* Total frame length */
		switch (ssel) {
		case 0:
			/* Assert only SSEL0 */
			XferSetup.ssel = SPI_TXCTL_ASSERT_SSEL0 | SPI_TXCTL_DEASSERT_SSEL1
					| SPI_TXCTL_DEASSERT_SSEL2 |
					SPI_TXCTL_DEASSERT_SSEL3;
			break;
		case 1:
			/* Assert only SSEL1 */
			XferSetup.ssel = SPI_TXCTL_DEASSERT_SSEL0 | SPI_TXCTL_ASSERT_SSEL1
					| SPI_TXCTL_DEASSERT_SSEL2 |
					SPI_TXCTL_DEASSERT_SSEL3;
			break;
		case 2:
			/* Assert only SSEL2 */
			XferSetup.ssel = SPI_TXCTL_DEASSERT_SSEL0 | SPI_TXCTL_DEASSERT_SSEL1
					| SPI_TXCTL_ASSERT_SSEL2 |
					SPI_TXCTL_DEASSERT_SSEL3;
			break;
		case 3:
			/* Assert only SSEL3 */
			XferSetup.ssel = SPI_TXCTL_DEASSERT_SSEL0 | SPI_TXCTL_DEASSERT_SSEL1
					| SPI_TXCTL_DEASSERT_SSEL2 |
					SPI_TXCTL_ASSERT_SSEL3;
			break;
		default:
			/* Assert only SSEL0 */
			XferSetup.ssel = SPI_TXCTL_ASSERT_SSEL0 | SPI_TXCTL_DEASSERT_SSEL1
					| SPI_TXCTL_DEASSERT_SSEL2 |
					SPI_TXCTL_DEASSERT_SSEL3;
			break;
		}
		XferSetup.TxCnt = 0;
		XferSetup.RxCnt = 0;
		if (Chip_SPI_RWFrames_Blocking(LPC_SPI0, &XferSetup) > 0) {

		} else {
			/* Signal SPI error */
			errorSPI();
		}
		xSemaphoreGive(SpiMutex);
	}
}

/* Master SPI transmit in polling mode */
void WriteSpi1Mssg(uint16_t *xferPtr, uint32_t xferSize, uint32_t ssel,
		uint16_t *rferPtr) {
	if (xSemaphoreTake(SpiMutex, portMAX_DELAY) == pdTRUE) {
		/* Setup Transfer structure, this data should be retained for the entire transmission */
		XferSetup.pTx = xferPtr; /* Transmit Buffer */
		XferSetup.pRx = rferPtr;/* Receive Buffer */
		XferSetup.DataSize = sizeof(uint16_t) * 8; /* Data size in bits */
		XferSetup.Length = xferSize; /* Total frame length */
		switch (ssel) {
		case 0:
			/* Assert only SSEL0 */
			XferSetup.ssel = SPI_TXCTL_ASSERT_SSEL0 | SPI_TXCTL_DEASSERT_SSEL1;
			break;
		case 1:
			/* Assert only SSEL1 */
			XferSetup.ssel = SPI_TXCTL_DEASSERT_SSEL0 | SPI_TXCTL_ASSERT_SSEL1;
			break;
		default:
			/* Assert only SSEL0 */
			XferSetup.ssel = SPI_TXCTL_ASSERT_SSEL0 | SPI_TXCTL_DEASSERT_SSEL1;
			break;
		}
		XferSetup.TxCnt = 0;
		XferSetup.RxCnt = 0;
		if (Chip_SPI_RWFrames_Blocking(LPC_SPI1, &XferSetup) > 0) {

		} else {
			/* Signal SPI error */
			errorSPI();
		}
		xSemaphoreGive(SpiMutex);
	}
}

/*PININT handler setup*/
void Init_PIN_INT_PinMux(void) {
	/* Initialize PININT driver */
	Chip_PININT_Init(LPC_GPIO_PIN_INT);

	/* Set pin back to GPIO (on some boards may have been changed to something
	 else by Board_Init()) */
	/*IO Expander 4 interrupt pin */
	Chip_IOCON_PinMuxSet(LPC_IOCON, 1, 9,
			(IOCON_DIGMODE_EN | IOCON_MODE_PULLUP));
	/*IO Expander 5 interrupt pin */
	Chip_IOCON_PinMuxSet(LPC_IOCON, 1, 10,
			(IOCON_DIGMODE_EN | IOCON_MODE_PULLUP));

	/* Configure GPIO pin as input */
	Chip_GPIO_SetPinDIRInput(LPC_GPIO, 1, 9);
	Chip_GPIO_SetPinDIRInput(LPC_GPIO, 1, 10);

	/* Enable PININT clock */
	Chip_Clock_EnablePeriphClock(SYSCTL_CLOCK_PININT);

	/* Reset the PININT block */
	Chip_SYSCTL_PeriphReset(RESET_PININT);

	/* Configure interrupt channel for the GPIO pin in INMUX block */
	Chip_INMUX_PinIntSel(0, 1, 9);
	Chip_INMUX_PinIntSel(1, 1, 10);

	/* Configure channel interrupt as edge sensitive and falling edge interrupt */
	Chip_PININT_ClearIntStatus(LPC_GPIO_PIN_INT, PININTCH0);
	Chip_PININT_SetPinModeEdge(LPC_GPIO_PIN_INT, PININTCH0);
	Chip_PININT_EnableIntLow(LPC_GPIO_PIN_INT, PININTCH0);

	Chip_PININT_ClearIntStatus(LPC_GPIO_PIN_INT, PININTCH1);
	Chip_PININT_SetPinModeEdge(LPC_GPIO_PIN_INT, PININTCH1);
	Chip_PININT_EnableIntLow(LPC_GPIO_PIN_INT, PININTCH1);

	// set the priority level of the interrupt
	// The level must be equal or lower than the maximum priority specified in FreeRTOS config
	// Note that in a Cortex-M3 a higher number indicates lower interrupt priority
	NVIC_SetPriority(PIN_INT0_IRQn,
	configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY + 1);
	NVIC_SetPriority(PIN_INT1_IRQn,
	configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY + 1);

	/* Enable interrupt in the NVIC */
	NVIC_ClearPendingIRQ(PIN_INT0_IRQn);
	NVIC_ClearPendingIRQ(PIN_INT1_IRQn);
	NVIC_EnableIRQ(PIN_INT0_IRQn);
	NVIC_EnableIRQ(PIN_INT1_IRQn);
}

void setup_IO_Expander_CurrentScoreDisplay(void) {
	/*
	 * Register 0x00, 0x01: IODIR: I/O DIRECTION REGISTER
	 * Set GPIOA and GPIOB direction to output
	 */
	mo0_buff[0] = 0x4000;
	mo0_buff[1] = 0x0000;
	/* Write simple message over SPI */
	WriteSpi0Mssg(mo0_buff, sizeof(mo0_buff) / sizeof(mo0_buff[0]), 0,
			mi0_buff);
}

void setup_IO_Expander_TimeDisplay(void) {
	/*
	 * Register 0x00, 0x01: IODIR: I/O DIRECTION REGISTER
	 * Set GPIOA and GPIOB direction to output
	 */
	mo1_buff[0] = 0x4000;
	mo1_buff[1] = 0x0000;
	/* Write simple message over SPI */
	WriteSpi0Mssg(mo1_buff, sizeof(mo1_buff) / sizeof(mo1_buff[0]), 1,
			mi1_buff);
}

void setup_IO_Expander_HighScoreDisplay(void) {
	/*
	 * Register 0x00, 0x01: IODIR: I/O DIRECTION REGISTER
	 * Set GPIOA and GPIOB direction to output
	 */
	mo2_buff[0] = 0x4000;
	mo2_buff[1] = 0x0000;
	/* Write simple message over SPI */
	WriteSpi0Mssg(mo2_buff, sizeof(mo2_buff) / sizeof(mo2_buff[0]), 2,
			mi2_buff);
}

void setup_IO_Expander_Targets(void) {
	/*
	 * Register 0x00, 0x01: IODIR: I/O DIRECTION REGISTER
	 * Set bits 0-4 GPIOA and GPIOB direction to inputs, bits 5-7 to outputs
	 */
	mo4_buff[0] = 0x4000;
	mo4_buff[1] = 0x1F1F;
	/* Write simple message over SPI */
	WriteSpi1Mssg(mo4_buff, sizeof(mo4_buff) / sizeof(mo4_buff[0]), 0,
			mi4_buff);

	/*
	 * Register 0x02, 0x03: IPOL: INPUT POLARITY PORT REGISTER
	 * Set bits 0-4 GPIOA and GPIOB input polarity to be inverted
	 * LOGIC LOW = 5V, LOGIC HIGH = 0V
	 */
	mo4_buff[0] = 0x4002;
	mo4_buff[1] = 0x1F1F;
	/* Write simple message over SPI */
	WriteSpi1Mssg(mo4_buff, sizeof(mo4_buff) / sizeof(mo4_buff[0]), 0,
			mi4_buff);

	/*
	 * Register 0x0C, 0x0D: GPPU: GPIO PULL-UP RESISTOR REGISTER
	 * Set bits 0-4 GPIOA and GPIOB pull-up resistors
	 * NORMAL STATE = 5V, PRESSED STATE = 0V
	 */
	mo4_buff[0] = 0x400C;
	mo4_buff[1] = 0x1F1F;
	/* Write simple message over SPI */
	WriteSpi1Mssg(mo4_buff, sizeof(mo4_buff) / sizeof(mo4_buff[0]), 0,
			mi4_buff);

	/*
	 * Register 0x06, 0x07: DEFVAL: DEFAULT VALUE REGISTER
	 * Set GPIOA and GPIOB input default interrupt-on-change compare value to be zero
	 */
	mo4_buff[0] = 0x4006;
	mo4_buff[1] = 0x0000;
	/* Write simple message over SPI */
	WriteSpi1Mssg(mo4_buff, sizeof(mo4_buff) / sizeof(mo4_buff[0]), 0,
			mi4_buff);

	/*
	 * Register 0x08, 0x09: INTCON: INTERRUPT-ON-CHANGE CONTROL REGISTER
	 * Set bits 0-4 GPIOA and GPIOB input interrupt-on-change control register
	 * Pin value is compared against the associated bit in the DEFVAL register
	 */
	mo4_buff[0] = 0x4008;
	//mo4_buff[1] = 0x1F1F;
	mo4_buff[1] = 0x0000;
	/* Write simple message over SPI */
	WriteSpi1Mssg(mo4_buff, sizeof(mo4_buff) / sizeof(mo4_buff[0]), 0,
			mi4_buff);

	/*
	 * Register 0x0A, 0x0B: IOCON: I/O EXPANDER CONFIGURATION REGISTER
	 * Set IO Expander Configuration
	 * MIRROR: The INT pins are internally connected
	 */
	mo4_buff[0] = 0x400A;
	mo4_buff[1] = 0x4040;
	/* Write simple message over SPI */
	WriteSpi1Mssg(mo4_buff, sizeof(mo4_buff) / sizeof(mo4_buff[0]), 0,
			mi4_buff);

	/*
	 * Register 0x04, 0x05: GPINTEN: INTERRUPT-ON-CHANGE PINS
	 * Enables GPIO input pin for interrupt-on-change event for both GPIOA and GPIOB
	 */
	mo4_buff[0] = 0x4004;
	mo4_buff[1] = 0x1F1F;
	/* Write simple message over SPI */
	WriteSpi1Mssg(mo4_buff, sizeof(mo4_buff) / sizeof(mo4_buff[0]), 0,
			mi4_buff);
}

void setup_IO_Expander_Buttons(void) {
	/*
	 * Register 0x00, 0x01: IODIR: I/O DIRECTION REGISTER
	 * Set bits 0-4 GPIOA and GPIOB direction to inputs, bits 5-7 to outputs
	 */
	mo5_buff[0] = 0x4000;
	mo5_buff[1] = 0xFFFF;
	/* Write simple message over SPI */
	WriteSpi1Mssg(mo5_buff, sizeof(mo5_buff) / sizeof(mo5_buff[0]), 1,
			mi5_buff);

	/*
	 * Register 0x02, 0x03: IPOL: INPUT POLARITY PORT REGISTER
	 * Set bits 0-4 GPIOA and GPIOB input polarity to be inverted
	 * LOGIC LOW = 5V, LOGIC HIGH = 0V
	 */
	mo5_buff[0] = 0x4002;
	mo5_buff[1] = 0xFF07;
	/* Write simple message over SPI */
	WriteSpi1Mssg(mo5_buff, sizeof(mo5_buff) / sizeof(mo5_buff[0]), 1,
			mi5_buff);

	/*
	 * Register 0x0C, 0x0D: GPPU: GPIO PULL-UP RESISTOR REGISTER
	 * Set bits 0-4 GPIOA and GPIOB pull-up resistors
	 * NORMAL STATE = 5V, PRESSED STATE = 0V
	 */
	mo5_buff[0] = 0x400C;
	mo5_buff[1] = 0xFF07;
	/* Write simple message over SPI */
	WriteSpi1Mssg(mo5_buff, sizeof(mo5_buff) / sizeof(mo5_buff[0]), 1,
			mi5_buff);

	/*
	 * Register 0x06, 0x07: DEFVAL: DEFAULT VALUE REGISTER
	 * Set GPIOA and GPIOB input default interrupt-on-change compare value to be zero
	 */
	mo5_buff[0] = 0x4006;
	mo5_buff[1] = 0x0000;
	/* Write simple message over SPI */
	WriteSpi1Mssg(mo5_buff, sizeof(mo5_buff) / sizeof(mo5_buff[0]), 1,
			mi5_buff);

	/*
	 * Register 0x08, 0x09: INTCON: INTERRUPT-ON-CHANGE CONTROL REGISTER
	 * Set bits 0-4 GPIOA and GPIOB input interrupt-on-change control register
	 * Pin value is compared against the associated bit in the DEFVAL register
	 */
	mo5_buff[0] = 0x4008;
	//mo5_buff[1] = 0xFF07;
	mo5_buff[1] = 0x0000;
	/* Write simple message over SPI */
	WriteSpi1Mssg(mo5_buff, sizeof(mo5_buff) / sizeof(mo5_buff[0]), 1,
			mi5_buff);

	/*
	 * Register 0x0A, 0x0B: IOCON: I/O EXPANDER CONFIGURATION REGISTER
	 * Set IO Expander Configuration
	 * MIRROR: The INT pins are internally connected
	 */
	mo5_buff[0] = 0x400A;
	mo5_buff[1] = 0x4040;
	/* Write simple message over SPI */
	WriteSpi1Mssg(mo5_buff, sizeof(mo5_buff) / sizeof(mo5_buff[0]), 1,
			mi5_buff);

	/*
	 * Register 0x04, 0x05: GPINTEN: INTERRUPT-ON-CHANGE PINS
	 * Enables GPIO input pin for interrupt-on-change event for both GPIOA and GPIOB
	 */
	mo5_buff[0] = 0x4004;
	mo5_buff[1] = 0xFF07;
	/* Write simple message over SPI */
	WriteSpi1Mssg(mo5_buff, sizeof(mo5_buff) / sizeof(mo5_buff[0]), 1,
			mi5_buff);
}

void clear_Pin_Int0(void) {
	/*
	 * Register 0x10, 0x11: INTCAP: INTERRUPT CAPTURED VALUE FOR PORT REGISTER
	 * Reflects the logic level on the port pins at the time of interrupt due to pin change for both GPIOA and GPIOB
	 */
	mo4_buff[0] = 0x4110;
	mo4_buff[1] = 0x0000;
	/* Write simple message over SPI */
	WriteSpi1Mssg(mo4_buff, sizeof(mo4_buff) / sizeof(mo4_buff[0]), 0,
			mi4_buff);
}

void clear_Pin_Int1(void) {
	/*
	 * Register 0x10, 0x11: INTCAP: INTERRUPT CAPTURED VALUE FOR PORT REGISTER
	 * Reflects the logic level on the port pins at the time of interrupt due to pin change for both GPIOA and GPIOB
	 */
	mo5_buff[0] = 0x4110;
	mo5_buff[1] = 0x0000;
	/* Write simple message over SPI */
	WriteSpi1Mssg(mo5_buff, sizeof(mo5_buff) / sizeof(mo5_buff[0]), 1,
			mi5_buff);
}

void SCT_Init(void) {
	/* Enable SCT peripheral */
	Chip_SCT_Init(LPC_SCT0);		//start pwm large timer
	/* Enable the clock to the Switch Matrix */
	Chip_Clock_EnablePeriphClock(SYSCTL_CLOCK_SWM);
	//Chip_SWM_DisableFixedPin(SWM_FIXED_ACMP_I1);//disable fixed pin on pin 0_27
	Chip_SWM_MovablePortPinAssign(SWM_SCT0_OUT0_O, 0, 27);//assign movable function to the pin (0_27)
	/* Disable the clock to the Switch Matrix to save power */
	Chip_Clock_DisablePeriphClock(SYSCTL_CLOCK_SWM);

	/* Initialize SCT0 registers for simple square waves */
	LPC_SCT0->CONFIG |= 1; // unified timer
	LPC_SCT0->MATCHREL[0].U = (SystemCoreClock / (10 * 2)) - 1; // match 0 @ 20 Hz = 50 msec
																// doing square wave of 10Hz
	LPC_SCT0->EVENT[0].STATE = (1 << 0); // event 0 only happens in state 0
	LPC_SCT0->EVENT[0].CTRL = (0 << 0) | // related to match 0
			(1 << 12) | // COMBMODE[13:12] = match condition only
			(1 << 14) | // STATELD[14] = STATEV is loaded into state
			(1 << 15); // STATEV[15] = 1 (new state is 1)
	LPC_SCT0->EVENT[1].STATE = (1 << 1); // event 1 only happens in state 1
	LPC_SCT0->EVENT[1].CTRL = (0 << 0) | // related to match 0
			(1 << 12) | // COMBMODE[13:12] = match condition only
			(1 << 14) | // STATELD[14] = STATEV is loaded into state
			(0 << 15); // STATEV[15] = 0 (new state is 0)
	LPC_SCT0->OUT[0].SET = (1 << 0); // event 0 will set SCT_OUT0
	LPC_SCT0->OUT[0].CLR = (1 << 1); // event 1 will clear SCT_OUT0
	LPC_SCT0->LIMIT_L = 0x0003; // events 0 and 1 are used as counter limit
	//LPC_SCT0->CTRL_L &= ~(1 << 2); // unhalt by clearing bit 2 of CTRL register
}
/*************************************************************************************/

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief	Handle interrupt from GPIO pin or GPIO pin mapped to PININT
 * @return	Nothing
 */
void PIN_INT0_IRQHandler(void) {
	// This used to check if a context switch is required
	portBASE_TYPE xHigherPriorityWoken = pdFALSE;
	Chip_PININT_ClearIntStatus(LPC_GPIO_PIN_INT, PININTCH0);
	Board_LED_Toggle(0);
	// Give semaphore and set context switch flag if a higher priority task was woken up
	xSemaphoreGiveFromISR(clearPININT0, &xHigherPriorityWoken);
	// End the ISR and (possibly) do a context switch
	portEND_SWITCHING_ISR(xHigherPriorityWoken);
}

/**
 * @brief	Handle interrupt from GPIO pin or GPIO pin mapped to PININT
 * @return	Nothing
 */
void PIN_INT1_IRQHandler(void) {
	// This used to check if a context switch is required
	portBASE_TYPE xHigherPriorityWoken = pdFALSE;
	Chip_PININT_ClearIntStatus(LPC_GPIO_PIN_INT, PININTCH1);
	Board_LED_Toggle(1);
	// Give semaphore and set context switch flag if a higher priority task was woken up
	xSemaphoreGiveFromISR(clearPININT1, &xHigherPriorityWoken);
	// End the ISR and (possibly) do a context switch
	portEND_SWITCHING_ISR(xHigherPriorityWoken);
}

/**
 * @brief	UART interrupt handler using ring buffers
 * @return	Nothing
 */
void LPC_UARTHNDLR(void) {
	/* Want to handle any errors? Do it here. */
	// This used to check if a context switch is required
	portBASE_TYPE xHigherPriorityWoken = pdFALSE;

	/* Use default ring buffer handler. Override this with your own
	 code if you need more capability. */
	Chip_UART_IRQRBHandler(LPC_USART, &rxring, &txring);

	// Give semaphore and set context switch flag if a higher priority task was woken up
	xSemaphoreGiveFromISR(uartSem, &xHigherPriorityWoken);
	// End the ISR and (possibly) do a context switch
	portEND_SWITCHING_ISR(xHigherPriorityWoken);

}

#ifdef __cplusplus
}
#endif

/*************************************************************************************/

void loadConfig(void) {
	gameLengthV = eeprom_buff[1];
	litBeforeHitV = eeprom_buff[2];
	litAfterHitV = eeprom_buff[3];
	numberOfTargetsV = eeprom_buff[4];
	smallTargetPointV = eeprom_buff[5];
	mediumTargetPointV = eeprom_buff[6];
	largeTargetPointV = eeprom_buff[7];
	highScoreToggleV = eeprom_buff[8];
	timeLeftToggleV = eeprom_buff[9];
	soundToggleV = eeprom_buff[10];
}

void bootUpLoad(void) {
	eeprom_buff[0] = 1;
	eeprom_Load(eeprom_buff);
	loadConfig();
	uint8_t highScoreBuff[2];
	Chip_EEPROM_Read(HIGH_SCORE_START_ADD, highScoreBuff, HIGH_SCORE_BYTES);
	highScore = (highScoreBuff[0] << 8) | highScoreBuff[1];
}

void greenLED(unsigned int target, bool state) {
	char debug[20];
	sprintf(debug, "Green LED: %d, %d\r\n", target, state);
	ITM_write(debug);
}

void redLED(unsigned int target, bool state) {
	char debug[20];
	sprintf(debug, "Red LED: %d, %d\r\n", target, state);
	ITM_write(debug);
}

void soundStart(void) {
	xSemaphoreGive(soundStartSem);
}

void soundHit(void) {
	xSemaphoreGive(soundHitSem);
}

void soundMiss(void) {
	xSemaphoreGive(soundMissSem);
}

void soundEnd(void) {
	xSemaphoreGive(soundEndSem);
}

void increaseScore(unsigned int target) {
	if ((target >= 1) && (target <= 3)) {
		currentScore += smallTargetPointV;
	} else if ((target >= 4) && (target <= 6)) {
		currentScore += mediumTargetPointV;
	} else if ((target >= 7) && (target <= 10)) {
		currentScore += largeTargetPointV;
	}
	if (currentScore > highScore) {
		highScore = currentScore;
	}
}

void saveHighScore(void) {
	static int lastHighScore;
	static uint8_t highScoreBuff[2];
	Chip_EEPROM_Read(HIGH_SCORE_START_ADD, highScoreBuff,
	HIGH_SCORE_BYTES);
	lastHighScore = (highScoreBuff[0] << 8) | highScoreBuff[1];
	if (highScore > lastHighScore) {
		highScoreBuff[0] = highScore >> 8;
		highScoreBuff[1] = highScore & 0x00FF;
		Chip_EEPROM_Write(HIGH_SCORE_START_ADD, highScoreBuff,
		HIGH_SCORE_BYTES);
	}
}

/*************************************************************************************/

void vCalibrationTask(void *pvParameters) {
	Board_LED_Set(0, true);
	Board_LED_Set(1, true);
	Board_LED_Set(2, true);
	bool doneCal = false;
	unsigned int num = 2;
	int step = 0;

	QueueItem i;
	LiquidCrystal lcd(0, 1, 2, 3, 4, 5);
	lcd.begin(16, 2);
	char s[16];
	//snprintf(s, 16, "[%4d]", edit);
	while (1) {
		if (!doneCal) {
			while (step == 0) {
				lcd.clear();
				lcd.setCursor(0, 0);
				lcd.Print("How many");
				lcd.setCursor(0, 1);
				snprintf(s, 16, "  targets?  %2d", num);
				lcd.Print(s);
				if (xQueue != 0) {
					if (xQueueReceive(xQueue, &(i),
							(TickType_t ) portMAX_DELAY)) {
						if (i.type == QueueItem::Button) {
							if (i.value == BUTTON_UP) {
								if (num < 10) {
									num++;
								}
							} else if (i.value == BUTTON_DOWN) {
								if (num > 2) {
									num--;
								}
							} else if (i.value == BUTTON_OK) {
								step = 1;
								greenLED(step, true);
								lcd.clear();
								lcd.setCursor(0, 0);
								lcd.Print("Tap target");
								lcd.setCursor(0, 1);
								snprintf(s, 16, "  number:  %2d", step);
								lcd.Print(s);
							}
						}
					}
				}
			}

			while (step == 1) {
				if (xQueue != 0) {
					if (xQueueReceive(xQueue, &(i),
							(TickType_t ) portMAX_DELAY)) {
						if ((i.type == QueueItem::Target)
								&& (i.value == TARGET_1)) {
							step = 2;
							greenLED((step - 1), false);
							redLED((step - 1), true);
							greenLED(step, true);
							lcd.clear();
							lcd.setCursor(0, 0);
							lcd.Print("Tap target");
							lcd.setCursor(0, 1);
							snprintf(s, 16, "  number:  %2d", step);
							lcd.Print(s);
						}
					}
				}
			}

			while (step == 2) {
				if (xQueue != 0) {
					if (xQueueReceive(xQueue, &(i),
							(TickType_t ) portMAX_DELAY)) {
						if ((i.type == QueueItem::Target)
								&& (i.value == TARGET_2)) {
							if (num >= 3) {
								step = 3;
								redLED((step - 2), false);
								greenLED((step - 1), false);
								redLED((step - 1), true);
								greenLED(step, true);
								lcd.clear();
								lcd.setCursor(0, 0);
								lcd.Print("Tap target");
								lcd.setCursor(0, 1);
								snprintf(s, 16, "  number:  %2d", step);
								lcd.Print(s);
							} else {
								break;
							}
						}
					}
				}
			}

			while (step == 3) {
				if (xQueue != 0) {
					if (xQueueReceive(xQueue, &(i),
							(TickType_t ) portMAX_DELAY)) {
						if ((i.type == QueueItem::Target)
								&& (i.value == TARGET_3)) {
							if (num >= 4) {
								step = 4;
								redLED((step - 2), false);
								greenLED((step - 1), false);
								redLED((step - 1), true);
								greenLED(step, true);
								lcd.clear();
								lcd.setCursor(0, 0);
								lcd.Print("Tap target");
								lcd.setCursor(0, 1);
								snprintf(s, 16, "  number:  %2d", step);
								lcd.Print(s);
							} else {
								break;
							}
						}
					}
				}
			}

			while (step == 4) {
				if (xQueue != 0) {
					if (xQueueReceive(xQueue, &(i),
							(TickType_t ) portMAX_DELAY)) {
						if ((i.type == QueueItem::Target)
								&& (i.value == TARGET_4)) {
							if (num >= 5) {
								step = 5;
								redLED((step - 2), false);
								greenLED((step - 1), false);
								redLED((step - 1), true);
								greenLED(step, true);
								lcd.clear();
								lcd.setCursor(0, 0);
								lcd.Print("Tap target");
								lcd.setCursor(0, 1);
								snprintf(s, 16, "  number:  %2d", step);
								lcd.Print(s);
							} else {
								break;
							}
						}
					}
				}
			}

			while (step == 5) {
				if (xQueue != 0) {
					if (xQueueReceive(xQueue, &(i),
							(TickType_t ) portMAX_DELAY)) {
						if ((i.type == QueueItem::Target)
								&& (i.value == TARGET_5)) {
							if (num >= 6) {
								step = 6;
								redLED((step - 2), false);
								greenLED((step - 1), false);
								redLED((step - 1), true);
								greenLED(step, true);
								lcd.clear();
								lcd.setCursor(0, 0);
								lcd.Print("Tap target");
								lcd.setCursor(0, 1);
								snprintf(s, 16, "  number:  %2d", step);
								lcd.Print(s);
							} else {
								break;
							}
						}
					}
				}
			}

			while (step == 6) {
				if (xQueue != 0) {
					if (xQueueReceive(xQueue, &(i),
							(TickType_t ) portMAX_DELAY)) {
						if ((i.type == QueueItem::Target)
								&& (i.value == TARGET_6)) {
							if (num >= 7) {
								step = 7;
								redLED((step - 2), false);
								greenLED((step - 1), false);
								redLED((step - 1), true);
								greenLED(step, true);
								lcd.clear();
								lcd.setCursor(0, 0);
								lcd.Print("Tap target");
								lcd.setCursor(0, 1);
								snprintf(s, 16, "  number:  %2d", step);
								lcd.Print(s);
							} else {
								break;
							}
						}
					}
				}
			}

			while (step == 7) {
				if (xQueue != 0) {
					if (xQueueReceive(xQueue, &(i),
							(TickType_t ) portMAX_DELAY)) {
						if ((i.type == QueueItem::Target)
								&& (i.value == TARGET_7)) {
							if (num >= 8) {
								step = 8;
								redLED((step - 2), false);
								greenLED((step - 1), false);
								redLED((step - 1), true);
								greenLED(step, true);
								lcd.clear();
								lcd.setCursor(0, 0);
								lcd.Print("Tap target");
								lcd.setCursor(0, 1);
								snprintf(s, 16, "  number:  %2d", step);
								lcd.Print(s);
							} else {
								break;
							}
						}
					}
				}
			}

			while (step == 8) {
				if (xQueue != 0) {
					if (xQueueReceive(xQueue, &(i),
							(TickType_t ) portMAX_DELAY)) {
						if ((i.type == QueueItem::Target)
								&& (i.value == TARGET_8)) {
							if (num >= 9) {
								step = 9;
								redLED((step - 2), false);
								greenLED((step - 1), false);
								redLED((step - 1), true);
								greenLED(step, true);
								lcd.clear();
								lcd.setCursor(0, 0);
								lcd.Print("Tap target");
								lcd.setCursor(0, 1);
								snprintf(s, 16, "  number:  %2d", step);
								lcd.Print(s);
							} else {
								break;
							}
						}
					}
				}
			}

			while (step == 9) {
				if (xQueue != 0) {
					if (xQueueReceive(xQueue, &(i),
							(TickType_t ) portMAX_DELAY)) {
						if ((i.type == QueueItem::Target)
								&& (i.value == TARGET_9)) {
							if (num >= 10) {
								step = 10;
								redLED((step - 2), false);
								greenLED((step - 1), false);
								redLED((step - 1), true);
								greenLED(step, true);
								lcd.clear();
								lcd.setCursor(0, 0);
								lcd.Print("Tap target");
								lcd.setCursor(0, 1);
								snprintf(s, 16, "  number:  %2d", step);
								lcd.Print(s);
							} else {
								break;
							}
						}
					}
				}
			}

			while (step == 10) {
				if (xQueue != 0) {
					if (xQueueReceive(xQueue, &(i),
							(TickType_t ) portMAX_DELAY)) {
						if ((i.type == QueueItem::Target)
								&& (i.value == TARGET_10)) {
							break;
						}
					}
				}
			}

			step++;
			redLED((step - 2), false);
			greenLED((step - 1), false);
			redLED((step - 1), true);
			lcd.clear();
			lcd.setCursor(0, 0);
			lcd.Print("Reset HighScore?");
			lcd.setCursor(0, 1);
			lcd.Print("OK = Y, BACK = N");
			while (1) {
				if (xQueue != 0) {
					if (xQueueReceive(xQueue, &(i),
							(TickType_t ) portMAX_DELAY)) {
						if (i.type == QueueItem::Button) {
							if (i.value == BUTTON_OK) {
								uint8_t highScoreBuff[2];
								highScoreBuff[0] = 0x00;
								highScoreBuff[1] = 0x00;
								Chip_EEPROM_Write(HIGH_SCORE_START_ADD,
										highScoreBuff,
										HIGH_SCORE_BYTES);
								break;
							} else if (i.value == BUTTON_BACK) {
								break;
							}
						}
					}
				}
			}
			redLED((step - 1), false);
			doneCal = true;
			lcd.clear();
			lcd.setCursor(0, 0);
			lcd.Print("Calibration:Done");
			lcd.setCursor(0, 1);
			lcd.Print("Please Restart");
			Board_LED_Set(0, true);
			Board_LED_Set(1, true);
			Board_LED_Set(2, true);
		}

		else {
			Board_LED_Toggle(0);
			Board_LED_Toggle(1);
			Board_LED_Toggle(2);
			vTaskDelay(configTICK_RATE_HZ);
		}
	}
}
/*
 switch (index) {
 case 1:
 if (i.value == TARGET_1) {
 index++;
 }
 break;
 case 2:
 if (i.value == TARGET_2) {
 index++;
 }
 break;
 case 3:
 if (i.value == TARGET_3) {
 index++;
 }
 break;
 case 4:
 if (i.value == TARGET_4) {
 index++;
 }
 break;
 case 5:
 if (i.value == TARGET_5) {
 index++;
 }
 break;
 case 6:
 if (i.value == TARGET_6) {
 index++;
 }
 break;
 case 7:
 if (i.value == TARGET_7) {
 index++;
 }
 break;
 case 8:
 if (i.value == TARGET_8) {
 index++;
 }
 break;
 case 9:
 if (i.value == TARGET_9) {
 index++;
 }
 break;
 case 10:
 if (i.value == TARGET_10) {
 index++;
 }
 break;
 }*/

void vTargetSendToQueueTask(void *pvParameters) {
	uint16_t target, interruptFlag;
	QueueItem i;
	while (1) {
		if (xSemaphoreTake(clearPININT0, portMAX_DELAY) == pdTRUE) {
			//vTaskDelay(configTICK_RATE_HZ);
			/*Read which pin cause the interrupt*/
			mo4_buff[0] = 0x410E;
			mo4_buff[1] = 0x0000;
			/* Write simple message over SPI */
			WriteSpi1Mssg(mo4_buff, sizeof(mo4_buff) / sizeof(mo4_buff[0]), 0,
					mi4_buff);
			interruptFlag = mi4_buff[1];
			/*Read current state of pins to clear pending interrupt signal*/
			clear_Pin_Int0();
			/*Compare the pin that cause the interrupt with its current state to determine if from 0 to 1 or from 1 to 0*/
			target = interruptFlag & mi4_buff[1];
			/*Send to queue if from 0 to 1*/
			if (target != 0) {
				i.type = QueueItem::Target;
				i.value = target;
				xQueueSendToBack(xQueue, (void * ) &i, (TickType_t ) 0);
			}
		}
		vTaskDelay(configTICK_RATE_HZ / 10);
	}
}

void vButtonSendToQueueTask(void *pvParameters) {
	uint16_t button, interruptFlag;
	QueueItem i;
	while (1) {
		if (xSemaphoreTake(clearPININT1, portMAX_DELAY) == pdTRUE) {
			//vTaskDelay(configTICK_RATE_HZ);
			mo5_buff[0] = 0x410E;
			mo5_buff[1] = 0x0000;
			/* Write simple message over SPI */
			WriteSpi1Mssg(mo5_buff, sizeof(mo5_buff) / sizeof(mo5_buff[0]), 1,
					mi5_buff);
			interruptFlag = mi5_buff[1];
			clear_Pin_Int1();
			button = interruptFlag & mi5_buff[1];
			if (button != 0) {
				i.type = QueueItem::Button;
				i.value = button;
				xQueueSendToBack(xQueue, (void * ) &i, (TickType_t ) 0);
			}
		}
		vTaskDelay(configTICK_RATE_HZ / 10);
	}
}

void vPlayStartSoundTask(void *pvParameters) {
	char debug[50];
	while (1) {
		if (xSemaphoreTake(soundStartSem, portMAX_DELAY) == pdTRUE) {
			if (xSemaphoreTake(SoundMutex, portMAX_DELAY) == pdTRUE) {
				sprintf(debug, "Start game sound playing.\r\n");
				ITM_write(debug);

				LPC_SCT0->CTRL_L &= ~(1 << 2); // unhalt by clearing bit 2 of CTRL register
				LPC_SCT0->MATCHREL[0].U = (SystemCoreClock / (698.46 * 2)) - 1;
				vTaskDelay(250);
				LPC_SCT0->MATCHREL[0].U = (SystemCoreClock / (783.99 * 2)) - 1;
				vTaskDelay(250);
				LPC_SCT0->MATCHREL[0].U = (SystemCoreClock / (880.00 * 2)) - 1;
				vTaskDelay(250);
				LPC_SCT0->CTRL_L |= (1 << 2); // halt by setting bit 2 of CTRL register

				xSemaphoreGive(SoundMutex);
			}
		}
	}
}

void vPlayHitSoundTask(void *pvParameters) {
	char debug[50];
	while (1) {
		if (xSemaphoreTake(soundHitSem, portMAX_DELAY) == pdTRUE) {
			if (xSemaphoreTake(SoundMutex, portMAX_DELAY) == pdTRUE) {
				sprintf(debug, "Hit sound playing.\r\n");
				ITM_write(debug);

				LPC_SCT0->CTRL_L &= ~(1 << 2); // unhalt by clearing bit 2 of CTRL register
				LPC_SCT0->MATCHREL[0].U = (SystemCoreClock / (698.46 * 2)) - 1;
				vTaskDelay(125);
				LPC_SCT0->CTRL_L |= (1 << 2); // halt by setting bit 2 of CTRL register
				vTaskDelay(125);
				LPC_SCT0->CTRL_L &= ~(1 << 2); // unhalt by clearing bit 2 of CTRL register
				LPC_SCT0->MATCHREL[0].U = (SystemCoreClock / (698.46 * 2)) - 1;
				vTaskDelay(125);
				LPC_SCT0->CTRL_L |= (1 << 2); // halt by setting bit 2 of CTRL register

				xSemaphoreGive(SoundMutex);
			}
		}
	}
}

void vPlayMissSoundTask(void *pvParameters) {
	char debug[50];
	while (1) {
		if (xSemaphoreTake(soundMissSem, portMAX_DELAY) == pdTRUE) {
			if (xSemaphoreTake(SoundMutex, portMAX_DELAY) == pdTRUE) {
				sprintf(debug, "Miss sound playing.\r\n");
				ITM_write(debug);

				LPC_SCT0->CTRL_L &= ~(1 << 2); // unhalt by clearing bit 2 of CTRL register
				LPC_SCT0->MATCHREL[0].U = (SystemCoreClock / (87.31 * 2)) - 1;
				vTaskDelay(1000);
				LPC_SCT0->CTRL_L |= (1 << 2); // halt by setting bit 2 of CTRL register

				xSemaphoreGive(SoundMutex);
			}
		}
	}
}

void vPlayEndSoundTask(void *pvParameters) {
	char debug[50];
	while (1) {
		if (xSemaphoreTake(soundEndSem, portMAX_DELAY) == pdTRUE) {
			if (xSemaphoreTake(SoundMutex, portMAX_DELAY) == pdTRUE) {
				sprintf(debug, "End game sound playing.\r\n");
				ITM_write(debug);

				LPC_SCT0->CTRL_L &= ~(1 << 2); // unhalt by clearing bit 2 of CTRL register
				LPC_SCT0->MATCHREL[0].U = (SystemCoreClock / (698.46 * 2)) - 1;
				vTaskDelay(250);
				LPC_SCT0->MATCHREL[0].U = (SystemCoreClock / (783.99 * 2)) - 1;
				vTaskDelay(250);
				LPC_SCT0->MATCHREL[0].U = (SystemCoreClock / (698.46 * 2)) - 1;
				vTaskDelay(250);
				LPC_SCT0->CTRL_L |= (1 << 2); // halt by setting bit 2 of CTRL register

				xSemaphoreGive(SoundMutex);
			}
		}
	}
}

void vCanonButtonTask(void *pvParameters) {
	char debug[50];
	Chip_IOCON_PinMuxSet(LPC_IOCON, 0, 24,
	IOCON_DIGMODE_EN | IOCON_MODE_PULLUP | IOCON_INV_EN);
	Chip_GPIO_SetPinDIRInput(LPC_GPIO, 0, 24);

	Chip_IOCON_PinMuxSet(LPC_IOCON, 1, 0,
	IOCON_MODE_INACT | IOCON_DIGMODE_EN);
	Chip_GPIO_SetPinDIROutput(LPC_GPIO, 1, 0);
	Chip_GPIO_SetPinState(LPC_GPIO, 1, 0, false);

	bool old = false;
	bool now = false;

	while (1) {
		now = Chip_GPIO_GetPinState(LPC_GPIO, 0, 24);
		if (now != old) {
			if (now) {
				if (canVal) {
					sprintf(debug, "Canon shooting.\r\n");
					ITM_write(debug);
					Chip_GPIO_SetPinState(LPC_GPIO, 1, 0, true);
					vTaskDelay(150);
					Chip_GPIO_SetPinState(LPC_GPIO, 1, 0, false);
				}
			}
			old = now;
		}
		//Chip_GPIO_SetPinState(LPC_GPIO, 1, 0, now);
		vTaskDelay(configTICK_RATE_HZ / 10);
	}
}

static void vBigDisplaysTask(void *pvParameters) {
	uint8_t digit1 = 0, digit2 = 0, digit3 = 0, digit4 = 0, digit5 = 0, digit6 =
			0, digit7 = 0, digit8 = 0, digit9 = 0, digit10 = 0, digit11 = 0,
			digit12 = 0;
	while (1) {
//if (currentScore
//	!= (digit1 * 1000 + digit2 * 100 + digit3 * 10 + digit4)) {
		digit1 = currentScore / 1000;
		digit2 = (currentScore % 1000) / 100;
		digit3 = (currentScore % 100) / 10;
		digit4 = currentScore % 10;
		/*
		 * Register 0x12, 0x13: GPIO: GENERAL PURPOSE I/O PORT REGISTER
		 * Write GPIO outputs to both GPIOA and GPIOB
		 */
		mo0_buff[0] = 0x4012;
		mo0_buff[1] = ((0xF & digit1) << 12) | ((0xF & digit2) << 8)
				| ((0xF & digit3) << 4) | (0xF & digit4);
//		((0x0F & (uint8_t) digit1) << 8) | ((uint8_t) 0x00);
		/* Write simple message over SPI */
		WriteSpi0Mssg(mo0_buff, sizeof(mo0_buff) / sizeof(mo0_buff[0]), 0,
				mi0_buff);
//}

//if (timeLeft != (digit5 * 1000 + digit6 * 100 + digit7 * 10 + digit8)) {
		digit5 = timeLeft / 1000;
		digit6 = (timeLeft % 1000) / 100;
		digit7 = (timeLeft % 100) / 10;
		digit8 = timeLeft % 10;
		if (timeLeftToggleV) {
			/*
			 * Register 0x12, 0x13: GPIO: GENERAL PURPOSE I/O PORT REGISTER
			 * Write GPIO outputs to both GPIOA and GPIOB
			 */
			mo1_buff[0] = 0x4012;
			mo1_buff[1] = ((0xF & digit5) << 12) | ((0xF & digit6) << 8)
					| ((0xF & digit7) << 4) | (0xF & digit8);
			//		((0x0F & (uint8_t) digit1) << 8) | ((uint8_t) 0x00);
			/* Write simple message over SPI */
			WriteSpi0Mssg(mo1_buff, sizeof(mo1_buff) / sizeof(mo1_buff[0]), 1,
					mi1_buff);
		} else {
			/*
			 * Register 0x12, 0x13: GPIO: GENERAL PURPOSE I/O PORT REGISTER
			 * Write GPIO outputs to both GPIOA and GPIOB
			 */
			mo1_buff[0] = 0x4012;
			mo1_buff[1] = 0xFFFF;
			//		((0x0F & (uint8_t) digit1) << 8) | ((uint8_t) 0x00);
			/* Write simple message over SPI */
			WriteSpi0Mssg(mo1_buff, sizeof(mo1_buff) / sizeof(mo1_buff[0]), 1,
					mi1_buff);
		}
//	}

//	if (highScore
//		!= (digit9 * 1000 + digit10 * 100 + digit11 * 10 + digit12)) {
		digit9 = highScore / 1000;
		digit10 = (highScore % 1000) / 100;
		digit11 = (highScore % 100) / 10;
		digit12 = highScore % 10;
		if (highScoreToggleV) {
			/*
			 * Register 0x12, 0x13: GPIO: GENERAL PURPOSE I/O PORT REGISTER
			 * Write GPIO outputs to both GPIOA and GPIOB
			 */
			mo2_buff[0] = 0x4012;
			mo2_buff[1] = ((0xF & digit9) << 12) | ((0xF & digit10) << 8)
					| ((0xF & digit11) << 4) | (0xF & digit12);
			//		((0x0F & (uint8_t) digit1) << 8) | ((uint8_t) 0x00);
			/* Write simple message over SPI */
			WriteSpi0Mssg(mo2_buff, sizeof(mo2_buff) / sizeof(mo2_buff[0]), 2,
					mi2_buff);
		} else {
			/*
			 * Register 0x12, 0x13: GPIO: GENERAL PURPOSE I/O PORT REGISTER
			 * Write GPIO outputs to both GPIOA and GPIOB
			 */
			mo2_buff[0] = 0x4012;
			mo2_buff[1] = 0xFFFF;
			//		((0x0F & (uint8_t) digit1) << 8) | ((uint8_t) 0x00);
			/* Write simple message over SPI */
			WriteSpi0Mssg(mo2_buff, sizeof(mo2_buff) / sizeof(mo2_buff[0]), 2,
					mi2_buff);
		}
//	}
		vTaskDelay(configTICK_RATE_HZ / 5);
	}
}

static void vStateMachineTask(void *pvParameters) {
	char debug[50];
#if 1
	LiquidCrystal lcd(0, 1, 2, 3, 4, 5);
	lcd.begin(16, 2);
	BarGraph bg50px(lcd, 50);

	/*Menu Variables*/
	SimpleMenu menu;

	StatusEdit readyMenu(lcd, std::string("Ready."), currentPreset);
	StatusEdit runningMenu(lcd, std::string("Running."), currentPreset);
	StatusEdit pausingMenu(lcd, std::string("Pausing."), currentPreset);

	IntegerEdit buttonToConfig(lcd, std::string("Button to configure"), 1, 4,
			bg50px);

	IntegerEdit gameLength(lcd, std::string("Game Time"), 0, 255, bg50px);
	IntegerEdit litBeforeHit(lcd, std::string("Lit before hit"), 0, 255,
			bg50px);
	IntegerEdit litAfterHit(lcd, std::string("Lit after hit"), 0, 255, bg50px);

	IntegerEdit numberOfTargets(lcd, std::string("Number of target"), 2, 10,
			bg50px);

	IntegerEdit smallTargetPoint(lcd, std::string("Small target pts"), 0, 255,
			bg50px);
	IntegerEdit mediumTargetPoint(lcd, std::string("Medium target pts"), 0, 255,
			bg50px);
	IntegerEdit largeTargetPoint(lcd, std::string("Large target pts"), 0, 255,
			bg50px);

	IntegerEdit highScoreToggle(lcd, std::string("High score toggle"), 0, 1,
			bg50px);
	IntegerEdit timeLeftToggle(lcd, std::string("Time left toggle"), 0, 1,
			bg50px);
	IntegerEdit soundToggle(lcd, std::string("Sound toggle"), 0, 1, bg50px);

	SaveEdit saveButton(lcd, std::string("Save setting?"), menu);

	menu.addItem(new MenuItem(readyMenu));
	menu.addItem(new MenuItem(runningMenu));
	menu.addItem(new MenuItem(pausingMenu));

	menu.addItem(new MenuItem(buttonToConfig));

	menu.addItem(new MenuItem(gameLength));
	menu.addItem(new MenuItem(litBeforeHit));
	menu.addItem(new MenuItem(litAfterHit));

	menu.addItem(new MenuItem(numberOfTargets));

	menu.addItem(new MenuItem(smallTargetPoint));
	menu.addItem(new MenuItem(mediumTargetPoint));
	menu.addItem(new MenuItem(largeTargetPoint));

	menu.addItem(new MenuItem(highScoreToggle));
	menu.addItem(new MenuItem(timeLeftToggle));
	menu.addItem(new MenuItem(soundToggle));

	menu.addItem(new MenuItem(saveButton));
#endif

	Event tick = { Event::eTick, 0 };
	Event button = { Event::eButton, 0 };
	Event target = { Event::eTarget, 0 };
	StateMachine fsm(readyMenu, runningMenu, pausingMenu, buttonToConfig,
			gameLength, litBeforeHit, litAfterHit, numberOfTargets,
			smallTargetPoint, mediumTargetPoint, largeTargetPoint,
			highScoreToggle, timeLeftToggle, soundToggle, saveButton, menu);

	QueueItem i;
	xTimerStart(xTimer1, 0);

	while (1) {
		if (xQueue != 0) {
			if (xQueueReceive(xQueue, &(i), (TickType_t ) portMAX_DELAY)) {
				sprintf(debug, "In the queue: %d %X.\r\n", i.type, i.value);
				ITM_write(debug);
				if (i.type == QueueItem::Tick) {
					fsm.HandleState(tick);
				} else if (i.type == QueueItem::Button) {
					button.value = i.value;
					fsm.HandleState(button);
				} else if (i.type == QueueItem::Target) {
					switch (i.value) {
					case TARGET_1:
						target.value = 1;
						break;
					case TARGET_2:
						target.value = 2;
						break;
					case TARGET_3:
						target.value = 3;
						break;
					case TARGET_4:
						target.value = 4;
						break;
					case TARGET_5:
						target.value = 5;
						break;
					case TARGET_6:
						target.value = 6;
						break;
					case TARGET_7:
						target.value = 7;
						break;
					case TARGET_8:
						target.value = 8;
						break;
					case TARGET_9:
						target.value = 9;
						break;
					case TARGET_10:
						target.value = 10;
						break;
					}
					fsm.HandleState(target);
				}
			}
		}
		vTaskDelay(configTICK_RATE_HZ / 10);
	}
}

/* UART thread */
static void vUARTTask(void *pvParameters) {

	uint8_t key;
	int bytes;
	int parameter_index = 0;
	char parameter[4];
	int number_of_parameters = 0;
	uint8_t configuration[11];
	char debug[50];

	/* Poll the receive ring buffer for the ESC (ASCII 27) key */
	key = 0;
	while (1) {
		if (xSemaphoreTake(uartSem, portMAX_DELAY) == pdTRUE) {
			while (number_of_parameters < 11) {

				bytes = Chip_UART_ReadRB(LPC_USART, &rxring, &key, 1);
				if (bytes > 0) {

					if ((key != 10) && (key != 13)) {

						parameter[parameter_index] = (char) key;
						parameter_index++;

					}

					else {

						//send ok to app saying recieved
						Chip_UART_SendRB(LPC_USART, &txring, "ok\n",
								sizeof("ok\n") - 1);

						//convert to int
						configuration[number_of_parameters] = atoi(parameter);

						//clear parameter && reset parameter_index
						memset(parameter, 0, 4);
						parameter_index = 0;

						//increase number_of_parameters
						number_of_parameters++;
					}
					//if full configuration found
					if (number_of_parameters == 11) {
						sprintf(debug,
								"Configuration: %d %d %d %d %d %d %d %d %d %d %d\n",
								configuration[0], configuration[1],
								configuration[2], configuration[3],
								configuration[4], configuration[5],
								configuration[6], configuration[7],
								configuration[8], configuration[9],
								configuration[10]);
						ITM_write(debug);
						//put into eeprom
						if (eeprom_Save(configuration)) {
							ITM_write(
									"Saving configuration was successful (from task)\n");
						} else
							ITM_write(
									"There was an issue saving configuration (from task)\n");
						//clear parameter && reset parameter_index
						memset(parameter, 0, 4);
						parameter_index = 0;

					}
				}
			}
			//clear parameter && reset parameter_index
			memset(parameter, 0, 4);
			parameter_index = 0;
			//reset parameter
			number_of_parameters = 0;
		}
		//vTaskDelay(configTICK_RATE_HZ/20);
	}
}

/*
 * Call back function for timer 1
 * 1 second timer
 * decrementing the timeLeft value by 1 each second for displaying
 * Auto reload: ON
 */
void vTimer1Callback(TimerHandle_t xTimer) {
	QueueItem i = { QueueItem::Tick, 0 };
	xQueueSendToBack(xQueue, (void * ) &i, (TickType_t ) 0);
}

/*
 *
 *
 */

int main(void) {

	prvSetupHardware();
	/*************************************************************************************/
	Chip_IOCON_PinMuxSet(LPC_IOCON, 0, 24,
			(IOCON_DIGMODE_EN | IOCON_MODE_PULLUP | IOCON_INV_EN));
	Chip_GPIO_SetPinDIRInput(LPC_GPIO, 0, 24);
	/*************************************************************************************/
	if (Chip_GPIO_GetPinState(LPC_GPIO, 0, 24) == true) {
		while (Chip_GPIO_GetPinState(LPC_GPIO, 0, 24) == true) {
		}
#if 1
		xTaskCreate(vTargetSendToQueueTask, "vTargetSendToQueueTask",
				configMINIMAL_STACK_SIZE, NULL, (tskIDLE_PRIORITY + 3UL),
				(TaskHandle_t *) NULL);
#endif
#if 1
		xTaskCreate(vButtonSendToQueueTask, "vButtonSendToQueueTask",
				configMINIMAL_STACK_SIZE, NULL, (tskIDLE_PRIORITY + 3UL),
				(TaskHandle_t *) NULL);
#endif
		xTaskCreate(vCalibrationTask, "vCalibrationTask",
				configMINIMAL_STACK_SIZE*2, NULL, (tskIDLE_PRIORITY + 1UL),
				(TaskHandle_t *) NULL);
	}
	/*************************************************************************************/
	else {

		xTimer1 = xTimerCreate("Timer1", 1000, pdTRUE, (void *) 0,
				vTimer1Callback);

#if 1
		xTaskCreate(vTargetSendToQueueTask, "vTargetSendToQueueTask",
				configMINIMAL_STACK_SIZE, NULL, (tskIDLE_PRIORITY + 3UL),
				(TaskHandle_t *) NULL);
#endif
#if 1
		xTaskCreate(vButtonSendToQueueTask, "vButtonSendToQueueTask",
				configMINIMAL_STACK_SIZE, NULL, (tskIDLE_PRIORITY + 3UL),
				(TaskHandle_t *) NULL);
#endif
#if 1
		xTaskCreate(vPlayStartSoundTask, "vPlayStartSoundTask",
				configMINIMAL_STACK_SIZE, NULL, (tskIDLE_PRIORITY + 2UL),
				(TaskHandle_t *) NULL);
#endif
#if 1
		xTaskCreate(vPlayHitSoundTask, "vPlayHitSoundTask",
				configMINIMAL_STACK_SIZE, NULL, (tskIDLE_PRIORITY + 2UL),
				(TaskHandle_t *) NULL);
#endif
#if 1
		xTaskCreate(vPlayMissSoundTask, "vPlayMissSoundTask",
				configMINIMAL_STACK_SIZE, NULL, (tskIDLE_PRIORITY + 2UL),
				(TaskHandle_t *) NULL);
#endif
#if 1
		xTaskCreate(vPlayEndSoundTask, "vPlayEndSoundTask",
				configMINIMAL_STACK_SIZE, NULL, (tskIDLE_PRIORITY + 2UL),
				(TaskHandle_t *) NULL);
#endif
#if 1
		xTaskCreate(vCanonButtonTask, "vCanonButtonTask",
				configMINIMAL_STACK_SIZE, NULL, (tskIDLE_PRIORITY + 2UL),
				(TaskHandle_t *) NULL);
#endif
#if 1
		xTaskCreate(vBigDisplaysTask, "vBigDisplaysTask",
				configMINIMAL_STACK_SIZE*6, NULL, (tskIDLE_PRIORITY + 1UL),
				(TaskHandle_t *) NULL);
#endif
#if 1
		xTaskCreate(vStateMachineTask, "vStateMachineTask",
				configMINIMAL_STACK_SIZE*6, NULL, (tskIDLE_PRIORITY + 1UL),
				(TaskHandle_t *) NULL);
#endif
#if 1
		xTaskCreate(vUARTTask, "vUartTask", configMINIMAL_STACK_SIZE, NULL,
				(tskIDLE_PRIORITY + 1UL), (TaskHandle_t *) NULL);
#endif
	}
	/*************************************************************************************/

	/* Start the scheduler */
	vTaskStartScheduler();

	/* Should never arrive here */
	return 1;

}
