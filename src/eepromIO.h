#include "chip.h"
#include <cstring>

#ifndef EEPROMIO_H_
#define EEPROMIO_H_

/* EEPROM Address used for storage */
#define BUTTON_1_START_ADD 0x00000000
#define BUTTON_2_START_ADD 0x0000000B
#define BUTTON_3_START_ADD 0x00000016
#define BUTTON_4_START_ADD 0x00000021

#define NUM_BYTES_TO_READ_WRITE 11

/* EEPROM Address used for storage */
#define BUTTON_1_START_ADD 0x00000000
#define BUTTON_2_START_ADD 0x0000000B
#define BUTTON_3_START_ADD 0x00000016
#define BUTTON_4_START_ADD 0x00000021
#define HIGH_SCORE_START_ADD 0x0000002C
#define RANDOM_SEED_START_ADD 0x00000021

#define NUM_BYTES_TO_READ_WRITE 11
#define HIGH_SCORE_BYTES 2
#define RANDOM_SEED_BYTES 2

extern uint8_t eeprom_buff[];

void eeprom_Setup(void);

bool eeprom_Save(uint8_t arr[]);

bool eeprom_Load(uint8_t *config_to_change);



#endif
