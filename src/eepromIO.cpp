/*
 * eepromIO.cpp
 *
 *  Created on: Jan 29, 2018
 *      Author: macos
 */

#include "eepromIO.h"

/*EEPROM saving buffer*/
uint8_t eeprom_buff[11];

void eeprom_Setup(void){
	/* Enable EEPROM clock and reset EEPROM controller */
	Chip_Clock_EnablePeriphClock(SYSCTL_CLOCK_EEPROM);
	Chip_SYSCTL_PeriphReset(RESET_EEPROM);
}

bool eeprom_Save(uint8_t arr[]){

	uint8_t ret_code = 1;

	switch(arr[0]){

	case 1:
		ret_code = Chip_EEPROM_Write(BUTTON_1_START_ADD, arr, NUM_BYTES_TO_READ_WRITE);
		break;

	case 2:
		ret_code = Chip_EEPROM_Write(BUTTON_2_START_ADD, arr, NUM_BYTES_TO_READ_WRITE);
		break;

	case 3:
		ret_code = Chip_EEPROM_Write(BUTTON_3_START_ADD, arr, NUM_BYTES_TO_READ_WRITE);
		break;

	case 4:
		ret_code = Chip_EEPROM_Write(BUTTON_4_START_ADD, arr, NUM_BYTES_TO_READ_WRITE);
		break;
	}

    //If the save was successful
	if (ret_code == IAP_CMD_SUCCESS) {
		//ITM_write("Save to EEPROM successful\n");
		return true;
	}

	else{
		//ITM_write("FAILURE! Could not save to eeprom\n");
		return false;
	}
}

bool eeprom_Load(uint8_t *config_to_change){

	uint8_t ret_code;
	uint8_t temp[11];

	switch(config_to_change[0]){

	case 1:
		ret_code = Chip_EEPROM_Read(BUTTON_1_START_ADD, temp, NUM_BYTES_TO_READ_WRITE);
		break;

	case 2:
		ret_code = Chip_EEPROM_Read(BUTTON_2_START_ADD, temp, NUM_BYTES_TO_READ_WRITE);
		break;

	case 3:
		ret_code = Chip_EEPROM_Read(BUTTON_3_START_ADD, temp, NUM_BYTES_TO_READ_WRITE);
		break;

	case 4:
		ret_code = Chip_EEPROM_Read(BUTTON_4_START_ADD, temp, NUM_BYTES_TO_READ_WRITE);
		break;
	}
	//If the read was successful, replace old configuration and return the new.
	if (ret_code == IAP_CMD_SUCCESS) {
		for(int i = 0 ;i<=11;i++){
			config_to_change[i] = temp[i];
		}
		//ITM_write("Load from EEPROM successful");
		return true;
	}
	//If the read was unsuccessful, return the old configuration.
	else{
		//ITM_write("FAILURE! Could not load from eeprom\n");
		return false;
	}

}




