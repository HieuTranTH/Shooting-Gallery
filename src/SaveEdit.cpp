/*
 * SaveEdit.cpp
 *
 *  Created on: Dec 8, 2017
 *      Author: macos
 */

#include "SaveEdit.h"
#include "eepromIO.h"

SaveEdit::SaveEdit(LiquidCrystal& lcd_, std::string editTitle,
		SimpleMenu& menu_) :
		lcd(lcd_), title(editTitle), menu(menu_) {
	//menu = &menu_;
}

SaveEdit::~SaveEdit() {
}

void SaveEdit::increment() {
}

void SaveEdit::decrement() {
}

void SaveEdit::accept() {
	for (int i = 0; i <= 10; i++) {
		eeprom_buff[i] = menu.items[3 + i]->getValue();
	}
	eeprom_Save(eeprom_buff);
}

void SaveEdit::cancel() {

}

int SaveEdit::getValue() {

}

void SaveEdit::setFocus(bool focus) {
	this->focus = focus;
}

void SaveEdit::display() {
	lcd.clear();
	lcd.setCursor(0, 0);
	lcd.Print(title);
	if (focus) {
		lcd.setCursor(0, 1);
		lcd.Print("Press OK to save");
	}

}
