/*
 * StatusEdit.cpp
 *
 *  Created on: Dec 8, 2017
 *      Author: macos
 */

#include "StatusEdit.h"
#include <cstdio>

StatusEdit::StatusEdit(LiquidCrystal& lcd_, std::string editTitle, int& preset_) :
		lcd(lcd_), title(editTitle), preset(preset_){
	//preset = preset_;
}

StatusEdit::~StatusEdit() {
}

void StatusEdit::increment() {
}

void StatusEdit::decrement() {
}

void StatusEdit::accept() {

}

void StatusEdit::cancel() {
}

int StatusEdit::getValue() {
}

void StatusEdit::setFocus(bool focus) {
	this->focus = focus;
}

void StatusEdit::display() {
	lcd.clear();
	lcd.setCursor(0, 0);
	lcd.Print(title);
	lcd.setCursor(0, 1);
	char s[16];
	snprintf(s, 16, "Preset %d", preset);
	lcd.Print(s);
}

