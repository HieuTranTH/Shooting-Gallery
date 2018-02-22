/*
 * StatusEdit.h
 *
 *  Created on: Dec 8, 2017
 *      Author: macos
 */

#ifndef STATUSEDIT_H_
#define STATUSEDIT_H_

#include "PropertyEdit.h"
#include "LiquidCrystal.h"
#include "BarGraph.h"
#include <string>

class StatusEdit: public PropertyEdit {
public:
	StatusEdit(LiquidCrystal& lcd_, std::string editTitle, int& preset_);
	virtual ~StatusEdit();
	void increment();
	void decrement();
	void accept();
	void cancel();
	void setFocus(bool focus);
	void display();
	int getValue();
private:
	LiquidCrystal& lcd;
	std::string title;
	bool focus;
	int& preset;
};



#endif /* STATUSEDIT_H_ */
