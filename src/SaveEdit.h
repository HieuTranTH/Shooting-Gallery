/*
 * SaveEdit.h
 *
 *  Created on: Dec 8, 2017
 *      Author: macos
 */

#ifndef SAVEEDIT_H_
#define SAVEEDIT_H_

#include "PropertyEdit.h"
#include "LiquidCrystal.h"
#include "BarGraph.h"
#include "SimpleMenu.h"
#include <string>

class SaveEdit: public PropertyEdit {
public:
	SaveEdit(LiquidCrystal& lcd_, std::string editTitle, SimpleMenu& menu_);
	virtual ~SaveEdit();
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
	SimpleMenu& menu;
};



#endif /* SAVEEDIT_H_ */
