/*
 * IntegerEdit.h
 *
 *  Created on: 2.2.2016
 *      Author: krl
 */

#ifndef INTEGEREDIT_H_
#define INTEGEREDIT_H_

#include "PropertyEdit.h"
#include "LiquidCrystal.h"
#include "BarGraph.h"
#include <string>

class IntegerEdit: public PropertyEdit {
public:
	IntegerEdit(LiquidCrystal& lcd_, std::string editTitle, int low_lim, int up_lim, BarGraph& bg_);
	virtual ~IntegerEdit();
	void increment();
	void decrement();
	void accept();
	void cancel();
	void setFocus(bool focus);
	void display();
	int getValue();
	void setValue(int value);
private:
	void save();
	void displayEditValue();
	LiquidCrystal& lcd;
	std::string title;
	int value;
	int edit;
	bool focus;
	int upper_lim;
	int lower_lim;
	BarGraph bg;
};

#endif /* INTEGEREDIT_H_ */
