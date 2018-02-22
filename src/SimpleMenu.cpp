/*
 * SimpleMenu.cpp
 *
 *  Created on: 3.2.2016
 *      Author: krl
 */

#include "SimpleMenu.h"

SimpleMenu::SimpleMenu() {
	position = 0;
}

SimpleMenu::~SimpleMenu() {
	// TODO Auto-generated destructor stub
}

void SimpleMenu::addItem(MenuItem *item) {
	items.push_back(item);
}

void SimpleMenu::event(MenuItem::menuEvent e) {
	if (items.size() <= 0)
		return;

	if (!items[position]->event(e)) {
		if (e == MenuItem::up)
			position++;
		else if (e == MenuItem::down)
			position--;

		if (position < 3)
			position = 14;
		if (position >= 15)
			position = 3;

		items[position]->event(MenuItem::show);
	}
}

void SimpleMenu::setPosition(int position) {
	if ((position >= 0) && (position < items.size())) {
		this->position = position;
	}
}
