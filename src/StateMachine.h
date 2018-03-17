/*
 * StateMachine.h
 *
 *  Created on: Feb 13, 2018
 *      Author: macos
 */
#ifndef STATEMACHINE_H_
#define STATEMACHINE_H_

#include "IntegerEdit.h"
#include "StatusEdit.h"
#include "SaveEdit.h"
#include "MenuItem.h"
#include "SimpleMenu.h"

#define BUTTON_PRESET_1 0x8000
#define BUTTON_PRESET_2 0x4000
#define BUTTON_PRESET_3 0x2000
#define BUTTON_PRESET_4 0x1000
#define BUTTON_UP 0x0800
#define BUTTON_DOWN 0x0400
#define BUTTON_OK 0x0200
#define BUTTON_BACK 0x0100
#define BUTTON_PLAY 0x0004
#define BUTTON_PAUSE 0x0002
#define BUTTON_STOP 0x0001

#define TARGET_1 0x1000
#define TARGET_2 0x0800
#define TARGET_3 0x0400
#define TARGET_4 0x0200
#define TARGET_5 0x0100
#define TARGET_6 0x0010
#define TARGET_7 0x0008
#define TARGET_8 0x0004
#define TARGET_9 0x0002
#define TARGET_10 0x0001

class Event {
public:
	enum eventType {
		eEnter, eExit, eButton, eTarget, eTick
	};
	eventType type;
	unsigned int value;

};

class StateMachine {
public:
	StateMachine(StatusEdit& readyMenu, StatusEdit& runningMenu,
	 StatusEdit& pausingMenu, IntegerEdit& buttonToConfig,
	 IntegerEdit& gameLength, IntegerEdit& litBeforeHit,
	 IntegerEdit& litAfterHit, IntegerEdit& numberOfTargets,
	 IntegerEdit& smallTargetPoint, IntegerEdit& mediumTargetPoint,
	 IntegerEdit& largeTargetPoint, IntegerEdit& highScoreToggle,
	 IntegerEdit& timeLeftToggle, IntegerEdit& soundToggle,
	 SaveEdit& saveButton, SimpleMenu& menu);
	virtual ~StateMachine();
	void HandleState(const Event& e);
private:
	enum state {
		Ready, Pause, Running0, Running1, Running2, Config
	};
	state currentState, lastState;
	static int timer;
	void HandleStateReady(const Event& e);
	void HandleStatePause(const Event& e);
	void HandleStateRunning0(const Event& e);
	void HandleStateRunning1(const Event& e);
	void HandleStateRunning2(const Event& e);
	void HandleStateConfig(const Event& e);
	void SetState(state newState);

	 StatusEdit& readyMenu;
	 StatusEdit& runningMenu;
	 StatusEdit& pausingMenu;

	 IntegerEdit& buttonToConfig;
	 IntegerEdit& gameLength;
	 IntegerEdit& litBeforeHit;
	 IntegerEdit& litAfterHit;

	 IntegerEdit& numberOfTargets;

	 IntegerEdit& smallTargetPoint;
	 IntegerEdit& mediumTargetPoint;
	 IntegerEdit& largeTargetPoint;

	 IntegerEdit& highScoreToggle;
	 IntegerEdit& timeLeftToggle;
	 IntegerEdit& soundToggle;

	 SaveEdit& saveButton;

	 SimpleMenu& menu;
};

#endif /* STATEMACHINE_H_ */
