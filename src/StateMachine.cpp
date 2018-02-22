/*
 * StateMachine.cpp
 *
 *  Created on: Feb 13, 2018
 *      Author: macos
 */

#include "StateMachine.h"
#include "Game_FreeRTOSv2.h"

int StateMachine::timer = 0;

StateMachine::StateMachine(StatusEdit& readyMenu, StatusEdit& runningMenu,
		StatusEdit& pausingMenu, IntegerEdit& buttonToConfig,
		IntegerEdit& gameLength, IntegerEdit& litBeforeHit,
		IntegerEdit& litAfterHit, IntegerEdit& numberOfTargets,
		IntegerEdit& smallTargetPoint, IntegerEdit& mediumTargetPoint,
		IntegerEdit& largeTargetPoint, IntegerEdit& highScoreToggle,
		IntegerEdit& timeLeftToggle, IntegerEdit& soundToggle,
		SaveEdit& saveButton, SimpleMenu& menu) :
		readyMenu(readyMenu), runningMenu(runningMenu), pausingMenu(
				pausingMenu), buttonToConfig(buttonToConfig), gameLength(
				gameLength), litBeforeHit(litBeforeHit), litAfterHit(
				litAfterHit), numberOfTargets(numberOfTargets), smallTargetPoint(
				smallTargetPoint), mediumTargetPoint(mediumTargetPoint), largeTargetPoint(
				largeTargetPoint), highScoreToggle(highScoreToggle), timeLeftToggle(
				timeLeftToggle), soundToggle(soundToggle), saveButton(
				saveButton), menu(menu) {
	currentState = Ready;
	lastState = Ready;
	SetState(Ready);
}

StateMachine::~StateMachine() {
	// TODO Auto-generated destructor stub
}

//StateMachine::timer = 0;

void StateMachine::HandleState(const Event& e) {
	switch (currentState) {
	case Ready:
		HandleStateReady(e);
		break;
	case Pause:
		HandleStatePause(e);
		break;
	case Running0:
		HandleStateRunning0(e);
		break;
	case Running1:
		HandleStateRunning1(e);
		break;
	case Running2:
		HandleStateRunning2(e);
		break;
	case Config:
		HandleStateConfig(e);
		break;
	}
}

void StateMachine::HandleStateReady(const Event& e) {
	switch (e.type) {
	case Event::eEnter:
		menu.setPosition(0);
		menu.event(MenuItem::show);
		break;
	case Event::eExit:
		break;
	case Event::eButton:
		if (e.value == BUTTON_PRESET_1) {
			currentPreset = 1;
			eeprom_buff[0] = 1;
			eeprom_Load(eeprom_buff);
			loadConfig();
			SetState(Ready);
		} else if (e.value == BUTTON_PRESET_2) {
			currentPreset = 2;
			eeprom_buff[0] = 2;
			eeprom_Load(eeprom_buff);
			loadConfig();
			SetState(Ready);
		} else if (e.value == BUTTON_PRESET_3) {
			currentPreset = 3;
			eeprom_buff[0] = 3;
			eeprom_Load(eeprom_buff);
			loadConfig();
			SetState(Ready);
		} else if (e.value == BUTTON_PRESET_4) {
			currentPreset = 4;
			eeprom_buff[0] = 4;
			eeprom_Load(eeprom_buff);
			loadConfig();
			SetState(Ready);
		} else if (e.value == BUTTON_PLAY) {
			SetState(Running0);
		} else if (e.value == BUTTON_BACK) {
			SetState(Config);
		}
		break;
	case Event::eTarget:
		break;
	case Event::eTick:
		break;
	}
}

void StateMachine::HandleStatePause(const Event& e) {
	switch (e.type) {
	case Event::eEnter:
		menu.setPosition(2);
		menu.event(MenuItem::show);
		break;
	case Event::eExit:
		break;
	case Event::eButton:
		if (e.value == BUTTON_PLAY) {
			SetState(lastState);
		} else if (e.value == BUTTON_STOP) {
			SetState(Ready);
		}
		break;
	case Event::eTarget:
		break;
	case Event::eTick:
		break;
	}
}

void StateMachine::HandleStateRunning0(const Event& e) {
	switch (e.type) {
	case Event::eEnter:
		menu.setPosition(1);
		menu.event(MenuItem::show);
		srand(xTaskGetTickCount());
		timeLeft = gameLengthV;
		currentScore = 0;
		soundStart();
		timer = 0;
		currentTarget = rand() % numberOfTargetsV + 1;
		greenLED(currentTarget, true);
		canVal = true;
		break;
	case Event::eExit:
		greenLED(currentTarget, false);
		canVal = false;
		break;
	case Event::eButton:
		if (e.value == BUTTON_PAUSE) {
			SetState(Pause);
		} else if (e.value == BUTTON_STOP) {
			timer = 0;
			SetState(Ready);
		}
		break;
	case Event::eTarget:
		if (e.value == currentTarget) {
			soundHit();
			increaseScore(currentTarget);
			timer = 0;
			SetState(Running1);
		} else
			soundMiss();
		break;
	case Event::eTick:
		timer++;
		timeLeft--;
		if (timeLeft <= 0) {
			soundEnd();
			saveHighScore();
			timer = 0;
			SetState(Ready);
		} else if (timer >= litBeforeHitV) {
			timer = 0;
			SetState(Running1);
		}
		break;
	}
}

void StateMachine::HandleStateRunning1(const Event& e) {
	switch (e.type) {
	case Event::eEnter:
		menu.setPosition(1);
		menu.event(MenuItem::show);
		lastTarget = currentTarget;
		redLED(lastTarget, true);
		while (currentTarget == lastTarget) {
			currentTarget = rand() % numberOfTargetsV + 1;
		}
		greenLED(currentTarget, true);
		canVal = true;
		break;
	case Event::eExit:
		redLED(lastTarget, false);
		greenLED(currentTarget, false);
		canVal = false;
		break;
	case Event::eButton:
		if (e.value == BUTTON_PAUSE) {
			SetState(Pause);
		} else if (e.value == BUTTON_STOP) {
			timer = 0;
			SetState(Ready);
		}
		break;
	case Event::eTarget:
		if (e.value == currentTarget) {
			soundHit();
			increaseScore(currentTarget);
			timer = 0;
			SetState(Running1);
		} else
			soundMiss();
		break;
	case Event::eTick:
		timer++;
		timeLeft--;
		if (timeLeft <= 0) {
			soundEnd();
			saveHighScore();
			timer = 0;
			SetState(Ready);
		} else {
			if (litBeforeHitV < litAfterHitV) {
				if (timer >= litBeforeHitV) {
					timer = 0;
					SetState(Running1);
				}
			} else if (timer >= litAfterHitV) {
				SetState(Running2);
			}
		}
		break;
	}
}

void StateMachine::HandleStateRunning2(const Event& e) {
	switch (e.type) {
	case Event::eEnter:
		menu.setPosition(1);
		menu.event(MenuItem::show);
		greenLED(currentTarget, true);
		canVal = true;
		break;
	case Event::eExit:
		greenLED(currentTarget, false);
		canVal = false;
		break;
	case Event::eButton:
		if (e.value == BUTTON_PAUSE) {
			SetState(Pause);
		} else if (e.value == BUTTON_STOP) {
			timer = 0;
			SetState(Ready);
		}
		break;
	case Event::eTarget:
		if (e.value == currentTarget) {
			soundHit();
			increaseScore(currentTarget);
			timer = 0;
			SetState(Running1);
		} else
			soundMiss();
		break;
	case Event::eTick:
		timer++;
		timeLeft--;
		if (timeLeft <= 0) {
			soundEnd();
			saveHighScore();
			timer = 0;
			SetState(Ready);
		} else if (timer >= litBeforeHitV) {
			timer = 0;
			SetState(Running1);
		}
		break;
	}
}

void StateMachine::HandleStateConfig(const Event& e) {
	switch (e.type) {
	case Event::eEnter:
		buttonToConfig.setValue(currentPreset);
		gameLength.setValue(gameLengthV);
		litBeforeHit.setValue(litBeforeHitV);
		litAfterHit.setValue(litAfterHitV);
		numberOfTargets.setValue(numberOfTargetsV);
		smallTargetPoint.setValue(smallTargetPointV);
		mediumTargetPoint.setValue(mediumTargetPointV);
		largeTargetPoint.setValue(largeTargetPointV);
		highScoreToggle.setValue(highScoreToggleV);
		timeLeftToggle.setValue(timeLeftToggleV);
		soundToggle.setValue(soundToggleV);
		menu.setPosition(3);
		menu.event(MenuItem::show);
		break;
	case Event::eExit:
		break;
	case Event::eButton:
		if (e.value == BUTTON_UP) {
			menu.event(MenuItem::up);
		} else if (e.value == BUTTON_DOWN) {
			menu.event(MenuItem::down);
		} else if (e.value == BUTTON_OK) {
			menu.event(MenuItem::ok);
		} else if (e.value == BUTTON_BACK) {
			menu.event(MenuItem::back);
		} else if (e.value == BUTTON_PLAY) {
			SetState(Ready);
		}
		break;
	case Event::eTarget:
		break;
	case Event::eTick:
		break;
	}
}

void StateMachine::SetState(state newState) {
	Event exit = { Event::eExit, 0 };
	Event enter = { Event::eEnter, 0 };
	HandleState(exit);
	lastState = currentState;
	currentState = newState;
	HandleState(enter);
}
