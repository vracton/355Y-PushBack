#include "vractolib/solenoid.h"

namespace vractolib {
	Solenoid::Solenoid(pros::adi::DigitalOut &solenoid, bool defaultState) : solenoid(solenoid) {
		this->state = defaultState;
		Solenoid::setDefault(defaultState);
	}

	Solenoid::Solenoid(pros::adi::DigitalOut &solenoid) : Solenoid(solenoid, false) { };

	void Solenoid::setDefault(bool defaultState) {
		this->defaultState = defaultState;
		solenoid.set_value(defaultState);
	}

	void Solenoid::enable() {
		state = !defaultState;
		solenoid.set_value(state);
	}

	void Solenoid::disable() {
		state = defaultState;
		solenoid.set_value(state);
	}

	void Solenoid::toggle() {
		state = !state;
		solenoid.set_value(state);
	}

	void Solenoid::toggleOn(bool cond) {
		if (cond) toggle();
	}
}