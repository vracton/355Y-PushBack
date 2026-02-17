#include "vractolib/solenoid.h"

namespace vractolib {
	void Solenoid::set(bool newState) {
		state = newState;
		solenoid.set_value(state);
	}

	void Solenoid::toggle() {
		state = !state;
		solenoid.set_value(state);
	}

	void Solenoid::toggleOn(bool cond) {
		if (cond) 
			toggle();
	}
}