#include "vractolib/solenoid.h"

namespace vractolib {
	void Solenoid::set(bool newState) {
		if (state != newState) {
			state = newState;
			solenoid.set_value(state ? 1 : 0);
		}
	}

	void Solenoid::toggle() {
		set(!state);
	}

	void Solenoid::toggleOn(bool cond) {
		if (cond) {
			toggle();
		}
	}
}