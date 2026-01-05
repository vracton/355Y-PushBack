#include "pros/adi.hpp"

#ifndef SOLENOID_H
#define SOLENOID_H

namespace vractolib {
	class Solenoid {
		private:
			pros::adi::DigitalOut &solenoid;
			bool state;

		public:
			Solenoid(pros::adi::DigitalOut &solenoid) : solenoid(solenoid), state(false) { };

			void set(bool newState);
			void toggle();
			void toggleOn(bool cond);
	};
}

#endif