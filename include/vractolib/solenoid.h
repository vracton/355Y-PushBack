#include "pros/adi.hpp"

#ifndef SOLENOID_H
#define SOLENOID_H

namespace vractolib {
	class Solenoid {
		private:
			pros::adi::DigitalOut &solenoid;
			bool defaultState; //default state for disabled
			bool state;

		public:
			Solenoid(pros::adi::DigitalOut &solenoid);
			Solenoid(pros::adi::DigitalOut &solenoid, bool defaultState);

			void setDefault(bool defaultState);
			void enable();
			void disable();
			void toggle();
			void toggleOn(bool cond);
	};
}

#endif