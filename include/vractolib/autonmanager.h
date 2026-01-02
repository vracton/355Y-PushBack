#include "vractolib/drivetrain.h"

#ifndef AUTONMANAGER_H
#define AUTONMANAGER_H

namespace vractolib {
	enum Side {
		LEFT,
		RIGHT
	};

	enum AutonType {
		NONE,
		QUALS,
		SAWP,
		ELIMS,
		SKILLS
	};

	class AutonManager {
		private:
		Side side;
		AutonType autonType;
		int s; //sign for angles

		public:
		AutonManager() : side(LEFT), autonType(NONE), s(1) { };

		Side getSide() const {
			return side;
		}

		Side setSide(Side newSide);

		AutonType getAutonType() const {
			return autonType;
		}

		void selectAuton(AutonType newType) {
			autonType = newType;
		}

		int sign() const {
			return s;
		}
	};
}

#endif 