#include "vractolib/drivetrain.h"

#ifndef AUTONMANAGER_H
#define AUTONMANAGER_H

namespace vractolib {
	enum Alliance {
		RED,
		BLUE
	};
	
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
		Alliance alliance;
		int s; //sign for angles

		public:
		AutonManager() : side(LEFT), autonType(NONE), alliance(RED), s(1) { };

		Side getSide() const {
			return side;
		}

		Side setSide(Side newSide);

		Side toggleSide();

		AutonType getAutonType() const {
			return autonType;
		}

		void selectAuton(AutonType newType) {
			autonType = newType;
		}

		Alliance getAlliance() const {
			return alliance;
		}

		void setAlliance(Alliance newAlliance) {
			alliance = newAlliance;
		}

		Alliance toggleAlliance();

		int sign() const {
			return s;
		}
	};
}

#endif 