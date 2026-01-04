#include "vractolib/autonmanager.h"

namespace vractolib {
	Side AutonManager::setSide(Side newSide) {
		side = newSide;
		s = (side == LEFT) ? 1 : -1;
		return side;
	}

	Side AutonManager::toggleSide() {
		if (side == LEFT) {
			setSide(RIGHT);
		} else {
			setSide(LEFT);
		}
		return side;
	}

	Alliance AutonManager::toggleAlliance() {
		if (alliance == RED) {
			alliance = BLUE;
		} else {
			alliance = RED;
		}
		return alliance;
	}
}