#include "vractolib/autonmanager.h"

namespace vractolib {
	Side AutonManager::setSide(Side newSide) {
		side = newSide;
		s = (side == LEFT) ? 1 : -1;
		return side;
	}
}