#ifndef ANGLES_H
#define ANGLES_H

namespace vunits {
	constexpr double PI = 3.141592653589793;

	constexpr double degToRad(double degrees) {
		return degrees * (PI / 180.0);
	}

	constexpr double radToDeg(double radians) {
		return radians * (180.0 / PI);
	}

	constexpr double centiToRad(double centis) {
		return degToRad(centis / 100.0);
	}
}

#endif