#ifndef ANGLES_H
#define ANGLES_H

namespace vunits {
	constexpr double PI = 3.141592653589793;
	constexpr double TAU = 2.0 * PI;

	constexpr double degToRad(double degrees) {
		return degrees * (PI / 180.0);
	}

	constexpr double radToDeg(double radians) {
		return radians * (180.0 / PI);
	}

	constexpr double centiToRad(double centis) {
		return degToRad(centis / 100.0);
	}

	//clamp to [0, 2\pi)
	inline double clampToUnitCircle(double angleRad) {
		if (angleRad < 0) return angleRad + vunits::TAU;
		if (angleRad >= vunits::TAU) return angleRad - vunits::TAU;
		return angleRad;
	}
}

#endif