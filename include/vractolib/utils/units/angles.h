#ifndef ANGLES_H
#define ANGLES_H

#include <cmath>

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

	// wrap to [-\pi, \pi)
	inline double wrapToSignedRadians(double angleRad) {
		angleRad = std::fmod(angleRad + PI, TAU);
		if (angleRad < 0) angleRad += TAU;
		return angleRad - PI;
	}

	// wrap to [-180, 180)
	inline double wrapToSignedDegrees(double angleDeg) {
		angleDeg = std::fmod(angleDeg + 180.0, 360.0);
		if (angleDeg < 0) angleDeg += 360.0;
		return angleDeg - 180.0;
	}

	// find smallest difference between two angles in [-\pi, \pi) given a and b in the range
	inline double angleDiffRadians(double a, double b) {
		double d = b - a;
		if (d >= PI) d -= TAU;
		else if (d < -PI) d += TAU;
		return d;
	}
}

#endif