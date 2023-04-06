#ifndef GEOMETRY_H_
#define GEOMETRY_H_

#include <array>
#include "constant.h"
namespace geometry {
	using Vec2 = std::array<double, 2>;
	/* vec2 math */
	inline Vec2 operator-(const Vec2& a, const Vec2& b) { return { {a[0] - b[0], a[1] - b[1]} }; }
	inline Vec2 operator+(const Vec2& a, const Vec2& b) { return { {a[0] + b[0], a[1] + b[1]} }; }
	inline Vec2 operator*(const double& a, const Vec2& b) { return { {a * b[0], a * b[1]} }; }
	inline Vec2 operator*(const Vec2& a, const double& b) { return { {a[0] * b, a[1] * b} }; }
	inline double dot(const Vec2& a, const Vec2& b) { return a[0] * b[0] + a[1] * b[1]; }
	inline double length2(const Vec2& a) { return dot(a, a); }
	inline double length(const Vec2& a) { return sqrt(length2(a)); }
	inline double distance(const Vec2& a, const Vec2& b) {
		double dx = a[0] - b[0];
		double dy = a[1] - b[1];
		return sqrt(dx * dx  + dy * dy);
	}
	inline bool isEq(const Vec2& a, const Vec2& b) {
		if (abs(a[0] - b[0]) < 0.2 && abs(a[1] - b[1]) < 0.2)
			return true;
		else
			return false;
	}
}

#endif