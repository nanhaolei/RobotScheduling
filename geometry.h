#include <array>
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
}