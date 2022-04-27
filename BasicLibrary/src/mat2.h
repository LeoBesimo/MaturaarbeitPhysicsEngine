#pragma once

#include "vec2.h"

namespace lge
{
	struct mat2
	{
		vec2 x;
		vec2 y;

		mat2(double x1, double x2, double y1, double y2) :
			x(x1, y1), y(x2, y2)
		{}

		mat2(vec2 x = vec2(), vec2 y = vec2()) :
			x(x), y(y)
		{}

		friend std::ostream& operator<<(std::ostream& os, const mat2& a)
		{
			return os << "First: " << a.x << " Second: " << a.y;
		}

		mat2 operator-(const mat2& a) const
		{
			return mat2(x - a.x, y - a.y);
		}

		mat2 operator+(const mat2& a) const
		{
			return mat2(x + a.x, y - a.y);
		}

		mat2 operator*(const double& a) const
		{
			return mat2(x * a, y * a);
		}

		mat2 operator*(const mat2& a) const
		{
			double p1 = x.x * a.x.x + y.x * a.x.y;
			double p2 = x.x * a.y.x + y.x * a.y.y;
			double p3 = x.y * a.x.x + y.y * a.x.y;
			double p4 = x.y * a.y.x + y.y * a.y.y;
			return mat2(p1, p2, p3, p4);
		}

		vec2 operator*(const vec2& a) const
		{
			return x * a.x + y * a.y;
		}

	};
}